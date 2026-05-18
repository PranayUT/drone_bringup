#!/usr/bin/env python3
import argparse
import os
import sys
import time

import cv2


def _percentiles(values, ps):
    if not values:
        return {p: None for p in ps}
    xs = sorted(values)
    n = len(xs)
    out = {}
    for p in ps:
        if n == 1:
            out[p] = xs[0]
            continue
        k = (p / 100.0) * (n - 1)
        f = int(k)
        c = min(f + 1, n - 1)
        if c == f:
            out[p] = xs[f]
        else:
            out[p] = xs[f] + (xs[c] - xs[f]) * (k - f)
    return out


def _patch_torchvision_nms_for_jetson():
    """Use Ultralytics TorchNMS when torchvision CUDA ops lack sm_72 kernels."""
    try:
        import torch
        from torchvision.ops import nms

        boxes = torch.rand(2, 4, device="cuda")
        scores = torch.rand(2, device="cuda")
        nms(boxes, scores, 0.5)
    except Exception:
        import types

        from ultralytics.utils.nms import TorchNMS

        ops = types.SimpleNamespace(
            nms=lambda boxes, scores, iou_threshold: TorchNMS.nms(
                boxes, scores, iou_threshold
            )
        )
        tv = types.ModuleType("torchvision")
        tv.ops = ops
        sys.modules["torchvision"] = tv
        sys.modules["torchvision.ops"] = ops


def _build_gst_pipeline(sensor_id, width, height, fps):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} "
        f"! video/x-raw(memory:NVMM),width={width},height={height},"
        f"framerate={fps}/1,format=NV12 "
        f"! nvvidconv flip-method=0 "
        f"! video/x-raw,width={width},height={height},format=BGRx "
        "! videoconvert "
        "! video/x-raw,format=BGR "
        "! appsink drop=1"
    )


def main():
    ap = argparse.ArgumentParser(
        description="Continuously benchmark a YOLO model on an image or CSI camera."
    )
    ap.add_argument("--weights", default="best-v1.pt", help="Path to .pt / .engine weights")
    ap.add_argument(
        "--task",
        default="",
        metavar="TASK",
        help="Ultralytics task if needed (e.g. segment for seg models). Omit for auto.",
    )
    ap.add_argument("--image", default="bus.jpg", help="Path to input image (image mode)")
    ap.add_argument(
        "--source",
        choices=["image", "camera"],
        default="image",
        help="Input source: 'image' (static file) or 'camera' (CSI via GStreamer)",
    )
    # CSI camera args — defaults match camera.launch.py
    ap.add_argument("--sensor-id", type=int, default=0, help="nvarguscamerasrc sensor-id")
    ap.add_argument("--cam-width",  type=int, default=1920, help="Camera capture width")
    ap.add_argument("--cam-height", type=int, default=1080, help="Camera capture height")
    ap.add_argument("--cam-fps",    type=int, default=60,   help="Camera framerate")

    ap.add_argument("--imgsz", type=int, default=640, help="Inference image size")
    ap.add_argument(
        "--device",
        default="0",
        help="Device for inference (e.g. '0', 'cpu'). Default: 0 (GPU)",
    )
    ap.add_argument("--warmup", type=int, default=5, help="Warmup iterations")
    ap.add_argument(
        "--iters",
        type=int,
        default=0,
        help="Iterations to run (0 = run forever)",
    )
    ap.add_argument(
        "--save-every",
        type=int,
        default=0,
        help="Save an annotated image every N iters (0 = never)",
    )
    ap.add_argument(
        "--out-dir",
        default="runs/bench",
        help="Where to write annotated images (if enabled)",
    )
    ap.add_argument(
        "--half",
        action="store_true",
        help="Request FP16 (only used on CUDA; ignored/disabled on CPU)",
    )
    ap.add_argument(
        "--half-default",
        action="store_true",
        default=True,
        help="Enable FP16 by default (still only used on CUDA)",
    )
    ap.add_argument(
        "--stream",
        action="store_true",
        help="Use streaming generator API (usually lower overhead)",
    )
    ap.add_argument(
        "--delay",
        type=float,
        default=0.0,
        help="Seconds to sleep after each inference (default: 0, no sleep)",
    )
    ap.add_argument(
        "--preview",
        action="store_true",
        help="Show annotated YOLO output in a live window",
    )
    ap.add_argument(
        "--infer-every",
        type=int,
        default=1,
        metavar="N",
        help=(
            "Camera only: run YOLO every N frames (default 1). "
            "Preview redraws the last result on each new frame (no overlay flashing); "
            "detections update every N frames. Lowers CPU vs inferring every frame."
        ),
    )
    args = ap.parse_args()

    infer_every = max(1, int(args.infer_every))
    if infer_every > 1 and args.source != "camera":
        print("WARNING: --infer-every > 1 applies only to --source camera; using 1.", file=sys.stderr)
        infer_every = 1

    weights = os.path.expanduser(args.weights)
    if not os.path.exists(weights):
        print(f"ERROR: weights not found: {weights}", file=sys.stderr)
        return 2

    # --- open source ---
    cap = None
    static_img = None

    if args.source == "camera":
        pipeline = _build_gst_pipeline(
            args.sensor_id, args.cam_width, args.cam_height, args.cam_fps
        )
        print(f"GStreamer pipeline: {pipeline}")
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print("ERROR: failed to open CSI camera via GStreamer", file=sys.stderr)
            return 2
        # grab one frame to confirm it works
        ok, frame = cap.read()
        if not ok or frame is None:
            print("ERROR: camera opened but first frame read failed", file=sys.stderr)
            cap.release()
            return 2
        static_img = None  # will grab fresh each iteration
        print(
            f"Camera opened: sensor_id={args.sensor_id} "
            f"{args.cam_width}x{args.cam_height}@{args.cam_fps}fps"
        )
    else:
        image_path = os.path.expanduser(args.image)
        if not os.path.exists(image_path):
            print(f"ERROR: image not found: {image_path}", file=sys.stderr)
            return 2
        static_img = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if static_img is None:
            print(f"ERROR: failed to read image: {image_path}", file=sys.stderr)
            return 2

    _patch_torchvision_nms_for_jetson()
    from ultralytics import YOLO

    if args.task:
        model = YOLO(weights, task=args.task)
    else:
        model = YOLO(weights)
    want_half = bool(args.half or args.half_default)
    is_cpu = str(args.device).lower() == "cpu"
    use_half = want_half and not is_cpu
    print(
        f"model={os.path.basename(weights)} task={args.task or 'auto'} source={args.source} "
        f"device={args.device} half={'on' if use_half else 'off'} delay={args.delay}s imgsz={args.imgsz}"
        + (f" infer_every={infer_every}" if args.source == "camera" else "")
    )

    def get_frame():
        if cap is not None:
            ok, f = cap.read()
            if not ok or f is None:
                raise RuntimeError("Camera read failed")
            return f
        return static_img

    def predict_on(img):
        if args.stream:
            return next(
                model.predict(
                    source=img,
                    imgsz=args.imgsz,
                    device=args.device,
                    half=use_half,
                    verbose=False,
                    stream=True,
                )
            )
        return model.predict(
            source=img,
            imgsz=args.imgsz,
            device=args.device,
            half=use_half,
            verbose=False,
        )[0]

    def predict_once():
        return predict_on(get_frame())

    # warmup (always full-rate inference)
    for _ in range(max(args.warmup, 0)):
        predict_once()

    if args.save_every > 0:
        os.makedirs(args.out_dir, exist_ok=True)

    times_ms = []
    start_wall = time.perf_counter()
    i = 0
    last_report = start_wall
    last_res = None  # latest YOLO result; redraw on each camera frame to avoid preview flashing

    try:
        while True:
            img = get_frame()
            do_infer = args.source != "camera" or infer_every == 1 or (i % infer_every == 0)

            if do_infer:
                t0 = time.perf_counter()
                res = predict_on(img)
                t1 = time.perf_counter()
                times_ms.append((t1 - t0) * 1000.0)
                last_res = res

            i += 1

            if args.preview or (args.save_every > 0 and i % args.save_every == 0):
                annotated = last_res.plot(img=img) if last_res is not None else None
                if args.preview:
                    vis = annotated if annotated is not None else img
                    cv2.imshow("YOLO Preview", vis)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                if args.save_every > 0 and i % args.save_every == 0 and annotated is not None:
                    out_path = os.path.join(args.out_dir, f"iter_{i:06d}.jpg")
                    cv2.imwrite(out_path, annotated)

            now = time.perf_counter()
            if now - last_report >= 2.0 and times_ms:
                window = times_ms[-200:]
                p = _percentiles(window, [50, 90, 95, 99])
                avg = sum(window) / len(window)
                fps = 1000.0 / avg if avg > 0 else 0.0
                total_s = now - start_wall
                print(
                    f"iter={i} avg={avg:.2f}ms fps={fps:.2f} "
                    f"p50={p[50]:.2f} p90={p[90]:.2f} p95={p[95]:.2f} p99={p[99]:.2f} "
                    f"total={total_s:.1f}s"
                )
                last_report = now

            if args.iters > 0 and i >= args.iters:
                break

            if args.delay and args.delay > 0:
                time.sleep(args.delay)

    except KeyboardInterrupt:
        pass
    finally:
        if cap is not None:
            cap.release()
        if args.preview:
            cv2.destroyAllWindows()

    if times_ms:
        p = _percentiles(times_ms, [50, 90, 95, 99])
        avg = sum(times_ms) / len(times_ms)
        fps = 1000.0 / avg if avg > 0 else 0.0
        print(
            "\nSummary:"
            f" iters={len(times_ms)} avg={avg:.2f}ms fps={fps:.2f}"
            f" p50={p[50]:.2f} p90={p[90]:.2f} p95={p[95]:.2f} p99={p[99]:.2f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
