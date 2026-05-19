import socket
import time
import waypoints_pb2

msg = waypoints_pb2.WaypointList()
msg.timestamp_ns = time.time_ns()

points = [
    (0, -2, 0.78),
]
for lat, lon, yaw in points:
    wp = msg.waypoints.add()
    wp.lat, wp.lon, wp.yaw = lat, lon, yaw

payload = msg.SerializeToString()

# TCP — recommended for a waypoint list (delivery matters)
DOG_ADDR = ("192.168.2.26", 9000)
with socket.create_connection(DOG_ADDR) as s:
    # length-prefix so the dog knows where the message ends
    s.sendall(len(payload).to_bytes(4, "big") + payload)