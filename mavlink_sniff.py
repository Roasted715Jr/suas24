from pymavlink import mavutil
from util import connection
from time import sleep
import sys

conn = connection.connect()

# Wait for the first heartbeat
print("Waiting for heartbeat")
conn.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (conn.target_system, conn.target_component))

print("Sending heartbeat")
conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

message_type = None
if len(sys.argv) != 0:
    message_type = sys.argv[1]
while (True):
    if message_type:
        print(conn.recv_match(type=message_type, blocking=True))
    else:
        print(conn.recv_match(blocking=True))
