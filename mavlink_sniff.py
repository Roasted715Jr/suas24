from pymavlink import mavutil
from util import mavlink_commands
from time import sleep

using_sitl = True

if using_sitl:
    # conn = mavutil.mavlink_connection("udpin:localhost:14550", autoreconnect=False)
    conn = mavutil.mavlink_connection("udpin:localhost:15550", autoreconnect=False)
else:
    conn = mavutil.mavlink_connection("/dev/ttyS14", baud=57600, autoreconnect=False)

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
print("Waiting for heartbeat")
conn.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (conn.target_system, conn.target_component))

print("Sending heartbeat")
conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

while (True):
    # print(conn.recv_match(blocking=True))
    print(conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True))
