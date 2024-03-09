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

mavlink_commands.change_mode(conn, "GUIDED")

mavlink_commands.arm(conn)
print("Armed")

sleep(3)

print("Taking off")
conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                            0, 0, 0, 0, 0, 0, 30)

sleep(3)

mavlink_commands.change_mode(conn, "AUTO")

# # mavlink_commands.change_mode(conn, mavutil.mavlink.MAV_MODE_GUIDED_DISARMED)

while (True):
    # print(conn.recv_match(type="MISSION_CURRENT", blocking=True))

    mission_state = conn.recv_match(type="MISSION_CURRENT", blocking=True)
    print(mission_state)

    if mission_state.seq == mission_state.total:
        print("Finished mission")
        break

mavlink_commands.change_mode(conn, "GUIDED")

sleep(3)

mavlink_commands.change_mode(conn, "RTL")

sleep(10)

mavlink_commands.change_mode(conn, "LAND")

# print("Disarming")
# mavlink_commands.disarm(conn)
# print("Disarmed")
