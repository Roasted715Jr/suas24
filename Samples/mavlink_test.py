from pymavlink import mavutil

the_connection = mavutil.mavlink_connection("/dev/ttyS14", baud=57600, autoreconnect=False)

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
print("Waiting for heartbeat")
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

print("Sending heartbeat")
the_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

while (True):
    print(the_connection.recv_match(blocking=True))
