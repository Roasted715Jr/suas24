from pymavlink import mavutil

armed = False

def arm(conn):
	print("Arming")
	conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,
								0, 0, 0, 0, 0, 0)
	
	# response = conn.recv_match("COMMAND_ACK", blocking=True)
	# while not response or response.command != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
	# 	continue

	# return response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED

def disarm(conn):
	conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0,
								0, 0, 0, 0, 0, 0)

	#There's an error reading the response from this
	# response = conn.recv_match("COMMAND_ACK", blocking=True)
	# while not response or response.command != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
	# 	continue

	# return response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED

def change_mode(conn, mode: str):
	# conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, mode, 0, 0,
    #                         	0, 0, 0, 0, 0)
	print("Changing mode to " + mode)
	conn.set_mode(conn.mode_mapping()[mode])

	# response = conn.recv_match("COMMAND_ACK", blocking=True)
	# while not response or response.command != mode:
	# 	continue

	# return response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
