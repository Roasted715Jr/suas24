from pymavlink import mavutil
from enum import Enum
from ctypes import c_uint8, c_uint16, c_int32

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

def encode_mission_item(conn, cmd: c_uint16, seq: c_uint16, param1: float, param2: float, param3: float, param4: float, x: c_int32, y: c_int32, z: float, mission_type: c_uint8 = mavutil.mavlink.MAV_MISSION_TYPE_MISSION, frame: c_uint8 = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT):
	return conn.mav.mission_item_int_encode(conn.target_system, conn.target_component,
						   seq, frame, cmd,
						   0, 1, #Current, autocontinue
						   param1, param2, param3, param4,
						   x, y, z,
						   mission_type)

def enable_alt_geofence(connection):
	#pranav

	# not sure how to set the actual altitude for the fence

	connection.mav.command_long_send(connection.target_system, connection.target_component, 
									 mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE, 1, 1, 0, 0, 0, 0, 0)
	
	print("altitude fence enabled")

def send_poly_geofence(conn, points):
	conn.mav.mission_clear_all_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
	conn.mav.mission_count_send(conn.target_system, conn.target_component, len(points), mavutil.mavlink.MAV_MISSION_TYPE_FENCE)

	for i in range(len(points)):
		msg = conn.recv_match(type=['MISSION_REQUEST'],blocking=True)
		print(msg)
		point = points[i]
		print(f"Sending point {i}")
		conn.mav.send(encode_mission_item(conn, mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, i,
									len(points), 0, 0, 0, int(point[0] * 1e7), int(point[1] * 1e7), 0, mission_type = mavutil.mavlink.MAV_MISSION_TYPE_FENCE))


def enable_poly_geofence(connection):
	#pranav

	# not sure how to set the actual geofences
	# should I get the fence coordinates from a txt file or add them in a list locally

	connection.mav.command_long_send(connection.target_system, connection.target_component, 
									 mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE, 1, 4, 0, 0, 0, 0, 0)
	
	# connection.mav.command_int_send(connection.target_system, connection.target_component, 
	#                                 mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, 5, geofence, 0, 0, 0, 0, 0)
	print("enabled polygon fence")



# disabling geofences should be straightforward

def disable_alt_geofence(connection):
	#pranav
	connection.mav.command_long_send(connection.target_system, connection.target_component, 
									 mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE, 0, 1, 0, 0, 0, 0, 0)
	print("disabled altitude fence")


def disable_poly_geofence(connection):
	#pranav
	connection.mav.command_long_send(connection.target_system, connection.target_component, 
									 mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE, 0, 4, 0, 0, 0, 0, 0)
	print("disabled polygon fence")


def vtol_switch_fw(connection):
    connection.mav.command_long_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSISTION, 0, 4, 0, 0, 0, 0, 0, 0)
    print("Successfully switched to Fixed Wing")

def vtol_switch_mc(connection):
    connection.mav.command_long_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSISTION, 0, 3, 0, 0, 0, 0, 0, 0)
    print("Successfully switched to Multicopter")

def coords_goto(connection, coordinates):
    print(f"Going to given coordinates, {coordinates}")
    
    connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(connection.target_system, connection.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_INT, 0 ,coordinates[0], coordinates[1], 50, 0, 0, 0, 0, 0, 0, 0, 0, 0))

    print(f"At target coordinates, {coordinates}")

def init_check():
    PASSED = False
    msg = connection.recv_match(blocking=False)
    print(msg)
    return PASSED

def print_datastream():
    msg = connection.recv_match(blocking=True)
    return msg

def get_msg(type):
    #while True:
    msg = connection.recv_match(type=type, blocking=True)
    print(msg)
