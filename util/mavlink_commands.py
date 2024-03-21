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
