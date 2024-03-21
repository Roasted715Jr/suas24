from pymavlink import mavutil
from pymavlink import mavwp
from util import connection
from util.mavlink_commands import *
from time import sleep
import time
import json
from ctypes import c_uint8, c_uint16, c_int32

conn = connection.connect()

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
print("Waiting for heartbeat")
conn.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (conn.target_system, conn.target_component))

print("Sending heartbeat")
conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

wp = mavwp.MAVWPLoader()

#https://discuss.ardupilot.org/t/mission-upload-using-pymavlink-in-python3/60504
def cmd_set_home(home_location, altitude):
    # print('--- ', conn.target_system, ',', conn.target_component)
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        1, # set position
        0, # param1
        0, # param2
        0, # param3
        0, # param4
        home_location[0], # lat
        home_location[1], # lon
        altitude) 

# def uploadmission(aFileName):
def uploadmission():
    global mission
    home_location = mission["home_coords"]
    home_altitude = mission["home_alt_m"]
    waypoints = []

	#Add home and takeoff waypoints first
    waypoints.append(encode_mission_item(conn, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,
                                        0, 0, 0, 0, int(home_location[0] * 1e7), int(home_location[1] * 1e7), home_altitude, frame = mavutil.mavlink.MAV_FRAME_GLOBAL_INT))
    
    waypoints.append(encode_mission_item(conn, mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF, len(waypoints),
                                        0, mavutil.mavlink.VTOL_TRANSITION_HEADING_NEXT_WAYPOINT, 0, 0,
                                        int(home_location[0] * 1e7), int(home_location[1] * 1e7), mission["takeoff_alt_m"]))

    #Make sure we're transitioned back to horizontal (for when we restart the mission)
    waypoints.append(encode_mission_item(conn, mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, len(waypoints),
                                        4, 0, 0, 0,
                                        int(home_location[0] * 1e7), int(home_location[1] * 1e7), mission["takeoff_alt_m"]))

    for point in mission["waypoints"]:
        print(f"Adding waypoint: {point}")
        waypoints.append(encode_mission_item(conn, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, len(waypoints),
                                                0, mission["waypoint_acc_m"], 0, 0, int(point[0] * 1e7), int(point[1] * 1e7), point[2]))

    #Transition to vertical for payloads
    point = mission["loiter_point"]
    waypoints.append(encode_mission_item(conn, mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, len(waypoints),
                                        3, 0, 0, 0,
                                        int(point[0] * 1e7), int(point[1] * 1e7), point[2]))

    #Loiter point to let the Pi know we're finished with the mission
    waypoints.append(encode_mission_item(conn, mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, len(waypoints),
                                        0, 0, 25, 0, int(point[0] * 1e7), int(point[1] * 1e7), point[2]))

    cmd_set_home(home_location,home_altitude)
    msg = conn.recv_match(type = ['COMMAND_ACK'],blocking = True)
    print(msg)
    print('Set home location: {0} {1}'.format(home_location[0],home_location[1]))
    time.sleep(1)

    #send waypoint to airframe
    conn.waypoint_clear_all_send()
    conn.waypoint_count_send(len(waypoints))
    
    for waypoint in waypoints:
        msg = conn.recv_match(type=['MISSION_REQUEST'],blocking=True)
        print(msg)
        conn.mav.send(waypoint)
        print('Sending waypoint {0}'.format(msg.seq))
        print(waypoint)

mission = json.load(open("Missions/shelbourne.json"))

uploadmission()

#We only want to do this in simulation
using_sitl = True
if using_sitl:
    change_mode(conn, "GUIDED")

    arm(conn)
    print("Armed")

    sleep(3)

    change_mode(conn, "AUTO")

#Run the mission until we reach the last loiter waypoint
while (True):
    mission_state = conn.recv_match(type="MISSION_CURRENT", blocking=True)
    # print(mission_state)

    if mission_state.seq == mission_state.total:
        print("Finished mission")
        break

change_mode(conn, "GUIDED")

# print("Moving servo")
# conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
#                             9, 1500, 0, 0, 0, 0, 0)

# sleep(3)
