from pymavlink import mavutil
from pymavlink import mavwp
from util import mavlink_commands, connection
from time import sleep
import time

conn = connection.connect()

waypoints = []

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
    print('--- ', conn.target_system, ',', conn.target_component)
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

def uploadmission(aFileName):
    global home_location
    global home_altitude

    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:   
                linearray=line.split('\t')
                ln_seq = int(linearray[0])
                ln_current = int(linearray[1])
                ln_frame = int(linearray[2])
                ln_command = int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_x=float(linearray[8])
                ln_y=float(linearray[9])
                ln_z=float(linearray[10])
                ln_autocontinue = int(linearray[11].strip())
                if(ln_seq == 0):
                    home_location = (ln_x,ln_y)
                    home_altitude = ln_z
                p = mavutil.mavlink.MAVLink_mission_item_message(conn.target_system, conn.target_component, ln_seq, ln_frame,
                                                                ln_command,
                                                                ln_current, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_x, ln_y, ln_z)
                wp.add(p)
                    
    cmd_set_home(home_location,home_altitude)
    msg = conn.recv_match(type = ['COMMAND_ACK'],blocking = True)
    print(msg)
    print('Set home location: {0} {1}'.format(home_location[0],home_location[1]))
    time.sleep(1)

    #send waypoint to airframe
    conn.waypoint_clear_all_send()
    conn.waypoint_count_send(wp.count())

    for i in range(wp.count()):
        msg = conn.recv_match(type=['MISSION_REQUEST'],blocking=True)
        print(msg)
        conn.mav.send(wp.wp(msg.seq))
        print('Sending waypoint {0}'.format(msg.seq))

home_location = None
home_altitude = None
uploadmission("Missions/Shelbourne Park.txt")

#We only want to do this in simulation
using_sitl = True
if using_sitl:
    mavlink_commands.change_mode(conn, "GUIDED")

    mavlink_commands.arm(conn)
    print("Armed")

    sleep(3)

    print("Taking off")
    # takeoff_alt = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True).relative_alt // 1000 + 5
    takeoff_alt = 5
    print(f"Taking off to {takeoff_alt} m")
    conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                                0, 0, 0, 0, 0, 0, takeoff_alt)

    # conn.mav.command_int_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF, 0, 0,
    #                             0, mavutil.mavlink.VTOL_TRANSITION_HEADING_ANY, 0, 0, 0, 0, takeoff_alt)
    # conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF, 0,
    #                             0, 0, 0, 0, 0, 0, takeoff_alt)

    print("Flying to altitude")
    #Altitude is returned in thousandths of feet
    while conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True).relative_alt < (takeoff_alt - 1) * 1000:
        continue

    print("Reached altitude")

    mavlink_commands.change_mode(conn, "AUTO")

#Run the mission until we reach the last loiter waypoint
while (True):
    mission_state = conn.recv_match(type="MISSION_CURRENT", blocking=True)
    # print(mission_state)

    if mission_state.seq == mission_state.total:
        print("Finished mission")
        break

mavlink_commands.change_mode(conn, "GUIDED")

# print("Moving servo")
# conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
#                             9, 1500, 0, 0, 0, 0, 0)

# sleep(3)
