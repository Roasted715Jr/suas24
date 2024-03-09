from pymavlink import mavutil
import time

CONN_STRING = "udp:127.0.0.1:14580"  #"udp:localhost:5760"

# print("Connecting to Drone")
connection = mavutil.mavlink_connection(CONN_STRING)
# waits for heartbeat message
# connection.send_heartbeat()
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

# Starts script
def main():
    #while True:
    #    print(print_datastream())
    #change_mode(connection, 'AUTO')
    
    arm_and_takeoff(connection, 50)
    
    #coords_goto(connection, [-3536424788, 14916426778])
    
    #init_check()
    #get_msg(None)
    #change_mode("GUIDED")
    # MIGHT NEED 45 SECONDS OF WAIT BEFORE DRONE INITS
    # if init_check():
    #     print("Systems check passed")
    #     change_mode("GUIDED")
    #     arm_and_takeoff(connection)
    #     print("Arm and takeoff successful")
    #     vtol_switch_fw(connection)
        
    # else:
    #     print("Systems check failed")

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

def arm_and_takeoff(connection, takeoff_alt):
    connection.mav.command_long_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    connection.motors_armed_wait()
    print("Drone is armed")
    time.sleep(1)
    # CAN USE SPECIFIC VTOL TAKEOFF COMMAND
    change_mode(connection, "GUIDED")
    time.sleep(1)
    connection.mav.command_long_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, takeoff_alt)
    # while True:
    #     alt = connection.recv_match(type="ATTITUDE", blocking=False)["relative_alt"]
    #     if alt >= takeoff_alt:
    #         break
    print("takeoff complete")

def disarm(connection):
    connection.mav.command_long_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    connection.motors_disarmed_wait()

def change_mode(connection, mode):
    if mode not in connection.mode_mapping():
        print("mode not recognized")
    else:
        mode_id = connection.mode_mapping()[mode]
        connection.set_mode(mode_id)
        #connection.mav.command_long_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mode_id, 0, 0, 0, 0, 0, 0)
        print(f"Mode switched to {mode}")

def vtol_switch_fw(connection):
    connection.mav.command_long_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSISTION, 0, 4, 0, 0, 0, 0, 0, 0)
    print("Successfully switched to Fixed Wing")

def vtol_switch_mc(connection):
    connection.mav.command_long_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSISTION, 0, 3, 0, 0, 0, 0, 0, 0)
    print("Successfully switched to Multicopter")

if __name__ == "__main__":
    main()