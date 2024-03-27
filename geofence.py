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

mission = json.load(open("Missions/competition.json"))

print(mission["geofence"])
send_poly_geofence(conn, mission["geofence"])
