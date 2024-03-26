from pymavlink import mavutil
import time
import math
import numpy as np

CONN_STRING = "udp:localhost:14580"


# CURRENT_COORDINATES = 1000000, 1000000
# PIXELCOORDINATE = 300
# IMAGE_DIMENSIONS = 1331, 693
# TARGET_PIXELS = 500, 200

#imagePath = "SUASTestImage.png"

def retrieveTelemetry(connection):
	data = connection.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
	return data

def calcDist(connection, targetX, targetY):
	FOV = 58.4
	vertical_FOV = 44.6/2
	fov_angle = FOV/2

	#alt = retrieveTelemetry(connection, "ATTITUDE")
	#altitude = alt["relative_alt"]

	#TEST ALT ONLY
	altitude = 80

	y = altitude
	h = np.tan(fov_angle)*y
	#center = [665.5, 346.5]
	centerX = 665.5
	centerY = 346.5

	x_dist = (centerX - targetX)/centerX
	y_dist = (centerY - targetY)/centerY

	x_coords = x_dist*h
	y_coords = y_dist*(np.tan(fov_angle)*y)

	print(x_coords)
	print(y_coords)

	relCoords = [x_coords, y_coords]
    
	return relCoords

def findCoords(connection, targetDist):
	#TelemetryData = retrieveTelemetry(connection, "GLOBAL_POSITION_INT")
	#CurrentCoords = TelemetryData["lat", "lon", "relative_alt"]
	X_distance = targetDist[0]
	Y_distance = targetDist[1]

	#TEST DATA
	lat = -35.36424788
	lon = 149.16426778
	heading_degrees = 335

	# Calculate target coordinates from current coords/Telemetry data and target pixel coordinates

	# lat = TelemetryData["lat"]
	# lon = TelemetryData["lon"]
	# heading_degrees = TelemetryData["hdg"]

	# Earth's radius in meters
	R = 6371000.0

	# Convert latitude and longitude to radians
	lat_rad = math.radians(lat)
	lon_rad = math.radians(lon)

    # Convert heading to radians
	heading_rad = math.radians(heading_degrees)

    # Calculate changes in latitude and longitude
	delta_lat = (Y_distance / R) * math.cos(heading_rad)
	delta_lon = (X_distance / R) * math.sin(heading_rad) / math.cos(lat_rad)

    # Calculate new coordinates
	new_lat = math.degrees(lat_rad + delta_lat)
	new_lon = math.degrees(lon_rad + delta_lon)
	
	target_coords = [new_lat, new_lon]

	return target_coords

if __name__ == "__main__":

	# print("Connecting to Drone")
	#connection = mavutil.mavlink_connection(CONN_STRING)
	# waits for heartbeat message
	#connection.wait_heartbeat()
	#print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
	
	#TEST TEST TEST ONLY
	connection = None

	relativeCoords = calcDist(connection, 1322, 636)
	print(findCoords(connection, relativeCoords))