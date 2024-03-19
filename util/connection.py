from pymavlink import mavutil

connection_string = "udpin:localhost:15550"

def connect():
	return mavutil.mavlink_connection(connection_string, autoreconnect=False)
