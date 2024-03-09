import math

def calculate_target_coordinates(lat, lon, heading_degrees, Y_distance, X_distance):
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

# Example usage
lat = -35.36424788
lon = 149.16426778
heading_degrees = 335
Y_distance = 10.0  # Adjust as needed
X_distance = 5.0   # Adjust as needed

result = calculate_target_coordinates(lat, lon, heading_degrees, Y_distance, X_distance)
print(result)
