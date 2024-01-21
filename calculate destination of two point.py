from math import radians, degrees, sin, cos, asin, acos, atan2, atan
import numpy as np

# Constants
earth_radius = 6371         # Radius of the Earth in kilometers

#Point A - Baku

baku_lat = 40.465727        # deg
baku_lon = 50.052112        # deg

#Point D - Casablanca - Morocco
casablanca_lat = 33.371466  # deg
casablanca_lon = -7.584294  # deg


# Point C - your location Esenboga Airport
your_lat = 39.936602  # Replace YOUR_LATITUDE with your actual latitude deg
your_lon = 32.850626  # Replace YOUR_LONGITUDE with your actual longitude deg

speed_kmph = 900 # km/h
time_passed = 2 # hour

def great_circle(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    # The central angle (in radians) between two vectors
    Alpha = (acos(sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(lon1 - lon2)))

    # Distance
    Distance = 6371 * Alpha

    return Distance, Alpha  # Distance in Km, angle in radians


def get_azimuth(lat1, lon1, lat2, lon2):
    dLon = lon2 - lon1
    x = cos(radians(lat2)) * sin(radians(dLon))
    y = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(radians(dLon))
    az = np.arctan2(x,y)
    azimuth = np.degrees(az)
    if azimuth < 0:
        azimuth = azimuth + 360
    return azimuth

# Q1: Calculate the azimuth from Baku to your location

distance_baku_to_casablanca = great_circle(baku_lat, baku_lon, casablanca_lat, casablanca_lon)[0]
print("Distance from Baku to Casablanca (B to D):", distance_baku_to_casablanca)


azimuth_bd = get_azimuth(baku_lat, baku_lon, casablanca_lat, casablanca_lon)
print("Q1_a: Azimuth angle of QT4322 flight from Baku to Casablanca (B to D):", azimuth_bd)

azimuth_bc = get_azimuth(baku_lat, baku_lon, your_lat, your_lon)
print("Q1_b: Azimuth angle of QT4322 flight from Baku to your location (B to C):", azimuth_bc)

# Q2: Calculate the distance from Baku to your location
# Assuming your location is (your_lat, your_lon)

distance_baku_to_your_location = great_circle(baku_lat, baku_lon, your_lat, your_lon)[0]
print("Q2: Distance from Baku to your location (B to C):", distance_baku_to_your_location)

# Q3: Calculate the distance from the rendezvous point to Baku
# We know that the support aircraft starts refilling after 3 hours from take-off at Baku.
distance_rendezvous = speed_kmph * time_passed
print("Q3: Distance from the rendezvous point to Baku (A to B):", distance_rendezvous)


# Q4: Solve the spherical triangle using the cosine rule to obtain the distance between your location and the rendezvous point
# We'll use the law of cosines to calculate the distance between your location (C) and the rendezvous point (A)

# The angles are in radians
angleB = np.deg2rad(azimuth_bd - azimuth_bc)
c = distance_rendezvous / earth_radius
a = great_circle(baku_lat,baku_lon,your_lat,your_lon)[1]


cosa = np.cos(a)*np.cos(c)+np.sin(a)*np.sin(c)*np.cos(angleB)
b = np.arccos(cosa)


print("Q4: Distance between your location and the rendezvous point (C to A):", b * 6371)

# Q5: Use sine rule to obtain the inner angle at your location (angle_BAC)
angleC_rad = np.sin(angleB) * np.sin(c) / np.sin(b)
angleC= np.rad2deg(np.arcsin(angleC_rad))
azimuth_cb = get_azimuth(your_lat,your_lon,baku_lat,baku_lon)
# Using the Azimuth angle between your location and Baku (azimuth_bc), calculate the azimuth angle of the runway
azimuth_of_runway = azimuth_cb - angleC
print("Q5: Azimuth angle of the runway at your location:", azimuth_of_runway)



def get_point_at_distance(lat1, lon1, d, bearing, R=6371):
    """
    lat: initial latitude, in degrees
    lon: initial longitude, in degrees
    d: target distance from initial
    bearing: (true) azimuth(heading) in degrees
    R: optional radius of sphere, defaults to mean radius of earth

    Returns new lat/lon coordinate {d}km from initial, in degrees
    """
    lat1 = radians(lat1)
    lon1 = radians(lon1)
    a = radians(bearing)
    lat2 = asin(sin(lat1) * cos(d/R) + cos(lat1) * sin(d/R) * cos(a))
    lon2 = lon1 + atan2(
        sin(a) * sin(d/R) * cos(lat1),
        cos(d/R) - sin(lat1) * sin(lat2)
    )
    return (degrees(lat2), degrees(lon2))


latd, lond = get_point_at_distance(baku_lat, baku_lon, distance_rendezvous, azimuth_bd)

print(f"Destination point coordinates are latitude = {latd} and longitude = {lond}")