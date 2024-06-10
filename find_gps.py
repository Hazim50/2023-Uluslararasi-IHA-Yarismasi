# Importing the functions pi, sqrt, asin, sin, cos, atan2, and tan from the cmath library.
from cmath import pi, sqrt, asin, sin, cos, atan, tan


def findLL(lat, lon, x_dist, y_dist):
    """
    Given a latitude and longitude, and a distance in the x and y directions, return the latitude and
    longitude of the new point

    :param lat: latitude of the center of the image
    :param lon: longitude of the center of the image
    :param x_dist: The distance in meters to move the point in the x direction
    :param y_dist: The distance in meters that you want to move the point in the y direction
    :return: The latitude and longitude of the point that is x_dist and y_dist away from the given
    latitude and longitude.
    """
    R = 6371000

    dist = sqrt(x_dist * x_dist + y_dist * y_dist)
    lat1 = lat * (pi / 180)
    lon1 = lon * (pi / 180)
    _cos = y_dist / dist
    _sin = x_dist / dist

    latitude = asin(sin(lat1) * cos(dist / R) +
                    cos(lat1) * sin(dist / R) * _cos)
    longitude = (lon1 + atan((_sin * sin(dist / R) * cos(lat1)) /
                             (cos(dist / R) - sin(lat1) * sin(latitude))))

    latitude *= 180 / pi
    longitude *= 180 / pi

    latitude=round(float(latitude.real),8)
    longitude=round(float(longitude.real),8)

    return [latitude, longitude]


def findGPS(lat, lon, alt_to_ground, x, y, heigth, width, hfov, focal_len, roll, pitch):
    """
    The function takes in the drone's GPS coordinates, the distance from the drone to the object, and
    the angle from the drone to the object. It then returns the GPS coordinates of the object

    :param lat: The latitude of the drone
    :param lon: longitude of the drone
    :param alt_to_ground: The altitude of the drone to the ground
    :param x: x coordinate of the pixel
    :param y: The y coordinate of the pixel in the image
    :param heigth: The height of the image in pixels
    :param width: The width of the image in pixels
    :param hfov: horizontal field of view
    :param focal_len: The focal length of the camera
    :param roll: the roll of the drone in degrees
    :param pitch: The angle of the camera relative to the horizon
    :return: The function findLL is being returned.
    """
    v_angle_per_pixel = 2 * atan(heigth / 2 / focal_len) / heigth
    h_angle_per_pixel = hfov / width

    x_to_origin = x - width / 2
    y_to_origin = heigth / 2 - y

    x_angle = h_angle_per_pixel * x_to_origin
    y_angle = v_angle_per_pixel * y_to_origin

    x_dist_to_drone = alt_to_ground * tan((x_angle + roll) * (pi / 180))
    y_dist_to_drone = alt_to_ground * tan((y_angle - pitch) * (pi / 180))

    # Returning the latitude and longitude of the point that is x_dist_to_drone and y_dist_to_drone
    # away from the given latitude and longitude.
    return findLL(lat, lon, x_dist_to_drone, y_dist_to_drone)