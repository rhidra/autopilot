from math import pi, sin, cos
from mavros_msgs.msg import CommandCode, Waypoint


"""
Origin of the world as set by the Gazebo simulation
Corresponds to the (0, 0, 0) coordinates for a newly spawned UAV
"""
ORIGIN_LAT = 47.397742
ORIGIN_LON = 8.5455934


"""
localToGlobal()
convert a serie of waypoint in a local coordinate system to a global GPS
coordinate system. Each local coordinate is mapped to an offset relative to
a reference point in the GPS system.
May not be accurate for high offsets distances.

Reference :
- https://stackoverflow.com/questions/7477003/calculating-new-longitude-latitude-from-old-n-meters
- http://www.edwilliams.org/avform.htm#LL

@param waypoints: [[dx: int, dy: int, dz: int]] Offset to the longitude, latitude and altitude (in meters)
@param ref_lat Latitude of the reference point (in degrees)
@param ref_lon Longitude of the reference point (in degrees)
@return [[new_lat, new_lon]]
"""
def local_to_global(waypoints, ref_lat=ORIGIN_LAT, ref_lon=ORIGIN_LON):
    r_earth = 6378137. # WGS-84 ellipsoid parameters (in meters)
    new_wps = []

    for dx, dy, dz in waypoints:
        new_lat = ref_lat + (dy / r_earth) * (180. / pi)
        new_lon = ref_lon + (dx / r_earth) * (180. / pi) / cos(ref_lat * pi / 180.)
        new_wps.append([new_lat, new_lon, dz])

    return new_wps


"""
build_waypoints()
Build a list of MAVLink waypoints to be used in a mission.
The path input should be 3D, in global coordinates.

@param path: [[lat, lon, alt]] 3D path
@return [Waypoints]
"""
def build_waypoints(path):
    assert len(path) > 0, 'The path is empty !'

    waypoints = []
    for lat, lon, alt in path:
        wp = Waypoint()
        wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        wp.command = CommandCode.NAV_WAYPOINT
        wp.is_current = False
        wp.autocontinue = True
        wp.x_lat = lat
        wp.y_long = lon
        wp.z_alt = alt
        waypoints.append(wp)

    wp = Waypoint()
    wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    wp.command = CommandCode.NAV_LAND
    wp.is_current = False
    wp.autocontinue = True
    wp.x_lat = lat
    wp.y_long = lon
    wp.z_alt = alt
    waypoints.append(wp)
    waypoints[0].command = CommandCode.NAV_TAKEOFF
    waypoints[0].is_current = True
