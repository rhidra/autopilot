from math import pi, sin, cos

"""
localToGlobal()
convert a serie of waypoint in a local coordinate system to a global GPS
coordinate system. Each local coordinate is mapped to an offset relative to
a reference point in the GPS system.
May not be accurate for high offsets distances.

Reference :
- https://stackoverflow.com/questions/7477003/calculating-new-longitude-latitude-from-old-n-meters
- http://www.edwilliams.org/avform.htm#LL

@param ref_lat Latitude of the reference point (in degrees)
@param ref_lon Longitude of the reference point (in degrees)
@param waypoints: [[dx: int, dy: int, dz: int]] Offset to the longitude, latitude and altitude (in meters)
@return [[new_lat, new_lon]]
"""
def localToGlobal(ref_lat, ref_lon, waypoints):
    r_earth = 6378137. # WGS-84 ellipsoid parameters (in meters)
    new_wps = []

    for dx, dy, dz in waypoints:
        new_lat = ref_lat + (dy / r_earth) * (180. / pi)
        new_lon = ref_lon + (dx / r_earth) * (180. / pi) / cos(ref_lat * pi / 180.)
        # print('[{}, {}] => [{}, {}] => [{}, {}]'.format(dx, dy,
        #                                                 format((dy / r_earth) * (180/pi), '.60g'),
        #                                                 format((dx / r_earth) / cos(ref_lat * pi / 180.), '.60g'), format(new_lat, '.60g'), format(new_lon, '.60g')))
        new_wps.append([new_lat, new_lon, dz])

    return new_wps

def f(ref_lat, ref_lon, x, y):
    r_earth = 6378137. # WGS-84 ellipsoid parameters (in meters)
    new_lat = ref_lat + (y / r_earth) * (180. / pi)
    new_lon = ref_lon + (x / r_earth) * (180. / pi) / cos(ref_lat * pi / 180.)

    return new_lat, new_lon
