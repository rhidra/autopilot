import xml.etree.ElementTree as ET

"""
getWorld()
Return the world as parsed in the iris.world file
@return (start, goal, obstacles) Variable as they should be used in the RRT algorithm
"""
def getWorld():
    obs_w = 2 # Obstacle width
    obs_h = 2 # Obstacle height
    obs_d = 5 # Obstacle depth
    obstacles = []
    world = ET.parse('/home/rhidra/src/Firmware/Tools/sitl_gazebo/worlds/iris.world').getroot()[0]
    for include in world.findall('include'):
        type = include.find('uri').text[8:] #text="model://obstacle"
        if type == 'obstacle':
            x,y,z,rx,ry,rz = include.find('pose').text.split(' ')
            # We need to correct the origin,
            # in RRT, obstacle origin is the corner of the cuboid
            # in Gazebo, obstacle origin is the center of the bottom face
            obstacles.append((float(x)-obs_w/2, float(y)-obs_h/2, float(z), obs_w, obs_h, obs_d))
        elif type == 'goal':
            x,y,z,rx,ry,rz = include.find('pose').text.split(' ')
            goal = (float(x),float(y),float(z)+3) # z+3 because the goal is on top of the cylinder
        elif type == 'iris':
            x,y,z,rx,ry,rz = include.find('pose').text.split(' ')
            start = (float(x),float(y),float(0))
    return (start, goal, obstacles)
