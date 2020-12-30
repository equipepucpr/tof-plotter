import json
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
from matplotlib.patches import Circle
import numpy as np

fig = plt.figure()
ax = fig.gca(projection='3d')

from mpl_toolkits.mplot3d import art3d

def rotation_matrix(d):
    """
    Calculates a rotation matrix given a vector d. The direction of d
    corresponds to the rotation axis. The length of d corresponds to 
    the sin of the angle of rotation.

    Variant of: http://mail.scipy.org/pipermail/numpy-discussion/2009-March/040806.html
    """
    sin_angle = np.linalg.norm(d)

    if sin_angle == 0:
        return np.identity(3)

    d /= sin_angle

    eye = np.eye(3)
    ddt = np.outer(d, d)
    skew = np.array([[    0,  d[2],  -d[1]],
                  [-d[2],     0,  d[0]],
                  [d[1], -d[0],    0]], dtype=np.float64)

    M = ddt + np.sqrt(1 - sin_angle**2) * (eye - ddt) + sin_angle * skew
    return M

def pathpatch_2d_to_3d(pathpatch, z = 0, normal = 'z'):
    """
    Transforms a 2D Patch to a 3D patch using the given normal vector.

    The patch is projected into they XY plane, rotated about the origin
    and finally translated by z.
    """
    if type(normal) is str: #Translate strings to normal vectors
        index = "xyz".index(normal)
        normal = np.roll((1.0,0,0), index)

    normal /= np.linalg.norm(normal) #Make sure the vector is normalised

    path = pathpatch.get_path() #Get the path and the associated transform
    trans = pathpatch.get_patch_transform()

    path = trans.transform_path(path) #Apply the transform

    pathpatch.__class__ = art3d.PathPatch3D #Change the class
    pathpatch._code3d = path.codes #Copy the codes
    pathpatch._facecolor3d = pathpatch.get_facecolor #Get the face color    

    verts = path.vertices #Get the vertices in 2D

    d = np.cross(normal, (0, 0, 1)) #Obtain the rotation vector    
    M = rotation_matrix(d) #Get the rotation matrix

    pathpatch._segment3d = np.array([np.dot(M, (x, y, 0)) + (0, 0, z) for x, y in verts])

def pathpatch_translate(pathpatch, delta):
    """
    Translates the 3D pathpatch by the amount delta.
    """
    pathpatch._segment3d += delta   

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

    return (x_middle, y_middle, plot_radius)

colors = [
    "tab:blue",
    "tab:orange",
    "tab:green",
    "tab:red",
    "tab:purple",
    "tab:brown",
    "tab:pink",
    "tab:gray",
    "tab:olive",
    "tab:cyan"
]

index = -1
def Color():
    global index
    index += 1
    if (index >= len(colors)):
        index = 0
    
    return colors[index]

try:
    config = open("config.json")
except:
    print("config.json not found!")
    exit(1)

try:
    config = json.load(config)
    sensors = config["sensors"]
    robots = config["robots"]
    ground = config["global"]["ground"]

    print("====ToF Sensor Plotter====")
    for robot in robots.keys():
        print("- Robot:", robot)
        print("  - Origin (cm): %.2f, %.2f, %.2f" % tuple(robots[robot]["origin"]))
        print("  - Size (cm): %.2f, %.2f, %.2f" % tuple(robots[robot]["size"]))

        xOrigin = robots[robot]["origin"][0]
        yOrigin = robots[robot]["origin"][1]
        zOrigin = robots[robot]["origin"][2]
        xSize = robots[robot]["size"][0]
        ySize = robots[robot]["size"][1]
        zSize = robots[robot]["size"][2]
        ax.plot([xOrigin + xSize/2, xOrigin - xSize/2], [yOrigin - ySize/2, yOrigin - ySize/2], color="black", zs=(zOrigin - zSize/2))
        ax.plot([xOrigin + xSize/2, xOrigin - xSize/2], [yOrigin + ySize/2, yOrigin + ySize/2], color="black", zs=(zOrigin - zSize/2))
        ax.plot([xOrigin + xSize/2, xOrigin + xSize/2], [yOrigin + ySize/2, yOrigin - ySize/2], color="black", zs=(zOrigin - zSize/2))
        ax.plot([xOrigin - xSize/2, xOrigin - xSize/2], [yOrigin + ySize/2, yOrigin - ySize/2], color="black", zs=(zOrigin - zSize/2))
        
        ax.plot([xOrigin + xSize/2, xOrigin - xSize/2], [yOrigin - ySize/2, yOrigin - ySize/2], color="black", zs=(zOrigin + zSize/2))
        ax.plot([xOrigin + xSize/2, xOrigin - xSize/2], [yOrigin + ySize/2, yOrigin + ySize/2], color="black", zs=(zOrigin + zSize/2))
        ax.plot([xOrigin + xSize/2, xOrigin + xSize/2], [yOrigin + ySize/2, yOrigin - ySize/2], color="black", zs=(zOrigin + zSize/2))
        ax.plot([xOrigin - xSize/2, xOrigin - xSize/2], [yOrigin + ySize/2, yOrigin - ySize/2], color="black", zs=(zOrigin + zSize/2))
        
        ax.plot([xOrigin + xSize/2, xOrigin + xSize/2], [yOrigin - ySize/2, yOrigin - ySize/2], color="black", zs=[zOrigin - zSize/2, zOrigin + zSize/2])
        ax.plot([xOrigin + xSize/2, xOrigin + xSize/2], [yOrigin + ySize/2, yOrigin + ySize/2], color="black", zs=[zOrigin - zSize/2, zOrigin + zSize/2])
        ax.plot([xOrigin - xSize/2, xOrigin - xSize/2], [yOrigin + ySize/2, yOrigin + ySize/2], color="black", zs=[zOrigin - zSize/2, zOrigin + zSize/2])
        ax.plot([xOrigin - xSize/2, xOrigin - xSize/2], [yOrigin - ySize/2, yOrigin - ySize/2], color="black", zs=[zOrigin - zSize/2, zOrigin + zSize/2])

        print("  - ToF sensors:")
        robotSensors = robots[robot]["sensors"]
        for sensor in robotSensors:
            if not(sensor[0] in sensors.keys()):
                print(sensor[0], "sensor config not found!")
                exit(1)
            print("    - %s:" % sensor[0])
            print("      - Distance (cm): %.2f" % sensors[sensor[0]]["distance"])
            print("      - FoV (degrees): %.2f" % sensors[sensor[0]]["fov"])
            print("      - Distance from robot origin (cm): %.2f, %.2f, %.2f" % tuple(sensor[1]))
            print("      - Angles from positive horizontal axis (degrees): %.2f, %.2f" % tuple(sensor[2]))

            sensorFov = math.radians(sensors[sensor[0]]["fov"]/2.0)
            sensorDistance = sensors[sensor[0]]["distance"]

            sensorX = sensor[1][0] + xOrigin
            sensorY = sensor[1][1] + yOrigin
            sensorZ = sensor[1][2] + zOrigin
            sensorAngle = math.radians(sensor[2][0])
            sensorPhi = math.radians(sensor[2][1])

            x1 = [sensorX, sensorX + sensorDistance * math.cos(sensorAngle + sensorFov) * math.cos(sensorPhi)]
            x2 = [sensorX, sensorX + sensorDistance * math.cos(sensorAngle - sensorFov) * math.cos(sensorPhi)]

            y1 = [sensorY, sensorY + sensorDistance * math.sin(sensorAngle + sensorFov)]
            y2 = [sensorY, sensorY + sensorDistance * math.sin(sensorAngle - sensorFov)]

            x3 = [sensorX, sensorX + sensorDistance * math.cos(sensorPhi + sensorFov) * math.cos(sensorAngle)]
            x4 = [sensorX, sensorX + sensorDistance * math.cos(sensorPhi - sensorFov) * math.cos(sensorAngle)]
            z3 = [sensorZ, sensorZ + sensorDistance * math.sin(sensorPhi + sensorFov) * math.cos(sensorAngle)]
            z4 = [sensorZ, sensorZ + sensorDistance * math.sin(sensorPhi - sensorFov) * math.cos(sensorAngle)]
            
            x = [sensorX, (x3[1] + x4[1])/2.0]
            y = [sensorY, (y1[1] + y2[1])/2.0]
            z = [sensorZ, (z3[1] + z4[1])/2.0]

            diff = [(x[1] - x[0]), (y[1] - y[0]), (z[1] - z[0])]
            mod = (diff[0]**2.0 + diff[1]**2.0 + diff[2]**2.0)**(1.0/2.0)
            normal = [var / mod for var in diff]

            color = Color()

            p = Circle((0,0), ((x2[1] - x1[1])**2.0 + (y2[1] - y1[1])**2.0)**(1.0/2.0)/2.0, color = color, fill = False, linestyle='-.')
            ax.add_patch(p)
            pathpatch_2d_to_3d(p, z=0 , normal = normal)
            pathpatch_translate(p, (x[1], y[1], z[1]))

            ax.plot(x1, y1, color=color, zs=z, linestyle='--')
            ax.plot(x2, y2, color=color, zs=z, linestyle='--')
            ax.plot(x3, y, color=color, zs=z3, linestyle='--')
            ax.plot(x4, y, color=color, zs=z4, linestyle='--')
            ax.plot([sensorX], [sensorY], marker='o', markersize=4, color=color, zs=sensorZ)
    input("Press enter to continue...")

    limits = set_axes_equal(ax)
    if (ground["enabled"]):
        p = Circle((limits[0], limits[1]), limits[2], color = ground["color"], fill = False)
        ax.add_patch(p)
        art3d.pathpatch_2d_to_3d(p, z=ground["height"])

except Exception as e:
    print("invalid JSON!")
    print(e)
    exit(1)

plt.tight_layout()
plt.show()