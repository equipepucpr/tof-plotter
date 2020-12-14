import json
import math
import matplotlib.pyplot as plt

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

    print("====ToF Sensor Plotter====")
    for robot in robots.keys():
        print("- Robot:", robot)
        print("  - Origin (cm): %.2f, %.2f" % tuple(robots[robot]["origin"]))
        print("  - Size (cm): %.2f, %.2f" % tuple(robots[robot]["size"]))

        xOrigin = robots[robot]["origin"][0]
        yOrigin = robots[robot]["origin"][1]
        xSize = robots[robot]["size"][0]
        ySize = robots[robot]["size"][1]
        plt.plot([xOrigin + xSize/2, xOrigin - xSize/2], [yOrigin - ySize/2, yOrigin - ySize/2], color="black")
        plt.plot([xOrigin + xSize/2, xOrigin - xSize/2], [yOrigin + ySize/2, yOrigin + ySize/2], color="black")
        plt.plot([xOrigin + xSize/2, xOrigin + xSize/2], [yOrigin + ySize/2, yOrigin - ySize/2], color="black")
        plt.plot([xOrigin - xSize/2, xOrigin - xSize/2], [yOrigin + ySize/2, yOrigin - ySize/2], color="black")

        print("  - ToF sensors:")
        robotSensors = robots[robot]["sensors"]
        for sensor in robotSensors:
            if not(sensor[0] in sensors.keys()):
                print(sensor[0], "sensor config not found!")
                exit(1)
            print("    - %s:" % sensor[0])
            print("      - Distance (cm): %.2f" % sensors[sensor[0]]["distance"])
            print("      - FoV (degrees): %.2f" % sensors[sensor[0]]["fov"])
            print("      - Distance from robot origin (cm): %.2f, %.2f" % tuple(sensor[1]))
            print("      - Angle from positive horizontal axis (degrees): %.2f" % sensor[2])

            sensorFov = math.radians(sensors[sensor[0]]["fov"]/2.0)
            sensorDistance = sensors[sensor[0]]["distance"]

            sensorX = sensor[1][0] + xOrigin
            sensorY = sensor[1][1] + yOrigin
            sensorAngle = math.radians(sensor[2])

            x1 = [sensorX, sensorX + sensorDistance * math.cos(sensorAngle + sensorFov)]
            y1 = [sensorY, sensorY + sensorDistance * math.sin(sensorAngle + sensorFov)]

            x2 = [sensorX, sensorX + sensorDistance * math.cos(sensorAngle - sensorFov)]
            y2 = [sensorY, sensorY + sensorDistance * math.sin(sensorAngle - sensorFov)]

            color = Color()
            plt.plot(x1, y1, color=color)
            plt.plot(x2, y2, color=color)
            plt.plot([sensorX], [sensorY], marker='o', markersize=4, color=color)
    input("Press enter to continue...")

except:
    print("invalid JSON!")
    exit(1)

plt.axis('equal')
plt.show()