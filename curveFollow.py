from robodk.robodk import *
from robolink.robolink import *
import time

import csv

import dynamixel

import os

import random

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def translate_to_dynamixel(value):
    k = translate(value, -150, 150, 0, 1023)
    return int(k)

# A function to determine if any actuator is moving
# def actuators_moving(actuators):
#     for actuator in actuators:
#         if actuator.cache[dynamixel.defs.REGISTER_AX12["Moving"]]:
#             return True
#     return False


# The number of Dynamixels on our bus.
nServos = 5

# Set your serial port accordingly.
if os.name == "posix":
    portName = "/dev/ttyUSB0"
else:
    portName = "COM6"

# Default baud rate of the USB2Dynamixel device.
baudRate = 1000000

# Connect to the serial port

serial = dynamixel.SerialStream(port=portName, baudrate=baudRate, timeout=1)
net = dynamixel.dynamixel_network.DynamixelNetwork(serial)
net.scan(1, nServos)

# A list to hold the dynamixels
myActuators = list()

print "Scanning for Dynamixels...",
for dyn in net.get_dynamixels():
    print dyn.id,
    myActuators.append(net[dyn.id])
print "...Done"

# Set the default speed and torque
for actuator in myActuators:
    actuator.moving_speed = 20
    actuator.synchronized = True
    actuator.torque_enable = True
    actuator.torque_limit = 800
    actuator.max_torque = 800

myActuators[0].goal_position = translate_to_dynamixel(0)
myActuators[1].goal_position = translate_to_dynamixel(0)
myActuators[2].goal_position = translate_to_dynamixel(0)
myActuators[3].goal_position = translate_to_dynamixel(0)
myActuators[4].goal_position = translate_to_dynamixel(90)

net.synchronize()

# while(actuators_moving(myActuators)):{}
time.sleep(5)
#file names:
# DE3CB2~1.CSV

#depth_109.csv
#depth_2486.csv

#Import csv file(containning points) to array
POINTS = []
NUM_POINTS = 0
with open('/home/fongsu/PycharmProjects/curvefollow/csvs/cloud_2.csv') as csvfile:
    reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC) # change contents to floats
    for row in reader: # each row is a list
        POINTS.append(row)
        NUM_POINTS = NUM_POINTS + 1
# Default parameters:
P_START = POINTS[0]   # Start point with respect to the robot base frame

# Initialize the RoboDK API
RDK = Robolink()

# Automatically delete previously generated items (Auto tag)
#list_names = RDK.ItemList()  # list all names
#for item_name in list_names:
   # if item_name.startswith('Auto'):
       # RDK.Item(item_name).Delete()

# Promt the user to select a robot (if only one robot is available it will select that robot automatically)
robot = RDK.ItemUserPick('Select a robot', ITEM_TYPE_ROBOT)
# Set the initial joints of the robot
#robot.setJoints([0.000000, -90.000000, 70.000000, 0.000000, 112.000000, 0.000000])
#robot.setPose(TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 180]))

# A reference frame of the object(already set)
frame = RDK.Item('Object')


# Abort if the user hits Cancel
if not robot.Valid():
    quit()

# Retrieve the robot reference frame
reference = robot.Parent()

# Use the robot base frame as the active reference
robot.setPoseFrame(reference)
robot.setJoints([0,0,0,0,90,0])
#tool.setPose(transl(474.430,-109.000,607.850)*rotx(-69.282)*roty(69.282)*rotz(-69.282))

# get the current orientation of the robot (with respect to the active reference frame and tool frame)
pose_ref = robot.Pose()

object_curve = RDK.AddCurve(POINTS)
object_curve.setParent(frame)

path_settings = RDK.AddMillingProject("AutoCurveFollow settings")
path_settings.setSpeed(5)
# path_settings.setSpeed(10)
# path_settings.setRounding(5)
prog, status = path_settings.setMachiningParameters(part=object_curve)
robot.setSpeed(5)
robot.setSpeedJoints(5)
# path_settings.setSpeed(10)


prog.RunProgram()

while True:
    # get the current robot joints
    k = robot.Joints().list()
    print(k[0], "1")
    print(k[1], "2")
    print(k[2], "3")
    print(k[3], "4")
    print(k[4], "5")

    myActuators[0].goal_position = translate_to_dynamixel(k[0])
    myActuators[1].goal_position = translate_to_dynamixel(k[1])
    myActuators[2].goal_position = translate_to_dynamixel(k[2])
    myActuators[3].goal_position = translate_to_dynamixel((-1 * k[3]))
    myActuators[4].goal_position = translate_to_dynamixel(k[4])

    net.synchronize()
    time.sleep(0.1)

    # joints = tr(robot.Joints())
    # joints = joints.rows[0]
    # print('Current robot joints:')
    # print(joints)

# Done, stop program execution
#quit()
