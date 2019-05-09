import dynamixel

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
# Establish a serial connection to the dynamixel network.
# This usually requires a USB2Dynamixel
serial = dynamixel.SerialStream(port='/dev/ttyUSB0',
                                    baudrate='1000000',
                                    timeout=2)
# Instantiate our network object
net = dynamixel.DynamixelNetwork(serial)

Dynamixel1 = dynamixel.Dynamixel(1, net)
Dynamixel2 = dynamixel.Dynamixel(2, net)
Dynamixel3 = dynamixel.Dynamixel(3, net)
Dynamixel4 = dynamixel.Dynamixel(4, net)
Dynamixel5 = dynamixel.Dynamixel(5, net)


net._dynamixel_map[1] = Dynamixel1
net._dynamixel_map[2] = Dynamixel2
net._dynamixel_map[3] = Dynamixel3
net._dynamixel_map[4] = Dynamixel4
net._dynamixel_map[5] = Dynamixel5
# Populate our network with dynamixel objects

# Set up the servos
actuator1 = Dynamixel1
actuator2 = Dynamixel2
actuator3 = Dynamixel3
actuator4 = Dynamixel4
actuator5 = Dynamixel5

actuators = [actuator1, actuator2, actuator3, actuator4, actuator5]

# Dynamixel1 = dynamixel.Dynamixel(1, net)
# net._dynamixel_map[1] = Dynamixel1
# actuator1 = Dynamixel1
# actuators = [actuator1]

for actuator in actuators:
    actuator.moving_speed = 30
    actuator.torque_enable = True
    actuator.torque_limit = 800
    actuator.max_torque = 800

p1 = [0, 20, -20, -140, 30]
defalut = [0, 0, 0, 0, 0]
goal = defalut



actuators[0].goal_position = translate_to_dynamixel(goal[0])
actuators[1].goal_position = translate_to_dynamixel(goal[1])
actuators[2].goal_position = translate_to_dynamixel(goal[2])
actuators[3].goal_position = translate_to_dynamixel((-1 * goal[3]))
actuators[4].goal_position = translate_to_dynamixel(goal[4])


# actuators[0].goal_position = translate_to_dynamixel(90)

net.synchronize()

