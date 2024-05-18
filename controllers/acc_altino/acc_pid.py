from vehicle import Driver

sensorMax = 1000

driver = Driver()

basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep
front_left_sensor = driver.getDevice('front_left_sensor')
front_center_sensor = driver.getDevice('front_center_sensor')
front_right_sensor = driver.getDevice('front_right_sensor')

headlights = driver.getDevice("headlights")
backlights = driver.getDevice("backlights")

keyboard = driver.getKeyboard()
keyboard.enable(sensorTimeStep)

front_left_sensor.enable(sensorTimeStep)
front_center_sensor.enable(sensorTimeStep)
front_right_sensor.enable(sensorTimeStep)

side_left_sensor = driver.getDevice('side_left_sensor')
side_right_sensor = driver.getDevice('side_right_sensor')
back_sensor = driver.getDevice('back_sensor')

side_left_sensor.enable(sensorTimeStep)
side_right_sensor.enable(sensorTimeStep)
back_sensor.enable(sensorTimeStep)

acc_sensor =  driver.getDevice('accelerometer')
acc_sensor.enable(sensorTimeStep)   

# speed refers to the speed in km/h at which we want Altino to travel
speed = 1
# angle refers to the angle (from straight ahead) that the wheels
# currently have
angle = 0

# This the Altino's maximum speed
# all Altino controllers should use this maximum value
maxSpeed = 1.8
# ensure 0 starting speed and wheel angle
driver.setSteeringAngle(angle)
driver.setCruisingSpeed(speed)
# defaults for this controller
useManual = False
headlightsOn = False

printCounter = 0

while driver.step() != -1:
    acc_value = acc_sensor.getValues()

    round_acc_value = [f"{num:.2f}" for num in acc_value]
    roll = float(round_acc_value[0])
    # enable backlights for reverse
    speed = driver.getTargetCruisingSpeed()
    if speed < 0:
        backlights.set(1)
    else:
        backlights.set(0)
  
    fLeftVal = front_left_sensor.getValue()
    fCenterVal = front_center_sensor.getValue()
    fRightVal = front_right_sensor.getValue()

    sLeftVal = side_left_sensor.getValue()
    sRightVal = side_right_sensor.getValue()
    backVal = back_sensor.getValue()

        # Distance Sensor Values:
        # 1000: 0cm
        # 800:  12cm
        # 600:  24cm
        # 400:  36cm
        # 200:  48cm
        # 0:    60cm
        #-200 : 72cm

    # if fCenterVal > 400 and fCenterVal < 600:
    #     speed -= (0.01 * speed)
    # elif fCenterVal > 600 and fCenterVal < 800:
    #     speed /= 1.01
    # if backVal > 400 and backVal < 600:
    #     speed /= 1.01
    # elif backVal > 600 and backVal < 800:
    #     speed /= 1.1

    # if fLeftVal > fRightVal:
    #     angle += (fLeftVal - fRightVal) / (300 * sensorMax)
    # elif fRightVal > fLeftVal:  
    #     angle -= (fRightVal - fLeftVal) / (300 * sensorMax)
    # else:
    #     angle /= 1.5

    # if sLeftVal > 300:
    #     angle += 0.003
    # if sRightVal > 300:
    #     angle -= 0.003

    # speed += 0.001

    # clamp speed and angle to max values
    if speed > maxSpeed:
        speed = maxSpeed
    elif speed < -1 * maxSpeed:
        speed = -1 * maxSpeed
    if angle > 0.4:
        angle = 0.4
    elif angle < -0.4:
        angle = -0.4

    if (printCounter % 10) == 0:
    
        print("Current Wheel Angle and Throttle values:")
        print("Angle: %.2f" % angle)
    
        # print("Throttle: %.1f " % (100 * speed / maxSpeed))
        print("Throttle: {} " .format (speed))
        print("Sensor Value: {}".format(round_acc_value))
        print("Sensor Raw Value: {}".format(roll))
    driver.setCruisingSpeed(speed)
    driver.setSteeringAngle(angle)
    printCounter += 1
