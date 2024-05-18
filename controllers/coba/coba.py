from vehicle import Driver
import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt

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

# angle refers to the angle (from straight ahead) that the wheels
# currently have
angle = 0
setpoint  = 0
speed = 0
# maxSpeed = 1.8
maxSpeed = 1.8
minSpeed = 0
normal_speed = 1
# ensure 0 starting speed and wheel angle
driver.setSteeringAngle(angle)
driver.setCruisingSpeed(speed)
# defaults for this controller
useManual = False
headlightsOn = False


previousE = 0
printCounter = 0

def calculate_maxSpeedAltino(parameters):
    return (parameters/100)*1.8

while driver.step() != -1:

#-------------DEKLARASI VARIABEL-----------------------# 
    #sensor accelerometer
    acc_value = acc_sensor.getValues()

    #censor jarak pada bagian depan mobil
    fLeftVal = front_left_sensor.getValue()
    fCenterVal = front_center_sensor.getValue()
    fRightVal = front_right_sensor.getValue()
    #sensor jarak pada bagian samping mobil
    sLeftVal = side_left_sensor.getValue()
    sRightVal = side_right_sensor.getValue()
    #sensor jarak pada bagian belakang mobil
    backVal = back_sensor.getValue()
    
#-------------PERHITUNGAN ERROR FUZZY LOGIC---------------#    

    round_acc_value = [f"{num:.2f}" for num in acc_value]
    roll = float(round_acc_value[0])#mengambil 1 angle 

    speed = driver.getTargetCruisingSpeed()
    error_value =  setpoint - roll
    previousE = error_value
    dError = error_value - previousE

#---------------------FUZZY LOGIC-------------------------#
    x_error = np.arange(-11,11,0.1)
    x_dError = np.arange(-11,11,0.1)
    x_pwm = np.arange(0,1.8,0.1)

    e_NB =fuzz.trapmf(x_error, [-11,-11, -5, -2.5])
    e_NS =fuzz.trimf(x_error, [-5, -2.5, 0])
    e_Z =fuzz.trimf(x_error, [-2.5, 0, 2.5])
    e_PS = fuzz.trimf(x_error, [0, 2.5, 5])
    e_PB = fuzz.trapmf(x_error, [2.5,5,11,11])

    dE_NB =fuzz.trapmf(x_error, [-11,-11, -5, -2.5])
    dE_NS =fuzz.trimf(x_error, [-5, -2.5, 0])
    dE_Z =fuzz.trimf(x_error, [-2.5, 0, 2.5])
    dE_PS = fuzz.trimf(x_error, [0, 2.5, 5])
    dE_PB = fuzz.trapmf(x_error, [2.5,5,11,11])

    # pwm_lambat =  fuzz.trapmf(x_pwm,[-5,-5,-2.5,0])
    # pwm_normal =  fuzz.trimf(x_pwm,[-2.5,0,2.5])
    # pwm_cepat =  fuzz.trapmf(x_pwm,[0,2.5,5,5])
    pwm_lambat =  fuzz.trimf(x_pwm,[0,0.3,0.6])
    pwm_normal =  fuzz.trimf(x_pwm,[0.6,0.9,1.2])
    pwm_cepat =  fuzz.trimf(x_pwm,[1.2,1.5,1.8])

    error = error_value 
    deltaError = dError

    e_lvl_NB = fuzz.interp_membership(x_error, e_NB,error)
    e_lvl_NS = fuzz.interp_membership(x_error, e_NS,error)
    e_lvl_Z = fuzz.interp_membership(x_error, e_Z,error)
    e_lvl_PS = fuzz.interp_membership(x_error, e_PS,error)
    e_lvl_PB = fuzz.interp_membership(x_error, e_PB,error)

    dE_lvl_NB = fuzz.interp_membership(x_dError, dE_NB,deltaError)
    dE_lvl_NS = fuzz.interp_membership(x_dError, dE_NS,deltaError)
    dE_lvl_Z = fuzz.interp_membership(x_dError, dE_Z,deltaError)
    dE_lvl_PS = fuzz.interp_membership(x_dError, dE_PS,deltaError)
    dE_lvl_PB = fuzz.interp_membership(x_dError, dE_PB,deltaError)

    rule1 = np.fmin(e_lvl_NB, dE_lvl_NB)#low
    rule2 = np.fmin(e_lvl_NB, dE_lvl_NS)#low
    rule3 = np.fmin(e_lvl_NB, dE_lvl_Z)#low
    rule4 = np.fmin(e_lvl_NB, dE_lvl_PS)#low
    rule5 = np.fmin(e_lvl_NB, dE_lvl_PB)#normal
    rule6 = np.fmin(e_lvl_NS, dE_lvl_NB)#low
    rule7 = np.fmin(e_lvl_NS, dE_lvl_NS)#low
    rule8 = np.fmin(e_lvl_NS, dE_lvl_Z)#low
    rule9 = np.fmin(e_lvl_NS, dE_lvl_PS)#normal
    rule10 = np.fmin(e_lvl_NS, dE_lvl_PB)#fast
    rule11 = np.fmin(e_lvl_Z, dE_lvl_NB)#low
    rule12 = np.fmin(e_lvl_Z, dE_lvl_NS)#low
    rule13 = np.fmin(e_lvl_Z, dE_lvl_Z)#normal
    rule14 = np.fmin(e_lvl_Z, dE_lvl_PS)#fast
    rule15 = np.fmin(e_lvl_Z, dE_lvl_PB)#fast
    rule16 = np.fmin(e_lvl_PS, dE_lvl_NB)#low
    rule17 = np.fmin(e_lvl_PS, dE_lvl_NS)#normal
    rule18 = np.fmin(e_lvl_PS, dE_lvl_Z)#fast
    rule19 = np.fmin(e_lvl_PS, dE_lvl_PS)#fast
    rule20 = np.fmin(e_lvl_PS, dE_lvl_PB)#fast
    rule21 = np.fmin(e_lvl_PB, dE_lvl_PB)#normal
    rule22 = np.fmin(e_lvl_PB, dE_lvl_PB)#fast
    rule23 = np.fmin(e_lvl_PB, dE_lvl_PB)#fast
    rule24 = np.fmin(e_lvl_PB, dE_lvl_PB)#fast
    rule25 = np.fmin(e_lvl_PB, dE_lvl_PB)#fast

    gabungan_lambat = np.fmax(rule1,np.fmax(rule2,np.fmax(rule3,np.fmax(rule4,np.fmax(rule6,np.fmax(rule7,np.fmax(rule8,np.fmax(rule11,np.fmax(rule12,rule16))))))))) 
    gabungan_normal =  np.fmax(rule5,np.fmax(rule9,np.fmax(rule13,np.fmax(rule17,rule21))))
    gabungan_cepat = np.fmax(rule10,np.fmax(rule14,np.fmax(rule15,np.fmax(rule18,np.fmax(rule19,np.fmax(rule20,np.fmax(rule22,np.fmax(rule23,np.fmax(rule24,rule25))))))))) 


    hasil_lambat = np.fmin(gabungan_lambat,pwm_lambat)
    hasil_normal = np.fmin(gabungan_normal,pwm_normal)
    hasil_cepat = np.fmin(gabungan_cepat,pwm_cepat)

    aggregated = np.fmax(hasil_lambat,np.fmax(hasil_normal,hasil_cepat))
    signal = fuzz.defuzz(x_pwm, aggregated, 'centroid')

    pwm = signal*roll/signal
    
    rpm = calculate_maxSpeedAltino(pwm)
    # speed+=1
    # speed = normal_speed + rpm
    speed = normal_speed 
    # 48 53 tanpa fuzzy
    # 48 52 menggunakan fuzzy

    #     # Distance Sensor Values:
    #     # 1000: 0cm
    #     # 800:  12cm
    #     # 600:  24cm
    #     # 400:  36cm
    #     # 200:  48cm
    #     # 0:    60cm
    #     #-200 : 72cm

            
    if fLeftVal > fRightVal:
        angle += (fLeftVal - fRightVal) / (300 * sensorMax) 
    elif fRightVal > fLeftVal: 
        angle -= (fRightVal - fLeftVal) / (300 * sensorMax)
    else:
        angle /= 1.5

    if sLeftVal > 300:
        angle += 0.003
    if sRightVal > 300:
        angle -= 0.003
    
  

    # # clamp speed and angle to max values
    if speed > maxSpeed:
        speed = maxSpeed
    elif speed < -1 * maxSpeed:
        speed = -1 * maxSpeed
    if angle > 0.5:
        angle = 0.5
    elif angle < -0.4:
        angle = -0.4

    # if (printCounter % 10) == 0:
    # print("Angle: %.2f" % angle)
    # print("_______HASIL PRINT FUNGSI_____:")
    # print(f"Angle : {angle:.2f} || Throttle : {speed} | Pitch Sensor : {roll:.2f}")
    #     # print("Angle: %.2f" % angle)
    #     # print("ERROR : ", error)
    #     # print("DERROR : ", dError)
    #     # print("Throttle: {} " .format (speed))
    # print(speed)
        # print("Sensor pitch Value: {}".format(roll))
    driver.setCruisingSpeed(speed)
    driver.setSteeringAngle(angle)
    # printCounter += 1
