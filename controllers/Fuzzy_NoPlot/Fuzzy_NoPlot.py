from vehicle import Driver
import numpy as np
import time
import skfuzzy as fuzz
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.ndimage import gaussian_filter1d
import csv
import os


sensorMax = 1000
driver = Driver()
startTime = time.time()
basicTimeStep = int(driver.getBasicTimeStep())

csv_file_name = 'data_simulasi.csv'
if os.path.exists(csv_file_name):
    os.remove(csv_file_name)

def save_to_csv(data):
    with open(csv_file_name, mode='a', newline='') as file:
        writer = csv.writer(file)
     
        if file.tell() == 0:
            writer.writerow(['Time', 'SpeedMotor', 'ErrorValue', 'SensorAccelerometer']) 
        writer.writerow(data)

def initialize_and_enable_sensors(driver, basicTimeStep):
    sensors = [
        'front_left_sensor', 'front_center_sensor', 'front_right_sensor',
        'side_left_sensor', 'side_right_sensor', 'back_sensor',
        'accelerometer'
    ]
    devices = {sensor: driver.getDevice(sensor) for sensor in sensors}
    for device in devices.values():
        device.enable(basicTimeStep)
    return devices
devices = initialize_and_enable_sensors(driver, basicTimeStep)


def get_sensor_values(devices):
    # Mengambil nilai dari semua sensor yang diperlukan menggunakan dictionary
    acc_value = devices['accelerometer'].getValues()
    fLeftVal = devices['front_left_sensor'].getValue()
    fCenterVal = devices['front_center_sensor'].getValue()
    fRightVal = devices['front_right_sensor'].getValue()
    sLeftVal = devices['side_left_sensor'].getValue()
    sRightVal = devices['side_right_sensor'].getValue()
    backVal = devices['back_sensor'].getValue()

    return acc_value, fLeftVal, fCenterVal, fRightVal, sLeftVal, sRightVal, backVal
def calculate_maxSpeedAltino(parameters):
    return (parameters/100)*1.8

def run_fuzzy_logic_control(acc_value, setpoint):
    round_acc_value = [f"{num:.2f}" for num in acc_value]
    roll = float(round_acc_value[0])  # mengambil 1 angle

    # Mengolah Data Error
    # error_value = speed - setpoint
    error_value = roll-setpoint
    previousE = error_value
    dError = error_value - previousE

    # Definisi rentang untuk error, deltaError, dan pwm
    x_error = np.arange(-11, 11, 0.1)
    x_dError = np.arange(-11, 11, 0.1)
    x_pwm = np.arange(0, 1.8, 0.01)

    # Definisi fungsi keanggotaan untuk error dan deltaError
    e_NB = fuzz.trapmf(x_error, [-11, -11, -5, -2.5])
    e_NS = fuzz.trimf(x_error, [-5, -2.5, 0])
    e_Z = fuzz.trimf(x_error, [-2.5, 0, 2.5])
    e_PS = fuzz.trimf(x_error, [0, 2.5, 5])
    e_PB = fuzz.trapmf(x_error, [2.5, 5, 11, 11])

    dE_NB = fuzz.trapmf(x_dError, [-11, -11, -5, -2.5])
    dE_NS = fuzz.trimf(x_dError, [-5, -2.5, 0])
    dE_Z = fuzz.trimf(x_dError, [-2.5, 0, 2.5])
    dE_PS = fuzz.trimf(x_dError, [0, 2.5, 5])
    dE_PB = fuzz.trapmf(x_dError, [2.5, 5, 11, 11])

    # Definisi fungsi keanggotaan untuk output PWM
    pwm_lambat = fuzz.gaussmf(x_pwm, 0.3, 0.15)
    pwm_normal = fuzz.gaussmf(x_pwm, 0.9, 0.15)
    pwm_cepat = fuzz.gaussmf(x_pwm, 1.5, 0.15)

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

    
    # Perbaikan bug
    if signal == 0:
        signal = roll

    pwm = signal * roll / signal
    rpm = calculate_maxSpeedAltino(pwm)

    return rpm, roll,error_value

def adjust_steering_and_speed(fLeftVal, fRightVal, sLeftVal, sRightVal, angle, speed, normal_speed, maxSpeed):
    # Mengatur sudut berdasarkan sensor depan
    if fLeftVal > fRightVal:
        angle += (fLeftVal - fRightVal) / (300 * sensorMax)
    elif fRightVal > fLeftVal:
        angle -= (fRightVal - fLeftVal) / (300 * sensorMax)
    else:
        angle /= 1.5

    # Mengatur sudut berdasarkan sensor samping
    if sLeftVal > 300:
        angle += 0.003
    if sRightVal > 300:
        angle -= 0.003

    # Membatasi kecepatan dan sudut untuk mobil berbelok
    normal_speed = min(normal_speed, 1)
    speed = max(speed, -1 * maxSpeed)
    angle = max(min(angle, 0.5), -0.4)

    normal_speed += 0.01

    return angle, speed, normal_speed

def update_speed_based_on_time(startTime, normal_speed, rpm):
    timeS = time.time()
    waktuSimulasi = timeS - startTime
    if waktuSimulasi <= 0.1:
        return normal_speed, waktuSimulasi
    elif waktuSimulasi >= 0.2:
        return normal_speed + rpm, waktuSimulasi
    return normal_speed, waktuSimulasi


def init():
    global startTime, speedValue, sensroValue, angle, speed, normal_speed, maxSpeed, setpoint, devices
    startTime = time.time()
    speedValue = []
    sensroValue = []
    angle = 0
    speed = 0
    maxSpeed = 1.8
    normal_speed = 1.01
    setpoint = 0

    while driver.step() != -1:  
        acc_value, fLeftVal, fCenterVal, fRightVal, sLeftVal, sRightVal, backVal = get_sensor_values(devices)    
        rpm, roll, error_value = run_fuzzy_logic_control(acc_value, setpoint)    
        angle, speed, normal_speed = adjust_steering_and_speed(fLeftVal, fRightVal, sLeftVal, sRightVal, angle, speed, normal_speed, maxSpeed)     
        speed, waktuSimulasi = update_speed_based_on_time(startTime, normal_speed, rpm)

        print(f"Error : {error_value:.2f}     || Throttle : {speed:.3f}       ||     Pitch Sensor : {roll:.2f}")

        driver.setCruisingSpeed(speed)
        driver.setSteeringAngle(angle)

        data = [waktuSimulasi,speed,error_value,roll]  
        save_to_csv(data)
        #Menyimpan Data Yang Didapatkan
        speedValue.append(speed)
        sensroValue.append(roll)



if __name__ == "__main__":
    init()

