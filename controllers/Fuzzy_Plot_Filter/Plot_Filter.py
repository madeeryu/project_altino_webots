from vehicle import Driver
import numpy as np
import time
import skfuzzy as fuzz
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.ndimage import gaussian_filter1d
#---------------------- DEKLARASI FUNGSI ROBOT ALTINO ---------------------#
sensorMax = 1000
driver = Driver()

speedValue = []
sensroValue = []
startTime = time.time()

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
maxSpeed = 1.8
minSpeed = 0
normal_speed = 0

# defaults for this controller
useManual = False
headlightsOn = False

previousE = 0
printCounter = 0

driver.setSteeringAngle(angle)
driver.setCruisingSpeed(speed)


def calculate_maxSpeedAltino(parameters):
    return (parameters/100)*1.8

while driver.step() != -1:

#-------------DEKLARASI WAKTU--------------------------# 
    timeS = time.time()
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

    error_value =  setpoint - roll
    previousE = error_value
    dError = error_value - previousE

#---------------------FUZZY LOGIC-------------------------#
    x_error = np.arange(-11,11,0.1)
    x_dError = np.arange(-11,11,0.1)
    x_pwm = np.arange(0,1.8,0.01)

    e_NB =fuzz.trapmf(x_error, [-11,-11, -5, -2.5])
    e_NS =fuzz.trimf(x_error, [-5, -2.5, 0])
    e_Z =fuzz.trimf(x_error, [-2.5, 0, 2.5])
    e_PS = fuzz.trimf(x_error, [0, 2.5, 5])
    e_PB = fuzz.trapmf(x_error, [2.5,5,11,11])

    dE_NB =fuzz.trapmf(x_dError, [-11,-11, -5, -2.5])
    dE_NS =fuzz.trimf(x_dError, [-5, -2.5, 0])
    dE_Z =fuzz.trimf(x_dError, [-2.5, 0, 2.5])
    dE_PS = fuzz.trimf(x_dError, [0, 2.5, 5])
    dE_PB = fuzz.trapmf(x_dError, [2.5,5,11,11])

    pwm_lambat = fuzz.gaussmf(x_pwm, 0.3, 0.15)  # Terpusat di 0.3 dengan sigma 0.15
    pwm_normal = fuzz.gaussmf(x_pwm, 0.9, 0.15)  # Terpusat di 0.9 dengan sigma 0.15
    pwm_cepat = fuzz.gaussmf(x_pwm, 1.5, 0.15)   # Terpusat di 1.5 dengan sigma 0.15

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
    if signal == 0:
       signal = 1
    
    pwm = signal * roll / signal
    rpm = calculate_maxSpeedAltino(pwm)

    #kecepatan motor menggunakan fuzzy control
    waktuSimulasi = timeS - startTime
    if waktuSimulasi <= 1 :
        speed = normal_speed
    elif waktuSimulasi >= 1.001 :
        speed = normal_speed + rpm
#------------------ SETTING ANGLE ROBOT ----------------#
    if fLeftVal > fRightVal:
        angle += (fLeftVal - fRightVal) / (300 * sensorMax) 
    elif fRightVal >fLeftVal: 
        angle -= (fRightVal - fLeftVal) / (300 * sensorMax)
    else:
        angle /= 1.5

    if sLeftVal > 300:
        angle += 0.003
    if sRightVal > 300:
        angle -= 0.003

    # membuat batasan kecepatan dan angle untuk mobil berbelok
    if normal_speed > 1:
        normal_speed = 1
    elif speed < -1 * maxSpeed:
        speed = -1 * maxSpeed
    if angle > 0.5:
        angle = 0.5
    elif angle < -0.4:
        angle = -0.4
        
    normal_speed += 0.01

    #------------------ SIMPAN DATA PLOT ---------------------#
    speedValue.append(speed)
    sensroValue.append(roll)
    #-------------------batas simulasi ---------------------- #
    # waktuSimulasi = timeS - startTime
    if waktuSimulasi >= 15:
        break
    # print(f"Angle : {angle:.2f}     || Throttle : {speed:.3f}       ||     Pitch Sensor : {roll:.2f}")
    print(f'{speed:.2f}')
    driver.setCruisingSpeed(speed)
    driver.setSteeringAngle(angle)

#memproses data yang didapatkan untuk dijadikan sumbu Y pada plot

#shape Pada speed motor
shape_value = np.array(speedValue)
get_data = shape_value.shape
get_data_1 = get_data[0]
x_values = np.linspace(0, waktuSimulasi, get_data_1)

# definisi time step 

time_step = x_values  # Assuming the time step 
# Membuat array waktu dari 0 sampai 15 detik

# Sinyal Gaussian dengan menggunakan filter Gaussian untuk menghaluskan data
sigma = 2  # Parameter sigma untuk Gaussian filter yang menentukan seberapa halus sinyal
smoothed_speed_data = gaussian_filter1d(speedValue, sigma)



time_step = x_values  # Assuming the time step 

# Menemikan peak time dan peak value menggunakan libary scpiy
peaks, _ = find_peaks(smoothed_speed_data)
peak_time = time_step[peaks[0]] if peaks.size > 0 else None
peak_value = smoothed_speed_data[peaks[0]] if peaks.size > 0 else None

# nilai Steady State
steady_state_value = smoothed_speed_data[-1]

# Calculate the rise time (time to go from 10% to 90% of the steady-state value)
# interp_speed = interp1d(time_step, speedValue)
# rise_time_indices = np.where((speedValue >= 0.1 * steady_state_value) & (speedValue <= 0.9 * steady_state_value))[0]
# rise_time = time_step[rise_time_indices[0]] if rise_time_indices.size > 0 else None

time_10 = np.interp(0.1 * steady_state_value, smoothed_speed_data, time_step)
time_90 = np.interp(0.9 * steady_state_value, smoothed_speed_data, time_step)
rise_time = time_90 - time_10 if time_10 and time_90 else None

# Calculate overshoot (percentage above the steady-state value)
overshoot = ((peak_value - steady_state_value) / steady_state_value) * 100 if peak_value and steady_state_value else None

# Calculate settling time (time to remain within 2% of the steady-state value)
settling_indices = np.where(np.abs(smoothed_speed_data - steady_state_value) <= 0.02 * steady_state_value)[0]
settling_time = time_step[settling_indices[0]] if settling_indices.size > 0 else None


#---------------- PLOT DATA MENTAH -------------- #
plt.figure(figsize=(12, 6))
plt.plot(time_step, speedValue,color= 'orange', label='Kecepatan Motor', linewidth=2)
# plt.plot(time_step, speedValue, label='Speed Value', color='orange')
plt.xlabel('Time (seconds)')
plt.ylabel('Speed')
plt.title('Speed Response with Time Response Characteristics')
plt.grid(True)
plt.legend()
plt.show()

#---------------- PLOT DATA YANG SUDAH DI FILTER --------------------#
plt.figure(figsize=(12, 6))
plt.plot(time_step, smoothed_speed_data,color= 'pink', label='Kecepatan Motor FILTER', linewidth=2)
# plt.plot(time_step, speedValue, label='Speed Value', color='orange')
plt.xlabel('Time (seconds)')
plt.ylabel('Speed')
plt.title('Speed Response with Time Response Characteristics')
plt.grid(True)
plt.legend()
plt.show()

#---------------- PLOT DATA FILTER YANG SUDAH DI OLAH --------------------#
plt.figure(figsize=(12, 6))
plt.plot(time_step, smoothed_speed_data, color= 'red',label='Kecepatan Motor diolah', linewidth=2)
plt.xlabel('Time (seconds)')
plt.ylabel('Speed')
plt.title('Speed Response with Time Response Characteristics')
plt.grid(True)
plt.legend()

# Annotate steady state value
plt.annotate(f'Steady State Value: {steady_state_value:.2f}', 
             xy=(time_step[-1], steady_state_value), 
             xytext=(time_step[-1]-0.1, steady_state_value + 0.1),
             arrowprops=dict(facecolor='black', arrowstyle='->'))

# Annotate peak time and peak value
if peak_time and peak_value:
    plt.plot(peak_time, peak_value, 'ro', label=f'peak time: {peak_time:.2f}')  # Mark the peak point
    plt.plot(peak_time, peak_value, 'ro', label=f'peak Value: {peak_value:.2f}')  # Mark the peak point
    plt.annotate(f'Peak Time: {peak_time:.2f}s\nPeak Value: {peak_value:.2f}', 
                 xy=(peak_time, peak_value), 
                 xytext=(peak_time + 0.05, peak_value+ 0.05),
                 arrowprops=dict(facecolor='black', arrowstyle='->'))

# Annotate rise time
# if rise_time:
#     plt.axvline(x=rise_time, color='g', linestyle='--', label=f'Rise Time: {rise_time:.2f}s')
#     plt.annotate(f'Rise Time: {rise_time:.2f}s', 
#                  xy=(rise_time, 0), 
#                  xytext=(rise_time + 1, -0.05),
#                  arrowprops=dict(facecolor='black', arrowstyle='->')

if rise_time:
    plt.axvline(x=rise_time, color='pink', linestyle='--', label=f'Rise Time: {rise_time:.2f}s')
    plt.annotate(f'Rise Time: {rise_time:.2f}s', 
                 xy=(rise_time, 0), 
                 xytext=(rise_time, -0.05),
                 arrowprops=dict(facecolor='black', arrowstyle='->'))

# Annotate settling time
if settling_time:
    plt.axvline(x=settling_time, color='green', linestyle='--', label=f'Settling Time: {settling_time:.2f}s')
    plt.annotate(f'Settling Time: {settling_time:.2f}s', 
                 xy=(settling_time, steady_state_value), 
                 xytext=(settling_time, steady_state_value - 0.1),
                 arrowprops=dict(facecolor='black', arrowstyle='->'))

plt.grid(True)
plt.legend()
plt.show()

