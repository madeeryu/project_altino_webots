import numpy as np
import skfuzzy as fuzz
import matplotlib.pyplot as plt
import acc_altino

x_error = np.arange(-101,150,1)
x_dError = np.arange(-101,150,1)
x_pwm = np.arange(0,150,1)

e_NB =fuzz.trimf(x_error, [-101, -75, -25])
e_NS =fuzz.trimf(x_error, [-75, -25, 25])
e_Z =fuzz.trimf(x_error, [-25, 25, 75])
e_PS = fuzz.trimf(x_error, [25, 75, 125])
e_PB = fuzz.trimf(x_error, [75,125,150])

dE_NB =fuzz.trimf(x_dError, [-101, -75, -25])
dE_NS =fuzz.trimf(x_dError, [-75, -25, 25])
dE_Z =fuzz.trimf(x_dError, [-25, 25, 75])
dE_PS = fuzz.trimf(x_dError, [25, 75, 125])
dE_PB = fuzz.trimf(x_dError, [75,125,150])

pwm_lambat =  fuzz.trimf(x_pwm,[0,25,50])
pwm_normal =  fuzz.trimf(x_pwm,[25,50,75])
pwm_cepat =  fuzz.trimf(x_pwm,[50,75,100])

fig, (ax1,ax2,ax3) = plt.subplots(nrows = 3 , figsize=(15, 10))

ax1.plot(x_error, e_NB, 'b', linewidth=1.5, label='NB')
ax1.plot(x_error, e_NS, 'g', linewidth=1.5, label='NS')
ax1.plot(x_error, e_Z, 'r', linewidth=1.5, label='Z')
ax1.plot(x_error, e_PS, 'yellow', linewidth=1.5, label='PS')
ax1.plot(x_error, e_PB, 'purple', linewidth=1.5, label='PB')
ax1.set_title('ERROR')
ax1.legend(loc = 3)

ax2.plot(x_dError, e_NB, 'b', linewidth=1.5, label='NB')
ax2.plot(x_dError, e_NS, 'g', linewidth=1.5, label='NS')
ax2.plot(x_dError, e_Z, 'r', linewidth=1.5, label='Z')
ax2.plot(x_dError, e_PS, 'yellow', linewidth=1.5, label='PS')
ax2.plot(x_dError, e_PB, 'purple', linewidth=1.5, label='PB')
ax2.set_title('ERROR')
ax2.legend(loc = 3)

ax3.plot(x_pwm, pwm_lambat, 'b', linewidth=1.5, label='LAMBAT')
ax3.plot(x_pwm, pwm_normal, 'g', linewidth=1.5, label='NORMAL')
ax3.plot(x_pwm, pwm_cepat, 'r', linewidth=1.5, label='CEPAT')
ax3.set_title('PWM')
ax3.legend(loc = 3)


error = acc_altino.error_value
dError = acc_altino.dError

e_lvl_NB = fuzz.interp_membership(x_error, e_NB,error)
e_lvl_NS = fuzz.interp_membership(x_error, e_NS,error)
e_lvl_Z = fuzz.interp_membership(x_error, e_Z,error)
e_lvl_PS = fuzz.interp_membership(x_error, e_PS,error)
e_lvl_PB = fuzz.interp_membership(x_error, e_PB,error)

dE_lvl_NB = fuzz.interp_membership(x_dError, e_NB,dError)
dE_lvl_NS = fuzz.interp_membership(x_dError, e_NS,dError)
dE_lvl_Z = fuzz.interp_membership(x_dError, e_Z,dError)
dE_lvl_PS = fuzz.interp_membership(x_dError, e_PS,dError)
dE_lvl_PB = fuzz.interp_membership(x_dError, e_PB,dError)

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
pwm = fuzz.defuzz(x_pwm, aggregated, 'centroid')

hasil_vic = fuzz.interp_membership(x_pwm, aggregated, pwm)

hasil = np.zeros_like(x_pwm)

fig, ax2 = plt.subplots(figsize=(8, 3))
ax2.plot(x_pwm, pwm_lambat, 'b', linewidth=1.5, label='LAMBAT')
ax2.plot(x_pwm, pwm_normal, 'g', linewidth=1.5, label='NORMAL')
ax2.plot(x_pwm, pwm_cepat, 'r', linewidth=1.5, label='CEPAT')
ax2.plot([pwm, pwm], [0, hasil_vic], 'k', linewidth=1.5, alpha=0.9)
ax2.fill_between(x_pwm, aggregated, facecolor='Orange', alpha=0.7)
ax2.set_title('PWM')    

# Turn off top/right axes
for ax in (ax2,):
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.get_xaxis().tick_bottom()
    ax.get_yaxis().tick_left()

plt.tight_layout()
plt.legend()