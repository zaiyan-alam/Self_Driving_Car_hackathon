from pwm import PWM
pwm0 = PWM(0)
pwm1 = PWM(1)
pwm0.export()
pwm1.export()
pwm0.period = 20000000
pwm1.period = 20000000
pwm0.duty_cycle = 1000000
pwm1.duty_cycle = 1500000
pwm0.enable = True
pwm1.enable = True
pwm1_ref = 1465000
pwm1.duty_cycle = pwm1_ref
# In[6]:
pwm_s = 1200000
pwm0.duty_cycle = pwm_s
