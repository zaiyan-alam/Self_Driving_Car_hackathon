from pwm import PWM
pwm0 = PWM(0)
pwm1 = PWM(1)
pwm0.export()
pwm1.export()
pwm0.period = 20000000
pwm1.period = 20000000
pwm0.duty_cycle = 1000000
pwm0.enable = False
pwm1.duty_cycle = 1465000
pwm1.enable = False
