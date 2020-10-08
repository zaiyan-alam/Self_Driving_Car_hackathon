cd /sys/class/pwm/
sudo chmod 777 -R pwmchip0
cd ~
source raceon/bin/activate
python -c "exec(\"from pwm import PWM\npwm0 = PWM(0)\npwm1 = PWM(1)\npwm0.export()\npwm1.export()\")"
deactivate
cd /sys/class/pwm/pwmchip0
sudo chmod 777 -R pwm0
sudo chmod 777 -R pwm1
