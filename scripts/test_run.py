#!/usr/bin/env python
# coding: utf-8

# In[1]:



#1. Captures images from pi
#2. Finds midpoint of peak - corresponds to centre of white line
#3. Uses PID controller but with midpoint of centre lane as reference - 2300

# Things to modify:
#     max_speed = 1190000
#     straight_threshold -removed
#     frame_rate = 120
#     How often the image updates steering angle, time_diff = 0.02

#     alpha2: PID output scaling
#     alpha1: speed scaling
#     row: the row that the camera analyzes
#     PID coefficients
#     Peak detection algorithm - what happens when (i) it sees all black (ii) it sees 2 lines during a curve 
#     (iii) start and stop lines - perpendicular lines


# In[1]:

pwm_s =  1230000
from picamera.array import PiYUVArray, PiRGBArray
from picamera import PiCamera


from scipy.signal import find_peaks, butter, filtfilt

import time



import numpy as np
from pwm import PWM

# In[2]:


# To filter the noise in the image we use a 3rd order Butterworth filter
# Wn = 0.02, the cut-off frequency, acceptable values are from 0 to 1
b, a = butter(3, 0.02)


# In[3]:


def get_midpoint_peak(Ibw, row, b, a):
    
    #converting image to numpy array and taking a row L
    Ibw_array = np.array(Ibw)
    L = Ibw_array[row]
    L.shape
       
    #smoothening both arrays to filter the noise in the image we use a 3rd order Butterworth filter
    L_filt = filtfilt(b, a, L)
    
    #image_peak.append(L_filt)

    all_peaks = find_peaks(L_filt, height = 120)   
    #TODO - does it return an empty array when it sees black?
    
    print(all_peaks)
    
    return all_peaks[0]  


# In[4]:


#PID Class Code
"""Ivmech PID Controller is simple implementation of a Proportional-Integral-Derivative (PID) Controller in the Python Programming Language.
More information about PID Controller: http://en.wikipedia.org/wiki/PID_controller
"""
import time

class PID:
    """PID Controller
    """
    
    #P=0.06, D=1.29, I=0.00031 - keeps moving to the left
    #P=0.03, D=1.29, I=0.00031
    
    def __init__(self, P=1.3, Pt=0.55, D=0.005, I=0.005, Ps=0.67):
    #def __init__(self, P, I, D, Ps):

        self.Kp = P
        self.Kpt = Pt
        self.Ki = I
        self.Kd = D
        self.Kps = Ps
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        self.SetPoint = 160
        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        .. figure:: images/pid_1.png
           :align:   center
           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
        """
#         error = self.SetPoint - feedback_value
        error = -1.0*feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
        print ('abs error',abs(error))
        if (delta_time >= self.sample_time):
            print ('abs error',abs(error))
            if abs(error) > 90:
                self.PTerm = self.Kp * error
            else:
                self.PTerm = self.Kpt * error
            print (self.PTerm/error)
            self.PsTerm = self.Kps * abs(error)
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
            
            alpha1=1000
            alpha2 = 3000
            return alpha2*self.output, alpha1*self.PsTerm

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sample time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time


# In[5]:


'''#hardcoding speed
pwm0.duty_cycle = 11000
#PWM duty cycle for straightline motion: 1500000
pwm1_ref = 1465000
pwm1.duty_cycle = pwm1_ref'''

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

pwm0.duty_cycle = pwm_s


# In[7]:


# break for 5 seconds if needed



# In[6]:


#Setting parameters
res = (320, 240)
straight_threshold = 30
row = 80
peak_ref = 160
prev_peak_pos = 160
time_th = 0.001
peak_diff_th = 70
peak_diff_th_st = 10
ctr_3peak = 0 
peak_array = []
#As soon as this becomes 2, the car should stop. Note: should stop within 2 segments of the finish line 


# In[ ]:


#Update PID according to slider values

# pwm_s = 1170000
# pwm0.duty_cycle = pwm_s

pwm0.duty_cycle = pwm_s


camera = PiCamera()


camera.sensor_mode = 7
camera.resolution = res
camera.framerate = 120 #is this actually 10? seems like 34
#was 60 before

# Initialize the buffer and start capturing
rawCapture = PiYUVArray(camera, size=res)
stream = camera.capture_continuous(rawCapture, format="yuv", use_video_port=True)

#creating PID class object
pid_object = PID()

i = 0
j = 0

temp = time.time()
time_start = temp

#Beginning to get camera stream
try:
    for f in stream:
    #     stop_switch = input()
    #     if stop_switch == 's': 
    #         break
        # Get the intensity component of the image (a trick to get black and white images)

        I = f.array[:, :, 0]
        j = j+1
        # Reset the buffer for the next image
        rawCapture.truncate(0)

        diff = time.time() - temp
        if diff < time_th:
            continue

        #print ("Time difference between frames = ", diff)
        temp = time.time()

        #appending
    #     image_raw.append(I)

    #   plt.imshow(I)
    #   plt.show()
    #   file_name = 'a/Apr3_raw_' + str(i) + '.jpg'
        #plt.savefig(file_name)    

    #   if(i > 20):
    #       time_stop = time.time()
    #       break
        peak_pos = get_midpoint_peak(I, row, b, a)

        #Three scenarios when the car should continue with previous PWM - 
        #(1) Car is very close to centre line (2) It sees black (no peaks) #(3) Sees peak of a different line

        #(2) Continue with old steering angle if we see only black
        if (not(np.array(peak_pos).size > 0)):
            peak_array.append(None)
            continue

       #If we see multiple peaks       
        if(np.array(peak_pos).size > 1):
            #Check - are we at the start line?
    #         if(np.array(peak_pos).size ==3):
    #             ctr_3peak = ctr_3peak+1
    #           #Check - did we cross the finish line?
    #             if(ctr_3peak==2):
    #                 pwm0.duty_cycle = 1000000
    #                 pwm1.duty_cycle = pwm_ref
    #                 break
            #Take the one closest in value to prev_peak_pos
            peak_diff = abs(peak_pos - prev_peak_pos)
            min_peak_index = np.argmin(peak_diff)
            peak_pos= peak_pos[min_peak_index]
           # peak_array.append(peak_pos)

        peak_array.append(peak_pos)
        #print('Selected peak: ', peak_pos)
        #Checking if prev_peak_pos should be updated based on peak_diff_th
        if(abs(peak_pos - prev_peak_pos) > peak_diff_th):
              continue

        #print("Stored value of prev_peak_pos:", prev_peak_pos)
#         if(abs(peak_pos - peak_ref) < peak_diff_th_st):
#               continue

        pos_diff = peak_ref - peak_pos #hard-coded

    #     #(1) Continue with old steering angle if it's too close to the centre peak
    #     if(abs(pos_diff) <= straight_threshold):
    #         continue


    #     #(3)Continue with old steering angle if we see peak of a line which is NOT the centre line
    #     if (i>0 and pos_diff * prev_pos_diff < 0):
    #         continue

        i = i+1
        prev_peak_pos = peak_pos

        #test_pid(1.2, 1, 0.001, L=50)
        new_angle, new_speed_error = pid_object.update(pos_diff)

    #     print ("new speed error", new_speed_error)
        new_steering = int(-new_angle + pwm1_ref)
        new_speed = int(pwm_s - new_speed_error)

        print("New angle: ", new_steering)
        #print("New speed: ", new_speed)

        if(new_steering<1010000):
            new_steering = 1010000
        if(new_steering>1990000):
            new_steering = 1990000


        if new_speed > pwm_s:
            new_speed = pwm_s
        #print("PWM new: ", new_steering)
    #     print ("Speed P:", new_speed)

        #Updating PWM
        pwm1.duty_cycle = new_steering
        pwm0.duty_cycle = new_speed
except KeyboardInterrupt:
    pwm0.duty_cycle = 1000000
    pwm0.duty_cycle = 1000000
    pwm0.duty_cycle = 1000000
    pwm1.duty_cycle = pwm1_ref
    pwm1.duty_cycle = pwm1_ref
    pwm1.duty_cycle = pwm1_ref
    pwm1.duty_cycle = pwm1_ref
#     pwm0.enable = False
#     pwm1.duty_cycle = 1465000
#     pwm1.enable = False
    print ('hello')
    for p in peak_array:
        print(p, ' ')
    stream.close()
    rawCapture.close()
    camera.close()
    
#Release resources
stream.close()
rawCapture.close()
camera.close()
pwm0.duty_cycle = 1000000
pwm0.enable = False
pwm1.duty_cycle = 1465000
pwm1.enable = False


# In[30]:
