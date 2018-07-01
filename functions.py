import numpy as np
from numpy import sin, cos, pi
import time
import threading
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_PWM_Servo_Driver

import pigpio
import os.path
import math
import sys, getopt

import atexit


class robot():

    def __init__(self, IC=[0.]*6, simulation=False):
      
      self.simulation = simulation
      self.ODOPIN1 = 4
      self.ODOPIN2 = 18
      self.TRIG_PIN = 22
      self.ECHO_PIN = 23

      self.dt = .1
      self.d = (6.125/2) * .0254  ## distance from wheel to car centerline (m)
      self.r = .065/2  ## radius of wheel (m)
      self.odo = [0, 0]  ## motor 1 and 2 odometer
      self.encoderRatio = 384./2. #odometers are only counting rising edges
      self.motorDirection = [1,1]  ## direction of motors: +1 for forward, -1 for backward
      self.lastOdo = [0,0] ## carries the odometer count from the last time it was referenced
      self.z = np.zeros((6,2))  ## state as currently predicted by the accelerometer
      self.z[:,0] = IC
      self.u_max = 144*.1047  #maximum wheel speed in rad/sec
      self.sensor_rate = .01
      self.minpwmL = 87
      self.minpwmR = 87

      self.x_h = IC #rob's state estimate 

      self.servo = Adafruit_PWM_Servo_Driver.PWM(address=0x60)
      self.servo.setPWMFreq(50)

      self.zRecord = np.zeros((6,10000))
      self.zCount = 0

      if not simulation:
          
          self.mypi = pigpio.pi()
          self.cb1 = self.mypi.callback(self.ODOPIN1)
          self.cb2 = self.mypi.callback(self.ODOPIN2)
          #self.cb1 = self.mypi.callback(self.ODOPIN1, pigpio.RISING_EDGE, self.odometer)
          #self.cb2 = self.mypi.callback(self.ODOPIN2, pigpio.RISING_EDGE, self.odometer(1))
          self.mypi.set_pull_up_down(self.ODOPIN1, pigpio.PUD_UP)
          self.mypi.set_pull_up_down(self.ODOPIN2, pigpio.PUD_UP)

          self.mypi.set_mode(self.TRIG_PIN, pigpio.OUTPUT)
          self.mypi.set_mode(self.ECHO_PIN, pigpio.INPUT)
          self.mypi.write(self.TRIG_PIN, 0)
          
    
#################################   INITIALIZE MOTORS  ##############################################

      if not simulation:
          # create a default object, no changes to I2C address or frequency
          self.mh = Adafruit_MotorHAT(addr=0x60)

          # recommended for auto-disabling motors on shutdown!

          self.turnOffMotors()
          

          atexit.register(self.turnOffMotors)

          ################################# DC motor test!
          self.rightMotor = self.mh.getMotor(1)
          self.leftMotor = self.mh.getMotor(4)

          # set the speed to start, from 0 (off) to 255 (max speed)
          self.leftMotor.setSpeed(0)
          self.leftMotor.run(Adafruit_MotorHAT.FORWARD);

          self.rightMotor.setSpeed(0)
          self.rightMotor.run(Adafruit_MotorHAT.FORWARD);
          # turn on motor

          self.leftMotor.run(Adafruit_MotorHAT.RELEASE);
          self.rightMotor.run(Adafruit_MotorHAT.RELEASE);

          print "motors on!"

    ##############################################################################
    ###########################################################################3###
      

###########################  MY FUNCTIONS  ########################################
    def turnOffMotors(self):
          self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
          self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
          self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
          self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)
          
    def odometer(self,):
        ## num is 0 for left motor. 1 for right motor
        self.odo[0] += self.motorDirection[0]
        print self.odo

    def accel_sensor(self):
      ## accelerometer measures linear acceleration in g's
      ## and angular velocity in deg/sec
        import sys, getopt

        sys.path.append('.')
        import RTIMU
        import os.path
        import time
        import math

        SETTINGS_FILE = "RTIMULib"

        print("Using settings file " + SETTINGS_FILE + ".ini")
        if not os.path.exists(SETTINGS_FILE + ".ini"):
          print("Settings file does not exist, will be created")

        s = RTIMU.Settings(SETTINGS_FILE)
        imu = RTIMU.RTIMU(s)

        print("IMU Name: " + imu.IMUName())

        if (not imu.IMUInit()):
            print("IMU Init Failed")
            sys.exit(1)
        else:
            print("IMU Init Succeeded")
        
        imu.setSlerpPower(0.02)
        imu.setGyroEnable(True)
        imu.setAccelEnable(True)
        imu.setCompassEnable(True)

        poll_interval = imu.IMUGetPollInterval()
        self.dts = poll_interval * 1.0/1000.
        
        z = self.z  ## make local copy of self.z
        dts = self.dts  ## sample rate

        #get data
        if imu.IMURead():
            print "sensor started successfully"
        else:
            print "sensor didn't start."
            
        data = imu.getIMUData()
        x_a0L, y_a0L, z_acc = data['accel']
        ax_cal = x_a0L
        ay_cal = y_a0L

        x_a0L -= ax_cal
        y_a0L -= ay_cal
        print "acclerations: ", ax_cal, ay_cal
        omega_x, omega_y, t_v0 = data['gyro']

        #convert acceleration in local coords to global
        # minus signs to indicate accelerations is in opposite direction of sensor
        x_a0 = -x_a0L*cos(z[2,0]) + -y_a0L*cos(z[2,0]-pi/2)
        y_a0 = -y_a0L*sin(z[2,0]-pi/2) + -x_a0L*sin(z[2,0])
        
        t_v0 *= pi/180 #convert deg/sec to rad/sec
        time.sleep(3)
        count = 0
        while True:
            # get next sensor data
            if imu.IMURead():
                data = imu.getIMUData()
                x_a1L, y_a1L, z_acc = data['accel']
                omega_x, omega_y, t_v1 = data['gyro']

                # subtract out the initial acceleration measurements
                x_a1L -= ax_cal
                y_a1L -= ay_cal

                # convert to global coords
                x_a1 = -x_a1L*cos(z[2,0]) + -y_a1L*cos(z[2,0]-pi/2)
                y_a1 = -y_a1L*sin(z[2,0]-pi/2) + -x_a1L*sin(z[2,0])
                
                t_v1 *= pi/180
                
                x_dv = x_a0*dts + .5*dts*(x_a1-x_a0)
                y_dv = y_a0*dts + .5*dts*(y_a1-y_a0)
                #t_dv = t_a0*dts + .5*dts*(t_a1-t_a0)

                z[3,1] = z[3,0] + x_dv
                z[4,1] = z[4,0] + y_dv
                #z[5,1] = z[5,0] + t_dv
                z[5,1] = t_v1

                x_dp = z[3,0]*dts + .5*dts*(z[3,1] - z[3,0])
                y_dp = z[4,0]*dts + .5*dts*(z[4,1] - z[4,0])
                t_dp = z[5,0]*dts + .5*dts*(z[5,1] - z[5,0])

                z[0,1] = z[0,0] + x_dp
                z[1,1] = z[1,0] + y_dp
                z[2,1] = (z[2,0] + t_dp)%(2*pi)

                time.sleep(dts)
                x_a0 = x_a1
                y_a0 = y_a1
                t_v0 = t_v1
                z[:,0] = z[:,1]
                
                self.z = z  # update self.z
        
    def get_wheel_revs(self):
        # returns the number of wheel revolutions since the last time it was called
        odo = self.odo  ## get current odometer reading
        lastOdo = self.lastOdo ## reference last odometer reading
        
##        odoL2 = odo[0]
##        odoR2 = odo[1]
        odoL2 = self.cb1.tally()
        odoR2 = self.cb2.tally()
        delta_odoL = float(odoL2 - lastOdo[0]) * self.motorDirection[0]
        delta_odoR = float(odoR2 - lastOdo[1]) * self.motorDirection[1]

        if delta_odoL != 0:
            revsL = delta_odoL / self.encoderRatio
        else:
            revsL = 0
        if delta_odoR != 0:
            revsR = delta_odoR / self.encoderRatio
        else:
            revsR = 0

        self.lastOdo[0] = odoL2  ## update last odometer reading
        self.lastOdo[1] = odoR2
        return revsL, revsR
            

    def get_position_errors(self, currentState, targetState):
    ##    Takes as input the current state and the target state.
    ##    Returns the normal distance from state to target and the heading error theta2
        
        dx = targetState[0] - currentState[0]
        dy = targetState[1] - currentState[1]
        theta = currentState[2]

        vec1 = [cos(theta), sin(theta)]  ## vector pointing in direction of heading

        vec2 = [dx, dy]  ## vector pointing from current state to target state in plane

        error_dist = np.dot(vec2, vec1) / np.linalg.norm(vec1)  ## normal distance from sate to target

        cross = np.cross(vec1, vec2)
        dot = np.dot(vec1, vec2)

        sin_theta2 = cross / (np.linalg.norm(vec1) * np.linalg.norm(vec2))
        error_heading = np.arccos( np.clip(  dot/(np.linalg.norm(vec1) * np.linalg.norm(vec2)),-1.0,1.0) )

        if cross < 0:
            error_heading *= -1

        return error_dist, error_heading


    def get_motor_inputs(self, error_dist, error_heading, Kv, Kt, u_max):
    ##    Takes as inputs the current distance and heading errors, and feedback gains, and max forward speed
    ##    Returns the required motor inputs uL and uR

        saturation_factor = 1.0  ## derate inputs to avoid saturation
        v = Kv*error_dist
        omega = Kt*error_heading

        uL = (v - omega*self.d)/self.r
        uR = (v + omega*self.d)/self.r
        
        while abs(uL) > u_max*saturation_factor or abs(uR) > u_max*saturation_factor:
            if abs(uL) > u_max*saturation_factor:
                scale = abs((saturation_factor*u_max) / uL)
            else:
                scale = abs((saturation_factor*u_max) / uR)
            uL *= scale
            uR *= scale
                
        return uL, uR


    def drive(self, uL=0, uR=0):
    ##    Sets the motor speeds to uL and uR (mapped to an int from 0 to 255)
    ##    also updates the motor directions so that the odometers count up or down correctly

        if uL < 0:
          self.leftMotor.run(Adafruit_MotorHAT.BACKWARD)
          self.motorDirection[0] = -1
          uL = abs(uL)
        else:
          self.leftMotor.run(Adafruit_MotorHAT.FORWARD)
          self.motorDirection[0] = 1

        if uR < 0:
          self.rightMotor.run(Adafruit_MotorHAT.BACKWARD)
          self.motorDirection[1] = -1
          uR = abs(uR)
        else:
          self.rightMotor.run(Adafruit_MotorHAT.FORWARD)
          self.motorDirection[1] = 1

        if uL < .01:
            pwmL = 0
        else:
            pwmL = int(np.interp(uL, [0, self.u_max], [self.minpwmL, 255]))
        if uR < .01:
            pwmR = 0
        else:
            pwmR = int(np.interp(uR, [0, self.u_max], [self.minpwmR, 255]))

        self.leftMotor.setSpeed(pwmL)
        self.rightMotor.setSpeed(pwmR)

    def estimate_state(self, x_hp, u, dt):
    ##    estimates the current state using a Kalman filter. The accelerometer is used
    ##    as a sensor. Takese the previous state estimate and last motor inputs as inputs
      
        # make local copies of variables
        r = self.r
        d = self.d
        z = self.z
        
        A = np.matrix(np.eye(6))
        A[0,3] = dt
        A[1,4] = dt
        A[2,5] = dt

        ## Relates state to sensor variables
        H = np.matrix(np.eye(6))

        P_0 = np.matrix(np.eye(6))
        
        # Q = 0 to ignore accelerometers
        Q = (.01**2)*np.matrix(np.eye(6))  ## Covariance matrix of noise in linear model
        
        P = A*P_0*A.T + Q  ## updated covariance matrix of state variables
        
        R = (.1**2)*np.matrix(np.eye(6))  ## Covariance matrix of sensor noise

        #K = P*H.T*np.linalg.inv( H*P*H.T + R )  ## Kalman gain

        K = np.matrix(np.array([ [0.17277, 0.00000,0.00000,0.05606,0.00000,0.00000],
                                   [0.00000,0.17277,0.00000,0.0000,0.05606,-0.00000],
                                   [0.00000,0.00000,0.17277,0.00000,0.00000,0.05606],
                                   [0.05606,0.00000,0.00000, 0.07191,0.00000,0.00000],
                                   [0.00000,0.05606,0.00000,0.00000,0.07191,0.00000],
                                   [0.00000,-0.00000,0.05606,0.00000,0.00000,0.07191] ]))
    

        ################################ Feedforward Prediction ################################
        x_h = np.matrix(np.array( self.feedforward_prediction(x_hp, u, dt) )).T

        ################################## Sensor prediction #################################
        # get z from self which is being continuously updated by sensors
        z = np.matrix(np.array(z[:,1])).T  
##        try:
##            self.zRecord[:,self.zCount] = np.squeeze(np.asarray(z))
##            break
        

        ################################### Update ##############################################
        x_h2 = x_h + K*(z - H*x_h)*0

        return np.squeeze(np.asarray(x_h2))

    def feedforward_prediction(self, x_hp, u, dt):
        xp = x_hp[0]
        yp = x_hp[1]
        uL = u[0]
        uR = u[1]
        theta_p = x_hp[2]
        theta_dot_p = x_hp[5]
        
        theta = theta_p + theta_dot_p*dt
        
        theta_dot = self.r/(2*self.d) * ( -uL + uR )
        x_dot = (self.r/2)*cos(theta_p) * (uL + uR)

        y_dot = (self.r/2)*sin(theta_p) * (uL + uR)
        
        x = xp + x_dot*dt
        y = yp + y_dot*dt

        x_h = [x, y, theta, x_dot, y_dot, theta_dot]

        return np.squeeze(np.asarray(x_h))


    def turnip(self, angle_deg):
        #"turn in place"
        # positive angle turns to the left.
        print "turning"
        angle = abs(angle_deg*(pi/180.))
        revsL = 0.
        revsR = 0.
        while abs(revsL)*2*pi < float(self.d*angle/(self.r)) \
              and abs(revsR)*2*pi < float(self.d*angle/(self.r)):
            drevsL, drevsR = self.get_wheel_revs()
            print drevsL, drevsR
            revsL += drevsL
            revsR += drevsR
            if angle_deg > 0:
                self.drive(-self.u_max/2, self.u_max/2)
            else:
                self.drive(self.u_max/2, -self.u_max/2)
            
        self.drive(0,0)

    def find_minimum_pwm(self):
        # applies gradually increasing wheel speed to the motors until the
        # motors begin to move, then returns that minimum desired wheel speed
        # that results in forward motion

        #inch left wheel forward until it budges
        self.drive(0.,0.)
        self.minpwmL = 0
        self.minpwmR = 0
        i = 0
        initLeftTicks = self.cb1.tally()
        while self.cb1.tally() - initLeftTicks < 5:
            self.drive(i, 0)
            i += 1
            time.sleep(.4) # give the motors a chance to speed up
            
        self.drive(0.,0.)
        self.minpwmL = int(i*255/self.u_max)

        # inch right wheel forward. . .
        i = 0
        initRightTicks = self.cb2.tally()
        while self.cb2.tally() - initRightTicks < 5:
            self.drive(0, i)
            i += 1
            time.sleep(.4) # give the motors a chance to speed up
            
        self.drive(0.,0.)
        self.minpwmR = int(i*255/self.u_max)

        return self.minpwmL, self.minpwmR

    def find_maximum_speed(self):
        # applies full speed to the wheels and measures how fast the robot
        # can actually travel in a straight line

        revsL, revsR = self.get_wheel_revs()
        t1 = time.time()
        self.drive(100, 100)
        while revsL < 2. and revsR < 2.:
            drevsL, drevsR = self.get_wheel_revs()
            revsL += drevsL
            revsR += drevsR
            print revsL, revsR
            
        t2 = time.time()    
        self.drive(0.,0.)
        leftSpeed = revsL*2*pi / (t2-t1)
        rightSpeed = revsR*2*pi / (t2-t1)
        self.u_max = min(leftSpeed, rightSpeed)

        return self.u_max

    def get_range(self):
        # max range in microsecond pulse
        MAX_DIST = 23200

        # set trigger pin high for at least 10 us
        self.mypi.write(self.TRIG_PIN, 1)
        time.sleep(10.*(10**-6))
        self.mypi.write(self.TRIG_PIN, 0)

        while self.mypi.read(self.ECHO_PIN) == 0:
            pass
        t1 = time.time()
        
        while self.mypi.read(self.ECHO_PIN) == 1:
            if time.time() > t1+1.0:
                raise Exception('Timeout: took too long to receive echo signal')
            pass
        t2 = time.time()
        
        pulse_width = (t2 - t1) * 10**6
        meters = pulse_width / 5800.

        if pulse_width > MAX_DIST:
            return 0
        else:
            return meters

    def turn_servo(self, angle_deg):
        #takes in an angle from 0 to 180 and sets the servo to it
        # I think the servo needs 5V to work well.
        # futaba s3003 wants a 50 Hz signal
 
        duty = angle_deg / 180.
        off = np.interp(angle_deg, [0,180], [150, 525])
        self.servo.setPWM(14, 0, int(off))


    def error_norm(self, state):
        # returns the euclidean distance (2-norm) of the current state
        # from the origin
        norm = np.linalg.norm(state[0:2], 2)
        return norm

    def p2p(self, forward, left, threshold=.06, timeout=10, Kv = 1, Kt = 1.5):
        # point - to - point
        # takes x and y coords (relative to rob)
        # rob moves to that location ignoring heading
        x_h = self.x_h
        x_target = np.add(x_h, [forward, left, 0, 0, 0, 0])
        t1 = time.time()
        totalTime = 0

        while self.error_norm(np.subtract(x_target, x_h)) > threshold \
              and totalTime < timeout:
            error_dist, error_heading = self.get_position_errors(x_h, x_target)
            
            # determine what wheel speeds should be
            uL, uR = self.get_motor_inputs(error_dist, error_heading, Kv, Kt,
                                          self.u_max)
            # apply wheel speeds
            self.drive(uL, uR)
            time.sleep(.2)

            # update state estimate
            t2 = time.time()
            dt = t2 - t1
            totalTime += dt
            t1 = t2

            # estimate what each wheel speed (rad/sec) actually was from encoders
            revsL, revsR = self.get_wheel_revs()
            uLh = 2*pi*revsL/dt
            uRh = 2*pi*revsR/dt

            x_h = self.estimate_state(x_h, [uLh, uRh], dt)
            self.x_h = x_h
            print x_h
        print "\n norm:", self.error_norm(np.subtract(x_target, x_h))
        print "total time: ", totalTime
        self.drive(0,0)


    def keyboard_control(self):
        ''' drive the car using a keyboard keys'''

        main = tk.Tk()
        def forward(event):
            self.drive(uL=100, uR=100)

        def left(event):
            self.drive(uL=0, uR=100)

        def right(event):
            self.drive(uL=100, uR=0)

        def stop(event):
            self.drive()

        frame = tk.Frame(main, width=100, height=100)
        main.bind('<Left>', left)
        main.bind('<Right>', right)
        main.bind('<Up>', forward)
        main.bind('<KeyRelease-Left>', stop)
        main.bind('<KeyRelease-Right>', stop)
        main.bind('<KeyRelease-Up>', stop)

        frame.pack()
        frame.mainloop()
        
        
            


        
    
#robot.turnOffMotors()

if __name__ == '__main__':
    rob = robot()
    




        
