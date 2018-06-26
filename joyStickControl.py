from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_PWM_Servo_Driver
import numpy as np
import os.path
import math
import sys, getopt
import atexit
import time
import usb.core, usb.util
import pigpio

from socket import socket, gethostbyname, AF_INET, SOCK_DGRAM
import sys
PORT_NUMBER = 5000
SERVER_IP = '192.168.1.21'
SIZE = 1024

hostName = gethostbyname( '0.0.0.0' )

mySocket = socket( AF_INET, SOCK_DGRAM )
mySocket.bind( (hostName, PORT_NUMBER) )

class robot():

    def __init__(self):
        self.u_max = 255
        self.motorDirection = [1,1]
        self.minpwmL = 0
        self.minpwmR = 0

        # for gpio pins
        self.mypi = pigpio.pi()

        # for the ultrasonic sensor
        self.TRIG_PIN = 22
        self.ECHO_PIN = 23
        self.mypi.set_mode(self.TRIG_PIN, pigpio.OUTPUT)
        self.mypi.set_mode(self.ECHO_PIN, pigpio.INPUT)
        self.useUltrasonic = False

        # servo motor
        self.servo = Adafruit_PWM_Servo_Driver.PWM(address=0x60)
        self.servoAngle = 90
        self.servo.setPWMFreq(50)
        self.turn_servo(self.servoAngle)

        # DC Drive motors
        self.mh = Adafruit_MotorHAT(addr=0x60)
        self.turnOffMotors()
          
        atexit.register(self.turnOffMotors)

        ## DC motor test
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

    def turnOffMotors(self):
        self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

    def turn_servo(self, angle_deg):
        '''takes in an angle from 0 to 180 and sets the servo to it
        I think the servo needs 5V to work well.
        futaba s3003 wants a 50 Hz signal'''
 
        duty = angle_deg / 180.
        off = np.interp(angle_deg, [0,180], [150, 525])
        self.servo.setPWM(14, 0, int(off))

    def drive(self, uL=0, uR=0):
        '''Sets the motor speeds to uL and uR (mapped to an int from 0
        to 255) also updates the motor directions so that the odometers
        count up or down correctly.
        '''
        
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

    def get_range(self, timeout=.1):
        ''' get the distance as measured by the ultrasonic distance sensor
        in meters. Returns -1 if out of range (4m)'''
        
        # max range in microsecond pulse
        MAX_DIST = 23200

        # set trigger pin high for at least 10 us
        self.mypi.write(self.TRIG_PIN, 1)
        time.sleep(10.*(10**-6))
        self.mypi.write(self.TRIG_PIN, 0)
        
        t0 = time.time()
        while self.mypi.read(self.ECHO_PIN) == 0:
            if time.time() > t0 + timeout:
                raise Exception('Timeout: took too long to receive trigger signal')
            
        t1 = time.time()
        
        while self.mypi.read(self.ECHO_PIN) == 1:
            if time.time() > t1 + timeout:
                raise Exception('Timeout: took too long to receive echo signal')
            
        t2 = time.time()
        
        pulse_width = (t2 - t1) * 10**6
        meters = pulse_width / 5800.

        if pulse_width > MAX_DIST:
            return -1
        else:
            return meters

    def joystick(self):
        '''
        turns on control via joystick
        '''
        self.servo.setPWMFreq(50)
        import pygame
        pygame.init()
        clk = pygame.time.Clock()
        joy = pygame.joystick.Joystick(0)
        joy.init()

        size = width, height = 600, 600

        screen = pygame.display.set_mode(size)
        pygame.display.set_caption("Tester")

        frameRect = pygame.Rect((45,45), (510,510))

        crosshair = pygame.surface.Surface((10,10))
        crosshair.fill(pygame.Color("magenta"))
        pygame.draw.circle(crosshair, pygame.Color("blue"), (5,5), 5, 0)
        crosshair.set_colorkey(pygame.Color("magenta"), pygame.RLEACCEL)
        crosshair = crosshair.convert()

        writer = pygame.font.Font(pygame.font.get_default_font(), 15)
        buttons = {}
        for b in range(joy.get_numbuttons()):
          buttons[b] = [
            writer.render(
              hex(b)[2:].upper(),
              1,
              pygame.Color("red"),
              pygame.Color("black")
              ).convert(),
            (15*b+45, 560)
             ]

        while True:
          pygame.event.pump()
          for events in pygame.event.get():
            if events.type == pygame.QUIT:
              pygame.quit()
              sys.exit()

          screen.fill(pygame.Color("black"))
          
          if self.useUltrasonic:
              try:
                  rangeTxt = writer.render(str(self.get_range()),
                                1,
                                pygame.Color("red"),
                                pygame.Color("black")
                                ).convert()
              except:
                  pass

              screen.blit(rangeTxt, (100,100))

          # x, y, and z are the roll, pitch, and yaw axes
          # they are a value from [-1,1]

          

          x = joy.get_axis(0)  # positive is to the right
          y = joy.get_axis(1)
          z = joy.get_axis(2)
          hat = joy.get_hat(0)

          
          

          # use the y (pitch) as a throttle and x (roll) for direction

          #scale uL and uR by y
          uL = uR = self.u_max*y*-1

          if x > 0:
              uR = (1-x)*uR
              self.drive(uL, uR)
          else :
              uL = (1+x)*uL
              self.drive(uL, uR)
              
          if abs(z) > .05 and abs(y) < .2:
              uL = self.u_max*z
              uR = -self.u_max*z
              self.drive(uL, uR)

          # turn the servo motor from the hat
          if hat[0] != 0:
              # turn the servo
              self.servoAngle -= 2*hat[0]
              self.turn_servo(self.servoAngle)

          screen.blit(crosshair, ((x*250)+300-5, (y*250)+300-5))
          pygame.draw.rect(screen, pygame.Color("red"), frameRect, 1)

          for b in range(joy.get_numbuttons()):
            if joy.get_button(b):
              screen.blit(buttons[b][0], buttons[b][1])
                      
          if joy.get_button(0):
              self.useUltrasonic = not self.useUltrasonic
              
          pygame.display.flip()
          clk.tick(40)


    def remote_control(self,):
        self.servo.setPWMFreq(50)
        while True:
          data, _ = mySocket.recvfrom(SIZE)
          x, y, z, hat = data.split(',')
          x = float(x)
          y = float(y)
          z = float(z)
          hat = float(hat)

          # saturate y to zero so that we don't get a trickle of a signal
          if abs(y) < .1:
              y = 0
            
            # use the y (pitch) as a throttle and x (roll) for direction

          #scale uL and uR by y
          uL = uR = self.u_max*y*-1

          if x > 0:
              uR = (1-x)*uR
              self.drive(uL, uR)
          else :
              uL = (1+x)*uL
              self.drive(uL, uR)
              
          if abs(z) > .05 and abs(y) < .2:
              uL = self.u_max*z
              uR = -self.u_max*z
              self.drive(uL, uR)

          # turn the servo motor from the hat
          if hat != 0:
              # turn the servo
              self.servoAngle -= 2*hat
              self.turn_servo(self.servoAngle)

          
        
if __name__ == '__main__':
    rob = robot()
    rob.remote_control()
    
