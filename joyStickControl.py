from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_PWM_Servo_Driver
import numpy as np
import os.path
import math
import sys, getopt
import atexit
import Tkinter as tk
import time
import usb.core, usb.util

class robot():

    def __init__(self):
        self.u_max = 255
        self.motorDirection = [1,1]
        self.minpwmL = 0
        self.minpwmR = 0
        
        self.mh = Adafruit_MotorHAT(addr=0x60)
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

    def turnOffMotors(self):
        self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

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

    def joystick(self):
        '''
        turns on control via joystick
        '''
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
              pygame.Color("black"),
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

          x = joy.get_axis(0)
          y = joy.get_axis(1)
          z = joy.get_axis(2)
          print "({}, {}, {})".format(x,y,z)
          self.drive(y*255,y*255)

          screen.blit(crosshair, ((x*250)+300-5, (y*250)+300-5))
          pygame.draw.rect(screen, pygame.Color("red"), frameRect, 1)

          for b in range(joy.get_numbuttons()):
            if joy.get_button(b):
              screen.blit(buttons[b][0], buttons[b][1])

          pygame.display.flip()
          clk.tick(40)


    
if __name__ == '__main__':
    rob = robot()
    rob.joystick()
    
