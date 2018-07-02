from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_PWM_Servo_Driver
import numpy as np
import os.path
import math
import sys, getopt
import atexit
import time
import usb.core, usb.util
import pigpio
import threading
import picamera
import io, struct
from socket import socket, gethostbyname, AF_INET, SOCK_DGRAM
import atexit

def clean_up():
  connection.shutdown(socket.SHUT_RDWR)
  connection.close()
  mySocket.shutdown(socket.SHUT_RDWR)
  mySocket.close()
  sendSocket.shutdown(socket.SHUT_RDWR)
  sendSocket.close()
  vidSocket.shutdown(socket.SHUT_RDWR)
  vidSocket.close()
  video.stop()
  receiver.stop()

atexit.register(clean_up)
  
# for receiving input commands
PORT_NUMBER = 5000
SIZE = 1024
hostName = gethostbyname( '0.0.0.0' )
mySocket = socket( AF_INET, SOCK_DGRAM )
mySocket.bind( (hostName, PORT_NUMBER) )

#for sending distance data back
PORT_NUMBER2 = 5001
LAPTOP_IP = '192.168.1.25'
sendSocket = socket( AF_INET, SOCK_DGRAM )

#for video streaming
vidSocket = socket()
vidSocket.connect((LAPTOP_IP, 8000))
connection = vidSocket.makefile('wb')




class robot():

    def __init__(self):
        self.u_max = 255
        self.motorDirection = [1,1]
        self.minpwmL = 0
        self.minpwmR = 0
        self.motorBias = 0

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
        uL *= (1+self.motorBias)
        uR *= (1-self.motorBias)
        
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


    def remote_control(self,):
        ''' receives a control signal from another PC on the network '''
        
        self.servo.setPWMFreq(50)
        while True:
          data, _ = mySocket.recvfrom(SIZE)
          x, y, z, hat, motorBias = data.split(',')
          x = float(x)
          y = float(y)
          z = float(z)
          hat = float(hat)
          self.motorBias = float(motorBias)

          # saturate y to zero so that we don't get a trickle of a signal
          if abs(y) < .1:
              y = 0
            
            # use the y (pitch) as a throttle and x (roll) for direction

          #scale uL and uR by y
          uL = uR = self.u_max*y*-1

          if x > 0 and abs(y) > .2:
              uR = (1-x)*uR
              self.drive(uL, uR)
              
          elif abs(y) > .2:
              uL = (1+x)*uL
              self.drive(uL, uR)
              
          elif abs(z) > .05 and abs(y) < .2:
              uL = self.u_max*z
              uR = -self.u_max*z
              self.drive(uL, uR)

          else:
              self.drive()

          # turn the servo motor from the hat
          if hat != 0:
              # turn the servo
              self.servoAngle -= 2*hat
              self.turn_servo(self.servoAngle)
          
            
    def send_data(self,):

        while True:
            try:

              dist = self.get_range()
              message = str(dist) + ', ' + str(self.servoAngle)
              print message
              sendSocket.sendto(message, (LAPTOP_IP, PORT_NUMBER2))
              time.sleep(1.5)

            except:
                  pass


    def stream_video(self):

        try:
            camera = picamera.PiCamera()
            camera.resolution = (640, 480)
            camera.framerate = 12
            # Start a preview and let the camera warm up for 2 seconds
            camera.start_preview()
            time.sleep(2)

            # Note the start time and construct a stream to hold image data
            # temporarily (we could write it directly to connection but in this
            # case we want to find out the size of each capture first to keep
            # our protocol simple)
            start = time.time()
            stream = io.BytesIO()
            for foo in camera.capture_continuous(stream, 'jpeg',
                                                 use_video_port=True):
                # Write the length of the capture to the stream and flush to
                # ensure it actually gets sent
                connection.write(struct.pack('<L', stream.tell()))
                connection.flush()
                # Rewind the stream and send the image data over the wire
                stream.seek(0)
                connection.write(stream.read())
                # If we've been capturing for more than 30 seconds, quit
##                    if time.time() - start > 10000000000:
##                        break
                # Reset the stream for the next capture
                stream.seek(0)
                stream.truncate()
            # Write a length of zero to the stream to signal we're done
            connection.write(struct.pack('<L', 0))
        finally:
            connection.close()
            vidSocket.close()
                
        
            
if __name__ == '__main__':
    rob = robot()
    sendThread = threading.Thread(target=rob.send_data)
    sendThread.start()
    
    videoThread = threading.Thread(target=rob.stream_video)
    videoThread.start()
    rob.remote_control()
    
