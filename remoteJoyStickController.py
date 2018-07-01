import pygame
import sys
from socket import socket, AF_INET, SOCK_DGRAM, gethostbyname
import threading
import cv2
from PIL import Image
import numpy as np
import io, struct
import atexit
from StringIO import StringIO


#FOR SENDING
RASPI_IP   = '192.168.1.20'
PORT_NUMBER = 5000
SIZE = 1024
mySocket = socket( AF_INET, SOCK_DGRAM )

#FOR RECEIVING
PORT_NUMBER2 = 5001
hostName = gethostbyname('0.0.0.0')
recSocket = socket( AF_INET, SOCK_DGRAM )
recSocket.bind((hostName, PORT_NUMBER2))

# FOR VIDEO STREAM
vidSocket = socket()
vidSocket.bind(('0.0.0.0', 8000))
vidSocket.listen(0)
connection = vidSocket.accept()[0].makefile('rb')

def clean_up():
  connection.close()
  mySocket.close()
  recSocket.close()
  vidSocket.close()
  video.stop()
  receiver.stop()

atexit.register(clean_up)

pygame.init()
clk = pygame.time.Clock()
joy = pygame.joystick.Joystick(0)
joy.init()

margin = 100
size = width, height = 640+margin*2, 480+margin*2
frameWidth = 640
frameHeight = 480

bgImage = None

screen = pygame.display.set_mode(size)
pygame.display.set_caption("Tester")

frameRect = pygame.Rect((margin,margin), (width-margin*2,height-margin*2))

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

def receive_data():
  ''' data format is [distance, servoAngle] '''
  global data
  while True:
    message = recSocket.recvfrom(SIZE)[0].split(',')
    print message
    distance = float(message[0])
    servoAngle = float(message[1])
    data = "Distance = {0:.2f} m, servo angle = {1:.2f}".format(distance,
                                                                servoAngle)
    

def stream_video():
  global bgImage
  try:
    while True:
      try:
        # Read the length of the image as a 32-bit unsigned int. If the
        # length is zero, quit the loop
        image_len = struct.unpack('<L', connection.read(struct.calcsize('<L')))[0]
        if not image_len:
            break
        # Construct a stream to hold the image data and read the image
        # data from the connection
        image_stream = io.BytesIO()
        image_stream.write(connection.read(image_len))
        # Rewind the stream, open it as an image with PIL and do some
        # processing on it
        image_stream.seek(0)
        image = Image.open(image_stream).rotate(-90).transpose(
          Image.FLIP_TOP_BOTTOM)
        
        bgImage = pygame.surfarray.make_surface(np.array(image))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
      except Exception as e:
        print "stream video"
        print e
        print '\n'

  finally:
      connection.close()
      vidSocket.close()


  
receiver = threading.Thread(target=receive_data)
receiver.start()

video = threading.Thread(target=stream_video)
video.start()

data = ''

while True:
  pygame.event.pump()
  for events in pygame.event.get():
    if events.type == pygame.QUIT:
      pygame.quit()
      sys.exit()

  try:
    screen.blit(bgImage, [margin,margin])

  except Exception as e:
    print "blit: "
    print e

  # x, y, and z are the roll, pitch, and yaw axes
  # they are a value from [-1,1]

  x = joy.get_axis(0)  # positive is to the right
  y = joy.get_axis(1)
  z = joy.get_axis(3)
  hat = joy.get_hat(0)[0]

  message = "{}, {}, {}, {}".format(x,y,z,hat)
  mySocket.sendto(message, (RASPI_IP,PORT_NUMBER))

  screen.blit(crosshair, ((.97*x*frameWidth/2)+frameWidth/2-5+margin,
                          (.97*y*frameHeight/2)+frameHeight/2-5+margin))
  
  pygame.draw.rect(screen, pygame.Color("red"), frameRect, 1)

  for b in range(joy.get_numbuttons()):
    if joy.get_button(b):
      screen.blit(buttons[b][0], buttons[b][1])

  if joy.get_button(1):
    pygame.quit()
    sys.exit()

              
  rangeTxt = writer.render(data,
                  1,
                  pygame.Color("red"),
                  pygame.Color("black"),
                  ).convert()
    
  screen.blit(rangeTxt, (50,50))
  
  pygame.display.flip()
  clk.tick(40)
