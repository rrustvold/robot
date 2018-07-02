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
import time

def clean_up():
  connection.shutdown(socket.SHUT_RDWR)
  connection.close()
  mySocket.shutdown(socket.SHUT_RDWR)
  mySocket.close()
  recSocket.shutdown(socket.SHUT_RDWR)
  recSocket.close()
  vidSocket.shutdown(socket.SHUT_RDWR)
  vidSocket.close()
  video.stop()
  receiver.stop()
  pygame.quit()

def receive_data():
  ''' data format is [distance, servoAngle] '''
  global data
  while True:
    message = recSocket.recvfrom(SIZE)[0].split(',')
    data['distance'] = float(message[0])
    data['servoAngle'] = float(message[1])

    

def stream_video():
  global bgImage
  global imgForCv
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

        imgForCv = np.array(image)
        imgForCv = cv2.cvtColor(imgForCv, cv2.COLOR_RGB2BGR)
        
        bgImage = pygame.surfarray.make_surface(np.array(image))

      except Exception as e:
        print "stream video"
        print e
        print '\n'

  finally:
      connection.close()
      vidSocket.close()

def tracker_init(target=(340,240)):
  global track_window
  global roi_hist
  global term_crit
  global hueToTrack
  global HSVUpper
  global HSVLower
  
  # initial tracking window location
  h = 20
  w = 20
  r = target[0]-h
  c = target[1]-w
  track_window = (c,r,w,h)

  # setup up ROI for tracking

  roi = imgForCv[r:r+h, c:c+w]
  hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

  hueToTrack = hsv_roi[h/2,w/2][0]
  HSVLower = np.array((hueToTrack-15., 50., 50.))
  HSVUpper = np.array((hueToTrack+15., 255., 255.))

  # mask color
  mask = cv2.inRange(hsv_roi, HSVLower, HSVUpper)
  roi_hist = cv2.calcHist([hsv_roi],[0],mask,[180],[0,180])
  cv2.normalize(roi_hist,roi_hist,0,255,cv2.NORM_MINMAX)

  # Setup the termination criteria, either 10 iteration or move by atleast 1 pt
  term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )


def locate_object(image):
  ''' Applies camshift to the global tracker window '''
  global track_window
  global roi_hist
  global term_crit
  global center
  
  
  hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
  dst = cv2.calcBackProject([hsv], [0], roi_hist, [0,180], 1)
  # apply meanshift to get the new location
  ret, track_window = cv2.CamShift(dst, track_window, term_crit)
  pts = cv2.boxPoints(ret)
  moments = cv2.moments(pts)
  pts = np.int0(pts)

  try:
    center[1] = int(moments['m10'] / moments['m00'])
    center[0] = int(moments['m01'] / moments['m00'])
  except:
    pass

  if displayMode == 0:
    return cv2.cvtColor(cv2.polylines(image, [pts], True, 255, 2),
                      cv2.COLOR_BGR2RGB)
  if displayMode == 1:
    
    mask = cv2.inRange(hsv, HSVLower, HSVUpper)
    res = cv2.bitwise_and(image, image, mask=mask)
    return cv2.cvtColor(cv2.polylines(res, [pts], True, 255, 2),
                      cv2.COLOR_BGR2RGB)

def locate_face(image):
  global face_cascade
  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
  face = face_cascade.detectMultiScale(gray, 1.1, 3)
  print face
  for (x,y,w,h) in face:
    print "got one"
    cv2.rectangle(image, (x,y), (x+w, y+h), (255,0,0), 2)

  return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
##  return gray


bgImage = None
imgForCv = None  


address = "C:\opencv\sources\data\haarcascades\\"
classifier = 'haarcascade_frontalface_default.xml'

cascade = address + classifier
face_cascade = cv2.CascadeClassifier(cascade)
if face_cascade.empty():
    print cascade
    print "empty"

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



atexit.register(clean_up)

pygame.init()
clk = pygame.time.Clock()
joy = pygame.joystick.Joystick(0)
joy.init()

margin = 100
size = width, height = 640+margin*2, 480+margin*2
frameWidth = 640
frameHeight = 480



screen = pygame.display.set_mode(size)
pygame.display.set_caption("Tester")

frameRect = pygame.Rect((margin,margin), (width-margin*2,height-margin*2))

crosshair = pygame.surface.Surface((10,10))
crosshair.fill(pygame.Color("magenta"))
pygame.draw.circle(crosshair, pygame.Color("blue"), (5,5), 5, 0)
crosshair.set_colorkey(pygame.Color("magenta"), pygame.RLEACCEL)
crosshair = crosshair.convert()

dial = pygame.surface.Surface((80,80))
colorSquare = pygame.surface.Surface((75,75))
colorRect = pygame.Rect((0,0), (margin,margin))

writer = pygame.font.Font(pygame.font.get_default_font(), 15)
buttons = {}

data = {'distance':0, 'servoAngle':90}
center = [0,0]

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




  
receiver = threading.Thread(target=receive_data)
receiver.start()

video = threading.Thread(target=stream_video)
video.start()

time.sleep(4)

tracker_init()

displayModes = ['normal', 'mask']
numDisplayModes= len(displayModes)
displayMode = 0
followObject = False

while True:
  pygame.event.pump()

  screen.fill(pygame.Color("black"))
  
  for events in pygame.event.get():
    if events.type == pygame.QUIT:
      pygame.quit()
      sys.exit()

##  try:
##    screen.blit(bgImage, [margin,margin])
##
##
##  except Exception as e:
##    print "blit: "
##    print e
  try:
    window = pygame.surfarray.make_surface(locate_object(imgForCv))
    screen.blit(window, [margin,margin])
  except Exception as e:
    print e

  # x, y, and z are the roll, pitch, and yaw axes
  # they are a value from [-1,1]

  
  if followObject:
    x = np.interp(center[0], [0,640], [-1,1])
    y = -.5
    z = 0
  else:
    x = joy.get_axis(0)  # positive is to the right
    y = joy.get_axis(1)
    z = joy.get_axis(3)
    

  hat = joy.get_hat(0)[0]
  motorBias = joy.get_axis(2)

  message = "{}, {}, {}, {}, {}".format(x,y,z,hat, motorBias)
  
  if not joy.get_button(2):
    mySocket.sendto(message, (RASPI_IP,PORT_NUMBER))

  #Draw the cross hair
  screen.blit(crosshair, ((.97*x*frameWidth/2)+frameWidth/2-5+margin,
                          (.97*y*frameHeight/2)+frameHeight/2-5+margin))
  
  # Draw the border around the webcam view
  pygame.draw.rect(screen, pygame.Color("red"), frameRect, 1)

  #Draw the range detector value in the upper right corner
  rangeTxt = writer.render(
            "Distance = {0:.2f} m, servo angle = {1:.2f}, Bias = {2}".format(
             data['distance'], data['servoAngle'], motorBias),
                  1,
                  pygame.Color("red"),
                  pygame.Color("black"),
                  ).convert()
    
  screen.blit(rangeTxt, (50,50))

  # Draw the color tracker and mode in bottom left
  colorTxt = writer.render("Color Tracking: {}. Display mode: {}".format(
                           hueToTrack, displayMode),
                           1,
                           pygame.Color("red"),
                           pygame.Color("black"),
                           ).convert()
  screen.blit(colorTxt, (margin/2, height-margin/2))

  #Draw a color square
  color = pygame.Color("red")
  color.hsva = (hueToTrack*2, 100, 100, 100)
  pygame.draw.rect(colorSquare, color, colorRect)
  screen.blit(colorSquare, (margin/2, height-(margin*2)))
  
  # Draw the servo direction indicator in the upper right corner
  dial.fill(pygame.Color("black"))
  pygame.draw.circle(dial, pygame.Color("red"), (40,40), 40, 2)
  pygame.draw.line(dial, pygame.Color("red"), (40,40),
                (-1*int(40*np.cos(data['servoAngle']*np.pi/180.)) + 40 ,
                 -1*int(40*np.sin(data['servoAngle']*np.pi/180.)) + 40) )

  dial.convert()

  screen.blit(dial, (width/2-40, height-margin))

  for b in range(joy.get_numbuttons()):
    if joy.get_button(b):
      screen.blit(buttons[b][0], buttons[b][1])

  if joy.get_button(0):
    xpx = int(np.interp(x, [-1,1], [0,640]))
    ypx = int(np.interp(y, [-1,1], [0,480]))
    try:
      tracker_init(target=(xpx,ypx))
    except:
      pass

  if joy.get_button(3):
    displayMode = (displayMode + 1) % numDisplayModes
    time.sleep(.2)

  if joy.get_button(1):
    followObject = not followObject
    time.sleep(.2)

  
  
  pygame.display.flip()
  clk.tick(40)


  
