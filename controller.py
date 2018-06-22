import functions as fn
import numpy as np
import atexit
import time
import threading
from numpy import pi



## Setup ##

x_h = [0.]*6 ## initial condition
##fn.turn_on_sensor(x_h)

x_hp = x_h
x_target = [.4,0.,0,0,0,0]  ## target final condition

u_max = 144*.1047  ## maximum angular speed of wheels in rad/sec
wheelRadius = .065 #(m)
wheelbase = (6.25/2)*.0254 #(m) distance from wheel centerline to car centerline

distance_threshold = .03  ## threshold to consider success
heading_threshold = 3

## get_motion_plan()
## get_target()
simLength = int(1.5 / dt)
inputRec = np.zeros((2,simLength))
x_hatRec = np.zeros((6,simLength))
distErrorRec = np.zeros(simLength)
headErrorRec = np.zeros(simLength)
uL = 0.
uR = 0.
uLh = 0.
uRh = 0.
inputhRec = np.zeros((2,simLength))


rob = robot(wheelRadius, wheelbase, x_hp, u_max)

sensorThread = threading.Thread(target=rob.accel_sensor)
sensorThread.start()
print "moving on"

error_dist, error_heading = rob.get_position_errors(x_h, x_target)

totalTime = 0
t1 = time.time() 
i = 0
while totalTime < 5.0:
    print t1
    if abs(x_target[0] - x_h[0]) > distance_threshold or abs(x_target[1] - x_h[1]) > distance_threshold:
        ## determine current state error
        error_dist, error_heading = rob.get_position_errors(x_h, x_target)
        
        ## determne what the motor inputs should be
        Kv = 10000
        Kt = 10000
        uL, uR = rob.get_motor_inputs(error_dist, error_heading, Kv, Kt, u_max) ## how fast I want the wheels to spin
        print uL, uR
        ## apply the motor inputs
        rob.drive(uL, uR, dt, u_max)  ## how fast the wheels actually spinned

        ## update the state estimate
    else:
        rob.turnOffMotors()

    t2 = time.time()
    dt = t2 - t1
    totalTime += dt
    t1 = t2
    
    revsL, revsR = rob.get_wheel_revs()
    uLh = 2*pi*revsL/dt  #wheel speed rad/sec
    uRh = 2*pi*revsR/dt
    
    x_h = rob.estimate_state(x_h, [uLh, uRh], dt)

    x_hatRec[:,i] = x_h
    inputRec[:,i] = [uL, uR]
    inputhRec[:,i] = [uLh, uRh]
    distErrorRec[i] = error_dist
    headErrorRec[i] = error_heading

    time.sleep(.5)
    i += 1
    print x_h

sensorThread.end()    
rob.turnOffMotors()
np.savetxt('x_hat.txt', x_hatRec, delimiter=',')
np.savetxt('inputRec.txt', inputRec, delimiter=',')
np.savetxt('inputhRec.txt', inputRec, delimiter=',')
np.savetxt('distErrorRec.txt', distErrorRec, delimiter=',')
np.savetxt('headErrorRec.txt', headErrorRec, delimiter=',')


