# -*- coding: utf-8 -*-
"""
Created on Thu Apr 30 01:11:23 2015

@author: anwar.maxsum
"""

#simExtRemoteApiStart(19999)

import vrep
import numpy
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys
import time

import robot_sys as robo
import particle_filter as pf

clientID = robo.connection_init()

particle_coord = [-1.8250, 1.9000, 0.1388]
particle_angle = [0,0,115]

i = 0
v = 0.05

offSetX = 0
offsetY = 0
initX = 0.5
initY = -2.0


#initialize map
mapCoord , mapWallOrientation = robo.map_generate(clientID)
mapAll = [mapCoord, mapWallOrientation]

#initialize robot handler - real and particlebot
robot_handler = robo.roboObj_init(clientID,'Pioneer_p3dx')
particle_handler = robo.roboObj_init(clientID,'particlebot')

#initialize robot's motor handler
left_motor, right_motor = robo.motor_init(clientID)

#initialize robot's sensorreal robot and particlebot
sensorH = robo.sensor_init(clientID)
sensorH_particle = robo.particle_sensor_init(clientID)

#initialize GPS - use if necessary
pX = pY = 0.0
for i in range (1,30) :
    errorCode,gpsX=vrep.simxGetFloatSignal(clientID,'gpsX',vrep.simx_opmode_oneshot_wait);
    errorCode,gpsY=vrep.simxGetFloatSignal(clientID,'gpsY',vrep.simx_opmode_oneshot_wait);
    pX += gpsX
    pY += gpsY
    
offSetX = (pX/30) - initX
offSetY = (pY/30) - initY

#initialize particles - uniform
particle_amount = 50
particle_bundle = particle_init(particle_amount)

#dummy action list - only moving straight with vr=vl=2
actionNow = [2,2] #edit this to list of movement to navigate around the rooms

#iterate!
for i in range(1,10):
    #while True :
    print 'Iteration : ', i  
    
    #initialize map plotting 
    fig = plt.figure()
    
    #gps reading
    gpsX , gpsY = robo.gps_read(clientID)
    gpsX = gpsX - offSetX
    gpsY = gpsY - offSetY
    
    #get real coordinate
    realCoord, realAngle = robo.pose(clientID,robot_handler)
    realX = realCoord[0]
    realY = realCoord[1]

    #move a robot for 1 second    
    robo.motor_move(clientID,left_motor,3,right_motor,3)
    time.sleep(1)
    robo.motor_move(clientID,left_motor,0,right_motor,0)
    
    #read sensor value    
    rStat, rVal = robo.sensor_readAll(clientID,sensorH)
    print 'read-real', rVal    
      
    print '-------------------'    
    
    #print 'position: ', realX, ',' ,realY
    print 'position-real: ', realCoord
    print 'angle-real: ',realAngle
    #print 'gps read: ', gpsX, ',', gpsY   
    
    #MCL STARTS HERE    
    particle_bundle = pf.run(clientID, particle_bundle, actionNow, rVal, mapAll, sensorH_particle, particle_handler)    
    #MCL ENDS HERE
    
    #map plotting
    for ii in range(-1,len(mapCoord)):
        ax1 = fig.add_subplot(111, aspect='equal')
        centerx, centery = mapCoord[ii]
        if mapWallOrientation[ii] == 'hor':
            wallx = centerx - 0.25
            wally = centery - 0.075
            ax1.add_patch(patches.Rectangle((wallx,wally),0.5,0.15,))
        else:
            wallx = centerx - 0.075
            wally = centery - 0.25
            ax1.add_patch(patches.Rectangle((wallx,wally),0.15,0.5,))
    
    #particle plotting
    for pp in particle_bundle:
        robo.particle_draw(fig,pp[0])    
    
    #real robot plotting
    plt.plot(gpsX,gpsY,'ro',realX,realY,'bo')
    plt.axis([-3,3,-3,3])
    plt.show()
    
    i=i+1



#let's stop this    
robo.motor_move(clientID,left_motor,0,right_motor,0)    
print 'Done....'
    