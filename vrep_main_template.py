# -*- coding: utf-8 -*-
"""
Created on Thu Apr 30 01:11:23 2015

@author: anwar.maxsum
"""

#simExtRemoteApiStart(19999)

import vrep
import numpy as np
import matplotlib
import sys
import math

#import kalman_template

print 'Python program : V-Rep'
vrep.simxFinish(-1)
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID!=-1:
    print 'Coonect to remote API Server'
else:
    print 'Connection not successful'
    sys.exit('Could not connect')

#Get motor handle
errorCode,left_motor=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)
errorCode,right_motor=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait)    
errorCode,p3dx=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_oneshot_wait)    

for i in range(0,10) :
	errorCode,usensor=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor8',vrep.simx_opmode_oneshot_wait)           
	errorCode,objpos1 = vrep.simxGetObjectPosition(clientID,p3dx,-1,vrep.simx_opmode_oneshot_wait)
	errorCode,objangle = vrep.simxGetObjectOrientation(clientID,p3dx,-1,vrep.simx_opmode_oneshot_wait)
	
	vR = 2
	vL = 2
	print 'iteration : ', i
	vrep.simxSetJointTargetVelocity(clientID,left_motor,vL,vrep.simx_opmode_oneshot_wait)
	vrep.simxSetJointTargetVelocity(clientID,right_motor,vR,vrep.simx_opmode_oneshot_wait)    

    # baca sensor sonar
    # Get ultrasound sensors handle
	
	errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,usensor,vrep.simx_opmode_oneshot_wait)
	print 'data sensor : ', detectedPoint
	
	errorCode,objpos = vrep.simxGetObjectPosition(clientID,p3dx,-1,vrep.simx_opmode_oneshot_wait)
	x = (objpos1[1]-objpos[1])
	y = (objpos1[0]-objpos[0])
	error,v,w = vrep.simxGetObjectVelocity(clientID,p3dx,vrep.simx_opmode_oneshot_wait)
	print 'pos : ',objpos
	print 'deltapos : ',x,', ',y
	print 'transV : ',math.sqrt(v[0]*v[0] + v[1]*v[1])
	#print 'w: ',math.sqrt(w[0]*w[0]+w[1]*w[1])
	#print 'w: ',w
	
	wheelHandle=left_motor
	res,zMin=vrep.simxGetObjectFloatParameter(clientID,wheelHandle,17,vrep.simx_opmode_oneshot_wait)
	res,zMax=vrep.simxGetObjectFloatParameter(clientID,wheelHandle,20,vrep.simx_opmode_oneshot_wait)
	r=(zMax-zMin)/2
	
	vTrans = ((vR*r) + (vL)*r)/2
	print 'vtrans : ', vTrans
	#print 'w: ',vTrans/r
	
	errorCode,pos1=vrep.simxGetObjectPosition(clientID,left_motor,p3dx,vrep.simx_opmode_oneshot_wait)
	errorCode,pos2=vrep.simxGetObjectPosition(clientID,right_motor,p3dx,vrep.simx_opmode_oneshot_wait)
	print pos1
	print pos2
	
	wheelsep = math.sqrt((pos1[0]-pos2[0])*(pos1[0]-pos2[0]) + (pos1[1]-pos2[1])*(pos1[1]-pos2[1]))
	vRot = (vR-vL)/wheelsep
	print 'vrot : ',vRot
	print ''
	
	px,py,pz = objpos1
	alpha,beta,omega = objangle
	
	mu = 0.0
	a1=0.01
	a2=0.01
	a3=0.01
	a4=0.01
	a5=0.01
	a6=0.01
	dt=1 
	
	std_v = a1*abs(vTrans) + a2*abs(vRot)
	std_w = a3*abs(vTrans) + a4*abs(vRot)
	std_rot = a5*abs(vTrans) + a6*abs(vRot)
	
	err_v = np.random.normal(mu, std_v)
	err_w = np.random.normal(mu, std_w)
	gamma = np.random.normal(mu, std_rot)
	
	vTransa = vTrans + err_v
	vRota = vRot + err_w 

    #find new pose

    #---------------------------
    #DO YOUR IMPLEMENTATION HERE
    #---------------------------
	
	xa = px - (vTransa/vRota)*math.sin(omega) + (vTransa/vRota)*math.sin(omega + vRota*dt)
	ya = py + (vTransa/vRota)*math.cos(omega) - (vTransa/vRota)*math.cos(omega + vRota*dt)
	omegaa = omega + vRota*dt + gamma*dt
	
	newPosition = (xa, ya, pz)
	newAngle = (alpha, beta, omegaa) 
	newPose = [newPosition, newAngle]
	
	print 'newPose : ',newPose
	
    
vrep.simxSetJointTargetVelocity(clientID,left_motor,0,vrep.simx_opmode_oneshot_wait)
vrep.simxSetJointTargetVelocity(clientID,right_motor,0,vrep.simx_opmode_oneshot_wait)    

print 'Done....'
    