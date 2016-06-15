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
import robot_sys as rsys
from scipy.integrate import quad

#import kalman_template

print 'Python program : V-Rep'
vrep.simxFinish(-1)
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)
#mapCoord , mapWallOrientation = rsys.map_generate(clientID)
#mapAll = [mapCoord, mapWallOrientation]
sensorH_particle = rsys.particle_sensor_init(clientID)

if clientID!=-1:
    print 'Coonect to remote API Server'
else:
    print 'Connection not successful'
    sys.exit('Could not connect')

#Get motor handle
errorCode,left_motor=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)
errorCode,right_motor=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait)    
errorCode,p3dx=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_oneshot_wait)    

#initialize map
mapCoord , mapWallOrientation = rsys.map_generate(clientID)
mapAll = [mapCoord, mapWallOrientation]
arr_err = []
sensorH = rsys.sensor_init(clientID)
rStat, rVal = rsys.sensor_readAll(clientID,sensorH)
sensorH_particle = rsys.particle_sensor_init(clientID)
	

for i in range(0,1) :
	#errorCode,usensor=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor8',vrep.simx_opmode_oneshot_wait)           
	#errorCode,objpos1 = vrep.simxGetObjectPosition(clientID,p3dx,-1,vrep.simx_opmode_oneshot_wait)
	#errorCode,objangle = vrep.simxGetObjectOrientation(clientID,p3dx,-1,vrep.simx_opmode_oneshot_wait)
	
	#vR = 2*0.25
	#vL = 2*0.25
	print 'iteration : ', i
	#vrep.simxSetJointTargetVelocity(clientID,left_motor,1,vrep.simx_opmode_oneshot_wait)
	#vrep.simxSetJointTargetVelocity(clientID,right_motor,2,vrep.simx_opmode_oneshot_wait)    

    # baca sensor sonar
    # Get ultrasound sensors handle
	
	#errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,usensor,vrep.simx_opmode_oneshot_wait)
	#print 'data sensor : ', detectedPoint
	
	#errorCode,objpos = vrep.simxGetObjectPosition(clientID,p3dx,-1,vrep.simx_opmode_oneshot_wait)
	#x = (objpos1[1]-objpos[1])
	#y = (objpos1[0]-objpos[0])
	#error,v,w = vrep.simxGetObjectVelocity(clientID,p3dx,vrep.simx_opmode_oneshot_wait)
	#print 'pos : ',objpos
	#print 'deltapos : ',x,', ',y
	#print 'transV : ',math.sqrt(v[0]*v[0] + v[1]*v[1])
	#print 'w: ',math.sqrt(w[0]*w[0]+w[1]*w[1])
	#print 'w: ',w
	
	#wheelHandle=left_motor
	#res,zMin=vrep.simxGetObjectFloatParameter(clientID,wheelHandle,17,vrep.simx_opmode_oneshot_wait)
	#res,zMax=vrep.simxGetObjectFloatParameter(clientID,wheelHandle,20,vrep.simx_opmode_oneshot_wait)
	#r=(zMax-zMin)/2
	
	#vTrans = ((vR) + (vL))/2
	#print 'vtrans : ', vTrans
	#print 'w: ',vTrans/r
	
	#errorCode,pos1=vrep.simxGetObjectPosition(clientID,left_motor,p3dx,vrep.simx_opmode_oneshot_wait)
	#errorCode,pos2=vrep.simxGetObjectPosition(clientID,right_motor,p3dx,vrep.simx_opmode_oneshot_wait)
	#print pos1
	#print pos2
	
	#wheelsep = math.sqrt((pos1[0]-pos2[0])*(pos1[0]-pos2[0]) + (pos1[1]-pos2[1])*(pos1[1]-pos2[1]))
	#vRot = (vR-vL)/wheelsep
	#print 'vrot : ',vRot
	#print ''
	
	#[px,py,pz],[alpha,beta,omega] = rsys.pose(clientID,p3dx)
	
	#mu = 0.0
	#a1=0.0001
	#a2=0.0001
	#a3=0.01
	#a4=0.01
	#a5=0.01
	#a6=0.01
	#dt=1 
	
	#std_v = a1*abs(vTrans) + a2*abs(vRot)
	#std_w = a3*abs(vTrans) + a4*abs(vRot)
	#std_rot = a5*abs(vTrans) + a6*abs(vRot)
	
	#err_v = np.random.normal(mu, std_v)
	#err_w = np.random.normal(mu, std_w)
	#gamma = np.random.normal(mu, std_rot)
	
	#vTransa = vTrans + err_v
	#vRota = vRot + err_w 

    #find new pose

    #---------------------------
    #DO YOUR IMPLEMENTATION HERE
    #---------------------------
	
	#xa = px - (vTransa/vRota)*math.sin(omega) + (vTransa/vRota)*math.sin(omega + vRota*dt)
	#ya = py + (vTransa/vRota)*math.cos(omega) - (vTransa/vRota)*math.cos(omega + vRota*dt)
	#omegaa = omega + vRota*dt + gamma*dt
	
	#newPosition = (xa, ya, pz)
	#newAngle = (alpha, beta, omegaa) 
	#newPose = [newPosition, newAngle]
	
	#print 'befPos : ',[px,py,pz],[alpha,beta,omega]
	#print 'newPose : ',newPose
	#actCood,actAngle = rsys.pose(clientID,p3dx)
	#actPose = [objpos1,objangle]
	#print 'actPose : ',actPose
	
	#sensoHandleBundle = rsys.sensor_init(clientID)
	#print sensoHandleBundle
	#stat, perception = rsys.sensor_readAll(clientID,sensoHandleBundle)
	#print stat
	#print perception
	
	#print quad(lambda x : x,0,2)[0]
	#prob_q = rsys.particle_sensor_model(clientID,perception,newPose,1,sensorH_particle)
	#print prob_q
	
	action = [2,2]
	rsys.motor_move(clientID,left_motor,2,right_motor,2)
	pose_past = rsys.pose(clientID,p3dx)
	maps = mapAll
	particle_handler = 1
	
	#print 'pose_past : ',pose_past
	
	#newPose = rsys.particle_motion_model(clientID,action,pose_past,maps, particle_handler)
	
	actCood,actAngle = rsys.pose(clientID,p3dx)
	actPose = [actCood,actAngle]
	#print 'actPose : ',actPose
	#print 'err : ',[actPose[0][0]-newPose[0][0],actPose[0][1]-newPose[0][1],actPose[0][2]-newPose[0][2]]
	
	#arr_err.append([actPose[0][0]-newPose[0][0],actPose[0][1]-newPose[0][1],actPose[1][2]-newPose[1][2]])
	
	
	prob_q = rsys.particle_sensor_model(clientID,rVal,actPose,maps,sensorH_particle)
	print prob_q
    
vrep.simxSetJointTargetVelocity(clientID,left_motor,0,vrep.simx_opmode_oneshot_wait)
vrep.simxSetJointTargetVelocity(clientID,right_motor,0,vrep.simx_opmode_oneshot_wait)    

print 'Done....'
#sum_err = [0,0,0]
#len_err = len(arr_err)-1
#for i in range(1,11) :
#	print arr_err[i]
#	sum_err= [sum_err[0]+arr_err[i][0],sum_err[1]+arr_err[i][1],sum_err[2]+arr_err[i][2]]

#avg_err = [sum_err[0]/len_err,sum_err[1]/len_err,sum_err[2]/len_err]
#err = math.sqrt(avg_err[0]**2 + avg_err[1]**2)
#print err,', ',avg_err[2]