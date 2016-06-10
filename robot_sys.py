# -*- coding: utf-8 -*-
import vrep, sys, math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from scipy.integrate import quad

def connection_init():
    print 'Python program : V-Rep'
    vrep.simxFinish(-1)
    clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)
    if clientID!=-1:
        print 'Coonect to remote API Server'
    else:
        print 'Connection not successful'
        sys.exit('Could not connect')
    return clientID

def map_generate(clientID):
    wallName = '80cmHighWall50cm'
    wallList = []
    wallPosList = []
    wallOrientationList = []
    for ii in range (0,63):
        wallNum = str(ii)
        wallName2 = "".join((wallName,wallNum))
        errorCode,wall=vrep.simxGetObjectHandle(clientID,wallName2,vrep.simx_opmode_oneshot_wait)
        returnCode, wallPos = vrep.simxGetObjectPosition(clientID,wall,-1,vrep.simx_opmode_oneshot_wait)
        returnCode, eulerAngles = vrep.simxGetObjectOrientation(clientID,wall,-1,vrep.simx_opmode_oneshot_wait)
        wallList.append(wall)
        wallPosList.append((wallPos[0],wallPos[1]))
        if eulerAngles[2] < 0 :
            wallOrientationList.append('hor')
        else: 
            wallOrientationList.append('ver')
        
    return wallPosList,wallOrientationList

def map_checkCollision(clientID,mapCoord,wallSegOri,pointX,pointY):
    nearest_idx = 0
    near_dist = 1000
    syalala = 0
    xMin = 1
    yMin = 1
    xMax = 1
    yMax = 1
    
    #cari indeks segmen tembok terdekat        
    for ii in range(0,len(mapCoord)):
        wallSegX, wallSegY = mapCoord[ii]
        dist = math.sqrt(math.pow(math.fabs(pointX-wallSegX),2) + math.pow(math.fabs(pointY-wallSegY),2))
        if dist<near_dist:
            nearest_idx = ii
            near_dist = dist
    
    #bikin xy min max buat tes collide
    nearX, nearY = mapCoord[nearest_idx]
    if wallSegOri[nearest_idx] == 'hor':
        xMin = nearX-0.25
        yMin = nearY-0.075
        xMax = nearX+0.25
        yMax = nearY+0.075
    elif wallSegOri[nearest_idx] == 'ver':
        xMin = nearX-0.075
        yMin = nearY-0.25
        xMax = nearX+0.075
        yMax = nearY+0.25
    
    #cek titik
    if pointX>xMin and pointX<xMax and pointY>yMin and pointY<yMax:
        return 1
    else:
        return 0


def roboObj_init(clientID,robotname):
    errorCode,robo=vrep.simxGetObjectHandle(clientID,robotname,vrep.simx_opmode_oneshot_wait)
    return robo

def pose(clientID,robo):
    returnCode, pos = vrep.simxGetObjectPosition(clientID,robo,-1,vrep.simx_opmode_streaming)
    returnCode, eulerAngles = vrep.simxGetObjectOrientation(clientID,robo,-1,vrep.simx_opmode_oneshot_wait)
    return (pos , eulerAngles)

def motor_init(clientID):
    #Get motor handle
    errorCode,left_motor=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)
    errorCode,right_motor=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait)    
    
    return (left_motor, right_motor)

def motor_move(clientID,left_motor,left_speed,right_motor,right_speed):
    vrep.simxSetJointTargetVelocity(clientID,left_motor,left_speed,vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(clientID,right_motor,right_speed,vrep.simx_opmode_oneshot_wait)

def sensor_init(clientID):
    # Get ultrasound sensors handle
    errorCode,usensor1=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor1',vrep.simx_opmode_oneshot_wait)    
    errorCode,usensor2=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor2',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor3=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor3',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor4=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor4',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor5=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor6=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor6',vrep.simx_opmode_oneshot_wait)    
    errorCode,usensor7=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor7',vrep.simx_opmode_oneshot_wait)    
    errorCode,usensor8=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor8',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor9=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor9',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor10=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor10',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor11=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor11',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor12=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor12',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor13=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor13',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor14=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor14',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor15=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor15',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor16=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor16',vrep.simx_opmode_oneshot_wait)    
       
    #return [usensor1,usensor2,usensor3,usensor4,usensor5,usensor6,usensor7,usensor8,usensor9,usensor10,usensor11,usensor12,usensor13,usensor14,usensor15,usensor16]
    return [usensor1,usensor2,usensor7,usensor8]
    
def sensor_read(clientID,sensorHandler):
    errorCode,state,point,detectedObjectHandle,detectedSurfaceNormalVector = vrep.simxReadProximitySensor(clientID,sensorHandler,vrep.simx_opmode_oneshot_wait)
    dist = math.sqrt(point[0]*point[0]+point[1]*point[1])
    zmax = 2
    if dist > zmax:
        dist=zmax
    return state, dist

def sensor_readAll(clientID, sensorHandlerBundle):
    readValList=[]
    readStateList=[]
    for h in sensorHandlerBundle:
        state,res = sensor_read(clientID,h)
        readStateList.append(state)
        readValList.append(res)
    return (readStateList,readValList)

def gps_read(clientID):
    #Read GPS Sensor
    errorCode,gpsX=vrep.simxGetFloatSignal(clientID,'gpsX',vrep.simx_opmode_oneshot_wait);
    errorCode,gpsY=vrep.simxGetFloatSignal(clientID,'gpsY',vrep.simx_opmode_oneshot_wait);
    return (gpsX, gpsY)
    

def camera_all():
    # Get camera sensors handle
    errorCode,cam_handle=vrep.simxGetObjectHandle(clientID,'Camera',vrep.simx_opmode_oneshot_wait)           
    errorCode,resolution,image=vrep.simxGetVisionSensorImage(clientID,cam_handle,0,vrep.simx_opmode_oneshot_wait)    
    
def particle_draw(fig, pose):
    position = pose[0]
    angle = pose[1]
    posx, posy, posz = position
    alpha, beta, omega = angle
    r = 0.2
    ax1 = fig.add_subplot(111, aspect='equal')
    #ax1.add_patch(patches.Circle((posx,posy),0.15,color='g'))
    ax1.add_patch(patches.Arrow(posx, posy,r*math.cos(omega),r*math.sin(omega) ,width=0.3,color='g'))
    #plt.plot([posx,posy],[r*math.cos(alpha),r*math.sin(alpha)],'k-')

def particle_init(amount):
    X_tmp = np.random.uniform(-2.5, 2.5, amount)
    Y_tmp = np.random.uniform(-2.5, 2.5, amount)
    Z_tmp = [0.1388]*amount
    ALPHA_tmp = [0]*amount
    BETA_tmp = [0]*amount
    THETA_tmp = np.random.uniform(0.0, math.pi*2.0, amount)
    
    position_tmp = zip(X_tmp, Y_tmp, Z_tmp)
    angle_tmp = zip(ALPHA_tmp, BETA_tmp, THETA_tmp)
    pose_tmp = zip(position_tmp, angle_tmp)
    W = [1.0/amount] * amount# uniform
    X = zip(pose_tmp, W)
    
    return X

def particle_sensor_init(clientID):
    # Get ultrasound sensors handle
    errorCode,usensor1=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor1#0',vrep.simx_opmode_oneshot_wait)    
    errorCode,usensor2=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor2#0',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor3=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor3#0',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor4=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor4#0',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor5=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5#0',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor6=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor6#0',vrep.simx_opmode_oneshot_wait)    
    errorCode,usensor7=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor7#0',vrep.simx_opmode_oneshot_wait)    
    errorCode,usensor8=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor8#0',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor9=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor9#0',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor10=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor10#0',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor11=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor11#0',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor12=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor12#0',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor13=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor13#0',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor14=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor14#0',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor15=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor15#0',vrep.simx_opmode_oneshot_wait)    
    #errorCode,usensor16=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor16#0',vrep.simx_opmode_oneshot_wait)    
       
    #return [usensor1,usensor2,usensor3,usensor4,usensor5,usensor6,usensor7,usensor8,usensor9,usensor10,usensor11,usensor12,usensor13,usensor14,usensor15,usensor16]
    return [usensor1,usensor2,usensor7,usensor8]

def particle_pose_set(clientID,particle_handler,pose):
    position = pose[0]
    angle = pose[1]
    returnCode = vrep.simxSetObjectPosition(clientID,particle_handler,-1,position,vrep.simx_opmode_oneshot)
    returnCode = vrep.simxSetObjectOrientation(clientID,particle_handler,-1,angle,vrep.simx_opmode_oneshot)

def particle_motion_model(clientID,action,pose_past,maps, particle_handler):
	vScale = 0.25
	vR = vScale*action[0]
	vL = vScale*action[1]    
	position = pose_past[0]
	angle = pose_past[1]
	px,py,pz = position
	alpha,beta,omega = angle
	
	#convert from vR and vL to v and w
	#cari jari2 roda & kec. translasi
	wheelHandle=vrep.simxGetObjectHandle('Pioneer_p3dx_rightMotor')
	res,zMin=vrep.simxGetObjectFloatParameter(clientID,wheelHandle,17,vrep.simx_opmode_oneshot_wait)
	res,zMax=vrep.simxGetObjectFloatParameter(clientID,wheelHandle,20,vrep.simx_opmode_oneshot_wait)
	r=(zMax-zMin)/2
	vTrans = ((vR*r) + (vL*r))/2
	
	#cari kec. rotasi
	errorCode,pos1=vrep.simxGetObjectPosition(clientID,left_motor,p3dx,vrep.simx_opmode_oneshot_wait)
	errorCode,pos2=vrep.simxGetObjectPosition(clientID,right_motor,p3dx,vrep.simx_opmode_oneshot_wait)
	wheelsep = math.sqrt((pos1[0]-pos2[0])*(pos1[0]-pos2[0]) + (pos1[1]-pos2[1])*(pos1[1]-pos2[1]))	
	vRot = (vR-vL)/wheelSep    
    
    # Set the (true) action error model
	mu = 0.0
	std = 0.25

    #set alpha & delta t
	a1=0.01
	a2=0.01
	a3=0.01
	a4=0.01
	a5=0.01
	a6=0.01
	dt=1 
	#konstan
    
    #simulating error

    #---------------------------
    #DO YOUR IMPLEMENTATION HERE
    #---------------------------
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
	particle_pose_set(clientID,particle_handler,newPose)
	
	return newPose
 
def integrant(sensed, sensed_sim, deltasq_hit):
    return (math.exp((-1/2)*(math.pow(sensed-sensed_sim,2))/deltasq_hit))/math.sqrt(2*math.pi*deltasq_hit)
   
def particle_sensor_model(clientID, perception, pose, maps, part_sensorH):
	prob_q = 1
	zmax = 2
    #get readings from particle : ray casting ala-ala
	stat, perception_sim = sensor_readAll(clientID,part_sensorH)
    #intrinsic parameters
	z_hit = 1
	delta_hit = 1
	deltasq_hit = delta_hit**2
	z_short = 1
	lambda_short = 1
	z_max = 1
	z_rand = 1
    
	p_hit = 0
	p_short = 0
	p_max = 0
	p_rand = 0
	
	n = 1
	
    #---------------------------
    #DO YOUR IMPLEMENTATION HERE
    #---------------------------
	
	if (perception >= 0) and (perception <= zmax):
		p_hit = quad.quad(lambda x: integrant(perception,perception_sim,deltasq_hit),0,zmax)*integrant(perception,perception_sim,deltasq_hit)
		p_rand = 1/zmax
	else :
		p_hit = 0
		p_rand = 0
	
	if (perception >= 0) and (perception <= perception_sim):
		p_short = (1/(1-math.exp(-lambda_short*perception_sim)))*lambda_short*(math.exp(-lambda_short*perception))
	else :
		p_short = 0
	
	if (perception == zmax):
		p_max = 1
	else :
		p_max = 0
	
	prob_q = z_hit*p_hit + z_short*p_short + z_max*p_max + z_rand*p_rand
	
	return prob_q
    

def particle_sensor_read(fig,position,angle,clientID,mapCoord,wallSegOri):
    posx, posy, posz = position
    alpha, beta, omega = angle
    omega2 = omega-1.5708
    readMax = 9 #30 cm
    read_rightHor = 0
    read_rightDiag = 0
    read_leftHor = 0
    read_leftDiag = 0
    
    lastx1 = 0
    lasty1 = 0    
    lastx2 = 0
    lasty2 = 0    
    lastx3 = 0
    lasty3 = 0    
    lastx4 = 0
    lasty4 = 0    
    
    #real ray casting
    
    #---------------------------
    #DO YOUR IMPLEMENTATION HERE
    #---------------------------     
    
    ax1 = fig.add_subplot(111, aspect='equal')
    ax1.add_patch(patches.Arrow(posx,posy,lastx1,lasty1,width=0.5,color='r'))
    ax1.add_patch(patches.Arrow(posx,posy,lastx2,lasty2,width=0.5,color='r'))
    ax1.add_patch(patches.Arrow(posx,posy,lastx3,lasty3,width=0.5,color='r'))
    ax1.add_patch(patches.Arrow(posx,posy,lastx4,lasty4,width=0.5,color='r'))
        
    
    return [read_rightHor, read_rightDiag, read_leftHor, read_leftDiag]