# -*- coding: utf-8 -*-
"""
Created on Tue May 31 16:46:29 2016

@author: vektor,novian
"""

# @obj: implement the standard MCL alg.; table 8.2 on the book Prob. Robotics by S. Thrun
# @author: vektor dewanto

import numpy as np
import math
import robot_sys as robo

def normalize_weight(X):
    # Normalize all weights, so that they sum up to one
    total_w = sum([xw[1] for xw in X])
    
    X = [(xw[0], xw[1]/total_w) for xw in X]
    
    return X
    
def resample(X_bar):
    ''' 
    draw i with probability proportional to w_t^i
    '''
    X_bar = normalize_weight(X_bar)
    X = []

    while len(X) < len(X_bar):
        candidate_idx = np.random.random_integers(low=0, high= len(X_bar)-1) 
        candidate_w = X_bar[candidate_idx][1]

        sampled = np.random.binomial(n=1, p=candidate_w)# a Bernoulli dist.
        
        if sampled==1:
            X.append(X_bar[candidate_idx])
            
    return X
    
def run(clientID, X_past, u, z, m, part_sensorH, part_botH):
	''' 
	\param X: is a list of tuples (x, w)
	\param u: the control/action
	\param z: the observation
	\param m: the given map
	'''
    #---------------------------
    #DO YOUR IMPLEMENTATION HERE
    #---------------------------
	arr_x_bar = []
	
	for i in range(len(X_past)):
		x_new = robo.particle_motion_model(clientID,u,X_past[i][0],m,part_botH)
		z_new = robo.particle_sensor_model(clientID,z,x_new,m,part_sensorH)
		arr_x_bar.append((x_new, z_new))

	total_w = sum([ x_bar[1] for x_bar in arr_x_bar ])

	arr_x = []
	if total_w != 0:
		arr_x = resample(arr_x_bar)
	else:
		arr_x = arr_x_bar
	return arr_x


    
def run_kld(X_past, u, z, m, eps, delta):
    #recommended eps = 0.05
    ''' 
    \param X: is a list of tuples (x, w)
    \param u: the control/action
    \param z: the observation
    \param m: the given map
    \param eps: error
    \param delta: error
    '''

    #---------------------------
    #DO YOUR IMPLEMENTATION HERE
    #---------------------------

    X_bar = []
    X = []
    n_past_particle = len(X_past)
    
    return X_past

