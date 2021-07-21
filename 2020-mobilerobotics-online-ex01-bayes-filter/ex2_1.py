#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

# Author : Adbdul Ahad
# Repo : https://github.com/abdulahad01/mobile_sensing_robotics/

def plot_belief(belief):
    
    plt.figure()
    
    ax = plt.subplot(2,1,1)
    ax.matshow(belief.reshape(1, belief.shape[0]))
    ax.set_xticks(np.arange(0, belief.shape[0],1))
    ax.xaxis.set_ticks_position("bottom")
    ax.set_yticks([])
    ax.title.set_text("Grid")
    
    ax = plt.subplot(2, 1, 2)
    ax.bar(np.arange(0, belief.shape[0]), belief)
    ax.set_xticks(np.arange(0, belief.shape[0], 1))
    ax.set_ylim([0, 1.05])
    ax.title.set_text("Histogram")


        
def motion_model(distr,command) :
    
    pCorrect = 0.7
    pUndershoot = 0.1
    pStay = 0.2
    new_distr = np.zeros(len(distr))
    
    # if forward command, control input ==1 else -1
    if command == 'F':
        U = 1
    else :
        U = -1
        # the probability that the robot is in its current position is the total probability of it moving correctly from
        # the previous cell and the probablity it has undershot or stays in its prev position
        # the world is not cyclic so we account for the edge cases i.e i=0 and i =14
    for i in range (len(distr)):
        if i >0 and i <14:
            new_distr[i] = distr[(i-U)%len(distr)]*pCorrect + distr[(i+U)%len(distr)]*pUndershoot + distr[(i)%len(distr)]*pStay
        elif i == 0:
            new_distr[i] = distr[(i+U)%len(distr)]*pUndershoot + distr[(i)%len(distr)]*pStay
        else :
            new_distr[i] = distr[(i-U)%len(distr)]*pCorrect + distr[(i)%len(distr)]*pStay
    return new_distr
                

# def sensor_model(observation, belief, world):
#     p_white = 0.7
#     p_black = 0.9
#     belief_out = np.copy(belief)

#     if observation == 0:

#         for i, val in enumerate(world):
#             # Assuming black observation by sensor and world is black too
#             if val == 0:
#                 belief_out[i] = p_black * belief[i]

#             # Accounting for sensor noise in the sensor that the world is white and sensor gives black
#             else:
#                 belief_out[i] = (1 - p_white) * belief[i]

#     else:
#         for i, val in enumerate(world):
#             # Assuming white observation by sensor and world is white too
#             if val == 1:
#                 belief_out[i] = p_white * belief[i]

#             # Accounting for sensor noise in the sensor that the world is black and sensor gives white
#             else:
#                 belief_out[i] = (1 - p_black) * belief[i]

#     # Normalizing it to get the PDF so sum of all elements must be one
#     return belief_out / sum(belief_out)

def sensor_model(observation, belief, world):
    pWhite  = 0.7   #propability that the sensor correctly observes the cell is white  p(white|cell is white)
    pBlack  = 0.9   #propability that the sensor correctly observes the cell is black  p(black|cell is black)
    belief_new = np.copy(belief)
    for i in range (len(world)):
        if observation == 1:
            if world[i] == 1:
              # if the sensor correctly observes the cell
                    belief_new[i] = belief[i] * pWhite
            else :
              # Incorrect observation due to sensor noise  
                    belief_new[i] = belief[i]  * (1-pBlack)            
        else:
            if world[i] == 0:
                    belief_new[i] = belief[i]  * pBlack
            else :
                    belief_new[i] = belief[i]  * (1-pWhite)
    return (belief_new/sum(belief_new))

def recursive_bayes_filter(actions, observations, belief, world):
    sensor_update = sensor_model(observations[0], belief, world)
    for i in range(len(actions)) :
        motion_update = motion_model(sensor_update,actions[i])
        sensor_update = sensor_model(observations[i+1], motion_update, world)
    return sensor_update

