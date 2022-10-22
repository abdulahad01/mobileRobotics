"""
Completed by Abdul Ahad
https://github.com/abdulahad01/mobileRobotics
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

# Author : Abdul Ahad
# Repo : https://github.com/abdulahad01/mobile_sensing_robotics/


def plot_belief(belief):

    plt.figure()

    ax = plt.subplot(2, 1, 1)
    ax.matshow(belief.reshape(1, belief.shape[0]))
    ax.set_xticks(np.arange(0, belief.shape[0], 1))
    ax.xaxis.set_ticks_position("bottom")
    ax.set_yticks([])
    ax.title.set_text("Grid")

    ax = plt.subplot(2, 1, 2)
    ax.bar(np.arange(0, belief.shape[0]), belief)
    ax.set_xticks(np.arange(0, belief.shape[0], 1))
    ax.set_ylim([0, 1.05])
    ax.title.set_text("Histogram")


def motion_model(distr, command):

    pCorrect = 0.7
    pOpp = 0.1
    pStay = 0.2

    # if forward command, control input ==1 else -1
    if command == 'F':
        U = 1
        # the probability that the robot is in its current position is the total probability of it moving correctly from
        # the previous cell and the probablity it has undershot or stays in its prev position
        # the world is not cyclic so we account for the edge cases i.e i=0 and i =14
        for i in range(len(distr)):
            if i > 0 and i < 14:
                distr[i] = distr[(i+U) % len(distr)]*pCorrect + \
                    distr[(i-U) % len(distr)]*pOpp + distr[i]*pStay
            elif i == 14:
                distr[i] = distr[(i-U) % len(distr)]*pOpp + distr[i]*pStay
            else:
                distr[i] = distr[(i+U) % len(distr)]*pCorrect + distr[i]*pStay
    else:
        U = -1
        for i in range(len(distr)):
            if i > 0 and i < 14:
                distr[i] = distr[(i+U) % len(distr)]*pCorrect + \
                    distr[(i-U) % len(distr)]*pOpp + distr[i]*pStay
            elif i == 1:
                distr[i] = distr[(i-U) % len(distr)]*pOpp + distr[i]*pStay
            else:
                distr[i] = distr[(i+U) % len(distr)]*pCorrect + distr[i]*pStay

    return distr/sum(distr)


def sensor_model(observation, belief, world):
    # propability that the sensor correctly observes the cell is white  p(white|cell is white)
    pWhite = 0.7
    # propability that the sensor correctly observes the cell is black  p(black|cell is black)
    pBlack = 0.9
    belief_new = np.copy(belief)
    for i in range(len(world)):
        if observation == 1:
            if world[i] == 1:
              # if the sensor correctly observes the cell
                belief_new[i] = belief[i] * pWhite
            else:
              # Incorrect observation due to sensor noise
                belief_new[i] = belief[i] * (1-pBlack)
        else:
            if world[i] == 0:
                belief_new[i] = belief[i] * pBlack
            else:
                belief_new[i] = belief[i] * (1-pWhite)
    return (belief_new/sum(belief_new))


def recursive_bayes_filter(actions, observations, belief, world):
    sensor_update = sensor_model(observations[0], belief, world)
    for i in range(len(actions)):
        motion_update = motion_model(sensor_update, actions[i])
        sensor_update = sensor_model(observations[i+1], motion_update, world)
    return sensor_update
