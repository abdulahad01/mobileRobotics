# -*- coding: utf-8 -*-
from math import atan2, sqrt,sin,cos
import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
import numpy as np
from matplotlib.patches import Ellipse


def plot_state(mu, S, M):

    # initialize figure
    ax = plt.gca()
    ax.set_xlim([np.min(M[:, 0]) - 2, np.max(M[:, 0]) + 2])
    ax.set_xlim([np.min(M[:, 1]) - 2, np.max(M[:, 1]) + 2])
    plt.plot(M[:, 0], M[:, 1], '^r')
    plt.title('EKF Localization')

    # visualize result
    plt.plot(mu[0], mu[1], '.b')
    plot_2dcov(mu, S)
    plt.draw()
    plt.pause(0.01)


def plot_2dcov(mu, cov):

    # covariance only in x,y
    d, v = np.linalg.eig(cov[:-1, :-1])

    # ellipse orientation
    a = np.sqrt(d[0])
    b = np.sqrt(d[1])

    # compute ellipse orientation
    if (v[0, 0] == 0):
        theta = np.pi / 2
    else:
        theta = np.arctan2(v[0, 1], v[0, 0])

    # create an ellipse
    ellipse = Ellipse((mu[0], mu[1]),
                      width=a * 2,
                      height=b * 2,
                      angle=np.deg2rad(theta),
                      edgecolor='blue',
                      alpha=0.3)

    ax = plt.gca()

    return ax.add_patch(ellipse)


def wrapToPi(theta):
    while theta < -np.pi:
        theta = theta + 2 * np.pi
    while theta > np.pi:
        theta = theta - 2 * np.pi
    return theta

def inverse_motion_model(u):
    # print(u)
    rot1 = atan2((u[1][1]-u[0][1]), (u[1][0]-u[0][0])) - u[0][2]
    trans = sqrt(((u[1][0]-u[0][0])**2) + ((u[1][1]-u[0][1])**2))
    rot2 = u[1][2] - u[0][2] - rot1
    # print(rot1,trans,rot2)
    return rot1,trans,rot2

def ekf_predict(x_pred,P_pred,u):
    for i in range(u.shape[0] - 1):
        rot1,trans,rot2 = inverse_motion_model(u[i:i+2,:])
        # predicted mean is the value of process model at linearization point
        theta = x_pred[2][0]
        G = np.array([[1, 0, -trans*(sin(theta +rot1))],
                      [0, 1, trans*cos(theta+rot1)],
                      [0, 0, 1]])
        # print(G)
        # V = np.array([[-trans*sin(theta+rot1), cos(theta+rot1), 0],
        #              [trans*cos(theta+rot1), sin(theta+rot1), 0],
        #              [1, 0, 1]])
        x_pred = x_pred + np.array([[trans*cos(theta+rot1), trans*sin(theta+rot1), rot2+rot1]]).T
        # covariance of prediction is GPG^T, where G is the jacobian of process at linearization point
        P_pred = np.dot(np.dot(G,P_pred),G.T)
    return x_pred,P_pred

def ekf_correct(x,P,z):
    # Calculate z_estimate from map readings and pos of robor

    # Calculate y 

    # Calculate S = HPH^T + R

    # Calculate Kalman gain K = PH^TS^-1

    # x_est = x-Ky

    # P_est = (I-KP_est)P_est

    return x_est,P_est
