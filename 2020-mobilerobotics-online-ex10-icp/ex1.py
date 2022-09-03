# github.com/abdulahad01
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

# icp_known_corresp: performs icp given that the input datasets
# are aligned so that Line1(:, QInd(k)) corresponds to Line2(:, PInd(k))
def icp_known_corresp(Line1, Line2, QInd, PInd):
    Q = Line1[:, QInd]
    P = Line2[:, PInd]

    MuQ = compute_mean(Q)
    MuP = compute_mean(P)
    # print("The means are : {} \n and \n {}".format(MuQ,MuP))
    
    W = compute_W(Q, P, MuQ, MuP)
    # print(W.shape)

    [R, t] = compute_R_t(W, MuQ, MuP)
    
    # Compute the new positions of the points after
    # applying found rotation and translation to them
    NewLine = R@P + t
    E = compute_error(Q, NewLine)
    
    return NewLine, E

# compute_W: compute matrix W to use in SVD
def compute_W(Q, P, MuQ, MuP):
    # add code here and remove pass
    # Compute P_n and Q_n by subtracting with centre of mass
    P_n =  P-MuP
    Q_n =  Q-MuQ

    # W matrix = Q_n * P_n^T
    W = Q_n @ P_n.T
    return W
    
# compute_R_t: compute rotation matrix and translation vector
# based on the SVD as presented in the lecture
def compute_R_t(W, MuQ, MuP):
    # add code here and remove pass
    U,D,V_T = np.linalg.svd(W, full_matrices = False)
    # print(U.shape, D.shape, V_T.shape)
    R = np.dot(U,V_T)
    t = MuQ -np.dot(R,MuP)
    return R,t


# compute_mean: compute mean value for a [M x N] matrix
def compute_mean(M):
    # add code here and remove pass
    # compute mean along x values and y values returns [Mx1] vector
    MuM = np.mean(M,axis =1, keepdims = True)
    return MuM



# compute_error: compute the icp error
def compute_error(Q, OptimizedPoints):
    # add code here and remove pass
    E =  np.array([[(Q[i][j]-OptimizedPoints[i][j])**2 for j in range(Q.shape[1])] for i in range(Q.shape[0])],np.float32)
    return np.sum(E)


# simply show the two lines
def show_figure(Line1, Line2):
    plt.figure()
    plt.scatter(Line1[0], Line1[1], marker='o', s=2, label='Line 1')
    plt.scatter(Line2[0], Line2[1], s=1, label='Line 2')
    
    plt.xlim([-8, 8])
    plt.ylim([-8, 8])
    plt.legend()  
    
    plt.show()
    

# initialize figure
def init_figure():
    fig = plt.gcf()
    fig.show()
    fig.canvas.draw()
    
    line1_fig = plt.scatter([], [], marker='o', s=2, label='Line 1')
    line2_fig = plt.scatter([], [], marker='o', s=1, label='Line 2')
    # plt.title(title)
    plt.xlim([-8, 8])
    plt.ylim([-8, 8])
    plt.legend()
    
    return fig, line1_fig, line2_fig


# update_figure: show the current state of the lines
def update_figure(fig, line1_fig, line2_fig, Line1, Line2, hold=False):
    line1_fig.set_offsets(Line1.T)
    line2_fig.set_offsets(Line2.T)
    if hold:
        plt.show()
    else:
        fig.canvas.flush_events()
        fig.canvas.draw()
        plt.pause(0.5)
