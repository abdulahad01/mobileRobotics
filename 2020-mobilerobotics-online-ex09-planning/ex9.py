#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np

thr_free = 0.9

def plot_path(path, x_start, x_goal, M):
    plt.matshow(M, cmap="gray")
    if path.shape[0] > 2:
        plt.plot(path[:, 1], path[:, 0], 'b')
    plt.plot(x_start[1], x_start[0], 'or')
    plt.plot(x_goal[1], x_goal[0], 'xg')
    plt.show()


def is_valid(v):
    if v > thr_free:
        return True
    return False

def plan_path_uninformed(x_start, x_goal, M):
    current_node = x_start
    path = dict()
    motions =[[0,1],[1,0],[0,-1],[-1,0]] # right , down, left, up
    cost = np.array([[999 for i in range(M.shape[1])]for j in range(M.shape[0])])
    g_cost = 1
    open =[]
    open.append([0,x_start[0],x_start[1]])

    while True:
            open.sort()
            current_node = [open[0][1],open[0][2]]
            if current_node[0] == x_goal[0] and current_node[1] == x_goal[1]:
                return path
                break
            else :

                for i in range (len(motions)):
                    y2 = current_node[1] + motions[i][1] 
                    x2 = current_node[0] + motions[i][0]
                    if x2 >=0 and x2 < M.shape[1] and y2 >=0 and y2 < M.shape[0] \
                        and M[y2][x2] ==1:
                        cost_new = cost[y2][x2] + g_cost
                        if  cost_new < cost[y2][x2] :
                            cost[y2][x2] = cost_new
                            open.append([cost_new,y2,x2])
                            path[tuple(y2,x2)].append(current_node)




    
# def plan_path_astar(x_start, x_goal, M):
	# add code here
  
