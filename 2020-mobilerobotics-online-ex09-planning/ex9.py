#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
import math

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

def getDistance(y,x,goal):
    goal_x = goal[1]
    goal_y = goal[0]
    dist = math.sqrt((x-goal_x)**2 + (y-goal_y)**2)
    return round(dist)

def plan_path_uninformed(x_start, x_goal, M):
    current_node = x_start
    neighbor = dict()
    motions =[[0,1],[1,0],[0,-1],[-1,0]] # right , down, left, up
    cost = np.array([[0 for i in range(M.shape[1])]for j in range(M.shape[0])])
    cost[x_start[0],x_start[1]] = 0
    # g_cost = 1
    open =[]
    neighbor[tuple([x_start[0],x_start[1]])] = 0
    open.append([0,x_start[0],x_start[1]])
    closed =[]
    # print(cost,"\n",open)
    path=[]
    while True:
            open.sort()
            # print(open)
            if(len(open) == 0):
                print("no path exists")
                break
            current_node = [open[0][1],open[0][2]]
            if current_node[0] == x_goal[0] and current_node[1] == x_goal[1]:
                path = [(x_goal[0],x_goal[1])]
                x = current_node[1]
                y = current_node[0]
                # print(neighbor)
                while neighbor[tuple([y,x])] !=0:
                    y2 = neighbor[tuple([y,x])][0]
                    x2 = neighbor[tuple([y,x])][1]
                    path.append([y2,x2])
                    y = y2
                    x = x2
                break
                
            else :
                x = current_node[1]
                y = current_node[0]
                try:
                    open.remove([cost[y,x], y,x])
                except:
                    print(open[:5])
                    print("list to remove \n",[cost[y,x], y,x])
                    break
                closed.append(tuple([y,x]))
                for i in range (len(motions)):
                    x2 = x + motions[i][1] 
                    y2 = y + motions[i][0]
                    # print(i,[cost[current_node[0],current_node[1]], current_node[0],current_node[1]])
                    
                    if x2 >=0 and x2 < M.shape[1] and y2 >=0 and y2 < M.shape[0] \
                        and is_valid(M[y2][x2]):
                        cost_new = getDistance(y2,x2,x_goal)
                        
                        if  cost_new < cost[y2][x2] or (tuple([y2,x2]) not in closed) :
                            # print(cost_new, y2,x2)
                            cost[y2][x2] = cost_new
                            open.append([float(cost_new),y2,x2])
                            neighbor[tuple([y2,x2])] = (tuple([y,x]))
    return path[::-1]

    
def plan_path_astar(x_start, x_goal, M):
    current_node = x_start
    neighbor = dict()
    motions =[[0,1],[1,0],[0,-1],[-1,0]] # right , down, left, up
    cost = np.array([[0 for i in range(M.shape[1])]for j in range(M.shape[0])])
    cost[x_start[0],x_start[1]] = 0
    g_cost = 1
    open =[]
    neighbor[tuple([x_start[0],x_start[1]])] = 0
    open.append([0,x_start[0],x_start[1]])
    closed =[]
    # print(cost,"\n",open)
    path=[]
    while True:
            open.sort()
            # print(open)
            if(len(open) == 0):
                print("no path exists")
                break
            current_node = [open[0][1],open[0][2]]
            if current_node[0] == x_goal[0] and current_node[1] == x_goal[1]:
                path = [(x_goal[0],x_goal[1])]
                x = current_node[1]
                y = current_node[0]
                # print(neighbor)
                while neighbor[tuple([y,x])] !=0:
                    y2 = neighbor[tuple([y,x])][0]
                    x2 = neighbor[tuple([y,x])][1]
                    path.append([y2,x2])
                    y = y2
                    x = x2
                break
                
            else :
                x = current_node[1]
                y = current_node[0]
                try:
                    open.remove([cost[y,x], y,x])
                except:
                    print(open[:5])
                    print("list to remove \n",[cost[y,x], y,x])
                    break
                closed.append(tuple([y,x]))
                for i in range (len(motions)):
                    x2 = x + motions[i][1] 
                    y2 = y + motions[i][0]
                    # print(i,[cost[current_node[0],current_node[1]], current_node[0],current_node[1]])
                    
                    if x2 >=0 and x2 < M.shape[1] and y2 >=0 and y2 < M.shape[0] \
                        and is_valid(M[y2][x2]):
                        cost_new = getDistance(y2,x2,x_goal) + cost[y,x]
                        
                        if  cost_new < cost[y2][x2] or (tuple([y2,x2]) not in closed) :
                            # print(cost_new, y2,x2)
                            cost[y2][x2] = cost_new
                            open.append([float(cost_new),y2,x2])
                            neighbor[tuple([y2,x2])] = (tuple([y,x]))
    return path[::-1]
  
