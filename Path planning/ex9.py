"""
Completed by Abdul Ahad
https://github.com/abdulahad01/mobileRobotics
"""
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


def isValid(v):
    if v > thr_free:
        return True
    return False


def getDistance(y, x, goal):
    goal_x = goal[1]
    goal_y = goal[0]
    dist = math.sqrt((x-goal_x)**2 + (y-goal_y)**2)
    return round(dist)


def getManhattan(y, x, y2, x2):
    return abs(x2-x) + abs(y2-y)

# The greedy path planning


def plan_path_uninformed(x_start, x_goal, grid):
    open = []
    closed = []
    neighbors = dict()
    open.append([getDistance(x_start[0], x_start[1], x_goal),
                x_start[0], x_start[1]])
    motions = [[0, 1], [1, 0], [0, -1], [-1, 0], [1, 1],
               [-1, 1], [1, -1], [-1, -1]]  # right , down, left, up
    neighbors[tuple([x_start[0], x_start[1]])] = 0
    path = []

    while len(open):
        open.sort()
        current = open.pop(0)
        g = current[0]
        y = current[1]
        x = current[2]

        if y == x_goal[0] and x == x_goal[1]:
            print("goal reached")
            path.append(tuple([y, x]))
            while neighbors[tuple([y, x])] != 0:
                path.append(neighbors[tuple([y, x])])
                y2 = neighbors[tuple([y, x])][0]
                x2 = neighbors[tuple([y, x])][1]
                y, x = y2, x2
            break
        else:
            closed.append([y, x])
            for i in range(len(motions)):
                y2 = y + motions[i][0]
                x2 = x + motions[i][1]
                if y2 < grid.shape[0] and y2 >= 0 and x2 < grid.shape[1] and x2 >= 0:
                    if isValid(grid[y2, x2]):
                        if [y2, x2] not in closed:
                            cost_new = g + getDistance(y2, x2, x_goal)
                            in_Open = False
                            for node in open:
                                if [y2, x2] == node[1:]:
                                    in_Open = True
                                    if node[0] > cost_new:
                                        node[0] = cost_new
                                        neighbors[tuple([y2, x2])] = (y, x)
                            if not in_Open:
                                open.append([cost_new, y2, x2])
                                neighbors[tuple([y2, x2])] = (y, x)

    return path[::-1]

# A star search


def plan_path_astar(x_start, x_goal, grid):
    open = []
    closed = []
    neighbors = dict()
    open.append([getDistance(x_start[0], x_start[1], x_goal),
                x_start[0], x_start[1]])
    motions = [[0, 1], [1, 0], [0, -1], [-1, 0], [1, 1],
               [-1, 1], [1, -1], [-1, -1]]  # right , down, left, up
    neighbors[tuple([x_start[0], x_start[1]])] = 0
    path = []

    while len(open):
        open.sort()
        current = open.pop(0)
        g = current[0]
        y = current[1]
        x = current[2]

        if y == x_goal[0] and x == x_goal[1]:
            print("goal reached")
            path.append(tuple([y, x]))
            while neighbors[tuple([y, x])] != 0:
                path.append(neighbors[tuple([y, x])])
                y2 = neighbors[tuple([y, x])][0]
                x2 = neighbors[tuple([y, x])][1]
                y, x = y2, x2
            break
        else:
            closed.append([y, x])
            for i in range(len(motions)):
                y2 = y + motions[i][0]
                x2 = x + motions[i][1]
                if y2 < grid.shape[0] and y2 >= 0 and x2 < grid.shape[1] and x2 >= 0:
                    if isValid(grid[y2, x2]):
                        if [y2, x2] not in closed:
                            cost_new = g + \
                                getDistance(y2, x2, x_goal) + \
                                getManhattan(y, x, y2, x2)
                            in_Open = False
                            for node in open:
                                if [y2, x2] == node[1:]:
                                    in_Open = True
                                    if node[0] > cost_new:
                                        node[0] = cost_new
                                        neighbors[tuple([y2, x2])] = (y, x)
                            if not in_Open:
                                open.append([cost_new, y2, x2])
                                neighbors[tuple([y2, x2])] = (y, x)

    return path[::-1]
