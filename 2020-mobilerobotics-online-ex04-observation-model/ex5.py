# github.com/abdulahad01
from math import *
import matplotlib.pyplot as plt
def landmark_observation_model(z,x,m,sigma):
    z_exp = sqrt((x[0]-m[0])**2 + (x[1]-m[1])**2)
    prob = exp((-0.5*(z-z_exp)**2)/sigma**2)*1/sqrt(2*pi*sigma**2)
    return prob

def plot_map(gridmap):
    plt.figure()
    plt.imshow(gridmap, cmap='Greys')