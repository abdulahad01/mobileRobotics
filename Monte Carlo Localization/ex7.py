# -*- coding: utf-8 -*-
"""
Completed by Abdul Ahad
https://github.com/abdulahad01/mobileRobotics
"""
from math import sqrt, cos, sin
import gif
import matplotlib.pyplot as plt
import numpy as np


def world2map(pose, gridmap, map_res):
    max_y = np.size(gridmap, 0) - 1
    new_pose = np.zeros_like(pose)
    new_pose[0] = np.round(pose[0] / map_res)
    new_pose[1] = max_y - np.round(pose[1] / map_res)
    return new_pose.astype(int)


def v2t(pose):
    c = np.cos(pose[2])
    s = np.sin(pose[2])
    tr = np.array([[c, -s, pose[0]], [s, c, pose[1]], [0, 0, 1]])
    return tr


def t2v(tr):
    x = tr[0, 2]
    y = tr[1, 2]
    th = np.arctan2(tr[1, 0], tr[0, 0])
    v = np.array([x, y, th])
    return v


def ranges2points(ranges, angles):
    # rays within range
    max_range = 80
    idx = (ranges < max_range) & (ranges > 0)
    # 2D points
    points = np.array([
        np.multiply(ranges[idx], np.cos(angles[idx])),
        np.multiply(ranges[idx], np.sin(angles[idx]))
    ])
    # homogeneous points
    points_hom = np.append(points, np.ones((1, np.size(points, 1))), axis=0)
    return points_hom


def ranges2cells(r_ranges, r_angles, w_pose, gridmap, map_res):
    # ranges to points
    r_points = ranges2points(r_ranges, r_angles)
    w_P = v2t(w_pose)
    w_points = np.matmul(w_P, r_points)
    # world to map
    m_points = world2map(w_points, gridmap, map_res)
    m_points = m_points[0:2, :]
    return m_points


def poses2cells(w_pose, gridmap, map_res):
    # covert to map frame
    m_pose = world2map(w_pose, gridmap, map_res)
    return m_pose


def init_uniform(num_particles, img_map, map_res):
    particles = np.zeros((num_particles, 4))
    particles[:, 0] = np.random.rand(num_particles) * np.size(img_map,
                                                              1) * map_res
    particles[:, 1] = np.random.rand(num_particles) * np.size(img_map,
                                                              0) * map_res
    particles[:, 2] = np.random.rand(num_particles) * 2 * np.pi
    particles[:, 3] = 1.0
    return particles


def plot_particles(particles, img_map, map_res):
    plt.matshow(img_map, cmap="gray")
    max_y = np.size(img_map, 0) - 1
    xs = np.copy(particles[:, 0]) / map_res
    ys = max_y - np.copy(particles[:, 1]) / map_res
    plt.plot(xs, ys, '.b')
    plt.xlim(0, np.size(img_map, 1))
    plt.ylim(0, np.size(img_map, 0))
    plt.show()


def wrapToPi(theta):
    while theta < -np.pi:
        theta = theta + 2 * np.pi
    while theta > np.pi:
        theta = theta - 2 * np.pi
    return theta


def sample(b):
    # sample from a triangular distribution
    return sqrt(6)*(np.random.uniform(-b, b)+np.random.uniform(-b, b))/2


def sample_motion_model_odometry(x0, u_t, alpha):
    # Odometry motion model for sampling
    rot1, trans, rot2 = u_t[0], u_t[1], u_t[2]
    rot1_hat = rot1 + sample(alpha[0]*abs(rot1) + alpha[1]*trans)
    trans_hat = trans + \
        sample(alpha[2]*trans + alpha[3] * (abs(rot1)+abs(rot2)))
    rot2_hat = rot2 + sample(alpha[0]*abs(rot2) + alpha[1]*trans)

    x = x0[0] + trans_hat*cos(x0[2]+rot1_hat)
    y = x0[1] + trans_hat*sin(x0[2]+rot1_hat)
    theta = wrapToPi(x0[2] + rot1_hat + rot2_hat)
    return np.array([x, y, theta, 1.0])


def compute_weights(x, scan, map, likelihood, map_res):
    weight = 1
    m = ranges2cells(scan[1, :].reshape(37, 1),
                     scan[0, :].reshape(37, 1), x, map, map_res)
    for i in range(m.shape[1]):
        if (0 < m[0, i] < likelihood.shape[1]) and (0 < m[1, i] < likelihood.shape[0]):
            weight = weight * likelihood[m[1, i], m[0, i]]
        else:
            weight = 1e-300
    return weight


def resample(particles, weights):
    X = np.zeros_like(particles)
    J = particles.shape[0]
    c = weights[0]
    i = 0
    r = np.random.uniform(0, 1/J)
    for j in range(1, J+1):
        U = r + ((j-1)/J)
        while U > c:
            i = ((i+1) % (J))
            c = c + weights[i]

        X[j-1] = particles[i]
    return X


@gif.frame
def get_frame(particles, img_map, map_res):
    plt.matshow(img_map, cmap="gray")
    max_y = np.size(img_map, 0) - 1
    xs = np.copy(particles[:, 0]) / map_res
    ys = max_y - np.copy(particles[:, 1]) / map_res
    plt.plot(xs, ys, '.b')
    plt.xlim(0, np.size(img_map, 1))
    plt.ylim(0, np.size(img_map, 0))


def mc_localization(particles, data, map_res):
    likelihood_map = data['likelihood_map']
    gridmap = data['img_map']
    frames = []
    num_particles = particles.shape[0]
    alpha = np.array([0.1, 0.1, 0.1, 0.1])

    for k in range(0, len(data['odom'])):
        if (k % 100 == 0):
            print("Particles after", k, "steps")
            plot_particles(particles, gridmap, map_res)

        weights = np.zeros((num_particles, 1))
        u_t = data['odom'][k]
        z = data['z'][k]

        for i in range(num_particles):
            # sampling
            particles[i] = sample_motion_model_odometry(
                particles[i], u_t, alpha)
            # weighing
            weights[i] = compute_weights(
                particles[i], z, gridmap, likelihood_map, map_res)
        # normalization
        weights = weights/sum(weights)
        # resampling
        particles = resample(particles, weights)
        frames.append(get_frame(particles, gridmap, map_res))

    gif.save(frames, "mcl_final.gif",
             duration=30, unit="s",
             between="startend")
    return particles
