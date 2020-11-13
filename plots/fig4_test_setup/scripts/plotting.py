#!/usr/bin/env python

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

import math
import time
import numpy as np
from matplotlib import animation
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import os

import csv
import matplotlib
matplotlib.use('TkAgg')

sx = -4
sy = 2.5

# scenario = "backward"
scenario = "parallel"

if scenario == "backward":
    ex = 1.359
    ey = -3.86443643718
    ephi = 1.581
    XYbounds = [-13.6406951857, 16.3591910364,
                -5.15258191624, 5.61797800844]

if scenario == "parallel":
    ex = 2.0
    ey = -1.25
    ephi = 0.0
    XYbounds = [-12.0, 17.0, -2.5, 5.6]


### Note: please replace the two size parameters with the data size in the data folder
### (1) hybrid_a_output_size
### (2) iterative_anchoring_output_size


# trajectories plot
fig1 = plt.figure(1)
ax = fig1.add_subplot(111)
plt.rcParams["font.weight"] = "bold"
plt.rcParams["axes.labelweight"] = "bold"
warm_start_arrow = None
smoothing_arrow = None
for i in range(0, hybrid_a_output_size[0]):
    # warm start
    downx = 1.055 * math.cos(phi_out[i] - math.pi / 2)
    downy = 1.055 * math.sin(phi_out[i] - math.pi / 2)
    leftx = 1.043 * math.cos(phi_out[i] - math.pi)
    lefty = 1.043 * math.sin(phi_out[i] - math.pi)
    x_shift_leftbottom = x_out[i] + downx + leftx
    y_shift_leftbottom = y_out[i] + downy + lefty
    warm_start_car = patches.Rectangle((x_shift_leftbottom, y_shift_leftbottom), 3.89 + 1.043, 1.055*2,
                                    angle=phi_out[i] * 180 / math.pi, linewidth=1, edgecolor='r', facecolor='none')
    warm_start_arrow = patches.Arrow(
        x_out[i], y_out[i], 0.25*math.cos(phi_out[i]), 0.25*math.sin(phi_out[i]), 0.2, edgecolor='r', facecolor='r')
    # ax.add_patch(warm_start_car)
    ax.add_patch(warm_start_arrow)
for i in range(0, iterative_anchoring_output_size[0]):
    # iterative_anchoring
    downx = 1.055 * math.cos(opt_phi_out[i] - math.pi / 2)
    downy = 1.055 * math.sin(opt_phi_out[i] - math.pi / 2)
    leftx = 1.043 * math.cos(opt_phi_out[i] - math.pi)
    lefty = 1.043 * math.sin(opt_phi_out[i] - math.pi)
    x_shift_leftbottom = opt_x_out[i] + downx + leftx
    y_shift_leftbottom = opt_y_out[i] + downy + lefty
    smoothing_car = patches.Rectangle((x_shift_leftbottom, y_shift_leftbottom), 3.89 + 1.043, 1.055*2,
                                    angle=opt_phi_out[i] * 180 / math.pi, linewidth=1, edgecolor='y', facecolor='none', alpha=0.2)
    smoothing_arrow = patches.Arrow(
        opt_x_out[i], opt_y_out[i], 0.25*math.cos(opt_phi_out[i]), 0.25*math.sin(opt_phi_out[i]), 0.2, edgecolor='y', facecolor='y')
    ax.add_patch(smoothing_car)
    ax.add_patch(smoothing_arrow)

ax.legend([warm_start_arrow, smoothing_arrow], ['Hybrid A*', "Iterative Anchoring Smoothing"], loc='lower right', fontsize='20',bbox_to_anchor=(1.0, 0.09))
ax.plot(sx, sy, "s")
ax.plot(ex, ey, "s")
if scenario == "backward":
    left_boundary_x = [-13.6407054776, 0.0, 0.0515703622475]
    left_boundary_y = [0.0140634663703, 0.0, -5.15258191624]
    down_boundary_x = [0.0515703622475, 2.8237895441]
    down_boundary_y = [-5.15258191624, -5.15306980547]
    right_boundary_x = [2.8237895441, 2.7184833539, 16.3592013995]
    right_boundary_y = [-5.15306980547, -0.0398078878812, -0.011889513383]
    up_boundary_x = [16.3591910364, -13.6406951857]
    up_boundary_y = [5.60414234644, 5.61797800844]
    ax.plot(left_boundary_x, left_boundary_y, "k", linewidth=2)
    ax.plot(down_boundary_x, down_boundary_y, "k", linewidth=2)
    ax.plot(right_boundary_x, right_boundary_y, "k", linewidth=2)
    ax.plot(up_boundary_x, up_boundary_y, "k", linewidth=2)
if scenario == "parallel":
    left_boundary_x = [-12.0, 0.0, 0.0]
    left_boundary_y = [0.0, 0.0, -2.5]
    down_boundary_x = [0.0, 7.5]
    down_boundary_y = [-2.5, -2.5]
    right_boundary_x = [7.5, 7.5, 17]
    right_boundary_y = [-2.5, 0.0, 0.0]
    up_boundary_x = [17.0, -12.0]
    up_boundary_y = [5.6, 5.6]
    ax.plot(left_boundary_x, left_boundary_y, "k", linewidth=1)
    ax.plot(down_boundary_x, down_boundary_y, "k", linewidth=1)
    ax.plot(right_boundary_x, right_boundary_y, "k", linewidth=1)
    ax.plot(up_boundary_x, up_boundary_y, "k", linewidth=1)

ax.set_xlim([-10, 15])
ax.set_ylim([-4, 6])
plt.axis('auto')
