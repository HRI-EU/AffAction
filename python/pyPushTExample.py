#!/usr/bin/python
# -*- coding: utf-8 -*-
#
#  Copyright (c) Honda Research Institute Europe GmbH.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
#  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
#  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
#  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
#  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
#  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

import platform
import sys
import numpy as np
import matplotlib.pyplot as plt

# Assumes to be started from AttentiveSupport/build or AttentiveSupport/install
if platform.system() == "Linux":
    sys.path.append("lib")
elif platform.system() == "Windows":
    sys.path.append("bin")

from pyPushT import *




def show_image(image_array):
    """
    Displays an image represented as a NumPy array of shape (height, width, 3).

    :param image_array: NumPy array of shape (height, width, 3), where each pixel is represented
                        by three consecutive doubles (R, G, B).
    """
    # Validate the input dimensions
    if len(image_array.shape) != 3 or image_array.shape[2] != 3:
        raise ValueError("Input array must have shape (height, width, 3).")

    # Clip the values to [0, 1] range for display if necessary
    image = np.clip(image_array, 0.0, 1.0)

    # Display the image
    plt.imshow(image)
    plt.axis('off')  # Turn off the axis
    plt.show()




p = PushT()
p.init(True)

# Example usage: 
# image_array = p.get_observation()
# show_image(image_array)

#vel_des = np.array([0.01, 0.0]) 
#p.step(vel_des)

