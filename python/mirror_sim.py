#!/usr/bin/env python3
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

import sys
import json
import os
import time
import platform
from pathlib import Path
import logging
import cv2
import numpy as np
import base64

logging.basicConfig(
    level=logging.INFO,                              # show INFO and above
    format="%(asctime)s | %(name)s | %(levelname)s | %(message)s",
)
logger = logging.getLogger("SimulatorManager")

class SimulatorManager:
    """
    Manages the lifecycle of the LlmSim simulator, including initialization,
    starting the GUI, and shutting down.
    """
    def __init__(self, scene: str = "g_attentive_support.xml"):
        """
        Create a SimulatorManager with default configurations.

        :param scene: Name of the XML configuration file for the simulator.
        """
        self.scene = scene
        self.sim = None

    def setup(self, build_directory: str):
        """
        Setup and configure the simulator instance.
        """
        smile_ws_path = Path(build_directory).resolve()
        os_name = platform.system()
        logger.info(f"Running on {os_name} from {smile_ws_path}")

        if os_name == "Windows":
            # Update sys.path and PATH environment variable for Windows dlls
            if smile_ws_path.name == "build":
                dll_path = smile_ws_path / "bin" / "Release"
            else:
                dll_path = smile_ws_path / "bin"
            sys.path.append(str(dll_path))
            os.environ["PATH"] = str(dll_path) + os.pathsep + os.environ.get("PATH", "")
        elif os_name == "Darwin":
            sys.path.append(str(smile_ws_path / "lib" ))
            sys.path.append(str(smile_ws_path / "lib" / "Release"))
        elif os_name == "Linux":
            sys.path.append(str(smile_ws_path / "lib"))
        else:
            logger.error(f"Unknown OS: {os_name}")
            
        from pyAffaction import (LlmSim, addResourcePath, printResourcePath, setLogLevel)
        logger.debug(f"Setting up the simulator. PATH: {os.environ.get('PATH')}")
        setLogLevel(0)
        addResourcePath(str(smile_ws_path / "config"))
        addResourcePath(str(smile_ws_path / "config" / "xml" / "examples"))
        addResourcePath(str(smile_ws_path / "config" / "xml" / "Franka"))
        printResourcePath()

        self.sim = LlmSim()
        self.sim.noTextGui = True
        self.sim.speedUp = 1
        self.sim.verbose = False
        self.sim.xmlFileName = self.scene
        self.sim.dt = 0.05
        self.sim.enableWireframeToggle = False

    def stop(self):
        """
        Stop the simulator if needed. Called after the GUI is closed.
        """
        if self.sim:
            logger.info("Stopping simulator.") 
            self.sim.stop()
            self.sim = None


def crop_bbox(img, bbox, margin=0, clip=True, round_coords=True, inclusive_max=False):
    """
    img: OpenCV image (H x W x C, BGR)
    bbox: [minX, minY, maxX, maxY] in pixels
    margin: extra pixels to include around the box
    clip: clamp to image bounds
    round_coords: round float coords to int
    inclusive_max: set True if your maxX/maxY are inclusive (common in some tools)
    """
    x1, y1, x2, y2 = bbox
    if inclusive_max:
        x2 += 1; y2 += 1
    if round_coords:
        x1, y1, x2, y2 = map(lambda v: int(round(v)), (x1, y1, x2, y2))
    x1 -= margin; y1 -= margin; x2 += margin; y2 += margin
    if clip:
        h, w = img.shape[:2]
        x1 = max(0, x1); y1 = max(0, y1)
        x2 = min(w, x2); y2 = min(h, y2)
    if x2 <= x1 or y2 <= y1:
        #return img.copy()
        raise ValueError(f"Invalid bbox after processing: ({x1},{y1},{x2},{y2}). h={h} w={w}")
    return img[y1:y2, x1:x2].copy()

def resize_min_dim(img, target_min=160):
    h, w = img.shape[:2]
    min_dim = min(h, w)
    if min_dim == 0:
        raise ValueError("Empty image/ROI.")
    scale = target_min / float(min_dim)
    new_w, new_h = int(round(w * scale)), int(round(h * scale))
    interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_CUBIC
    return cv2.resize(img, (new_w, new_h), interpolation=interp)


def convert_base64_image(b64_str) -> np.ndarray:
    """
    Convert base64-encoded JPEG/PNG (with or without data URL header) 
    into an OpenCV BGR image.
    """
    # Ensure it's a string
    if not isinstance(b64_str, str):
        raise TypeError(f"Expected str, got {type(b64_str)}")

    # If it's a data URL, strip the header before decoding
    if b64_str.startswith("data:"):
        b64_str = b64_str.split(",", 1)[1]

    # Fix missing padding (common issue when transmitting base64)
    pad_len = len(b64_str) % 4
    if pad_len:
        b64_str += "=" * (4 - pad_len)

    # Base64 decode to bytes
    img_bytes = base64.b64decode(b64_str)

    # Bytes to NumPy array (uint8)
    np_buf = np.frombuffer(img_bytes, dtype=np.uint8)

    # Decode to OpenCV image (BGR format)

    if np_buf.size == 0:
        print("Empty buffer, cannot decode.")
        bgr_img = None
    else:
        bgr_img = cv2.imdecode(np_buf, cv2.IMREAD_COLOR)
        
    if bgr_img is None:
        raise ValueError("cv2.imdecode failed — data may not be a valid image")

    return bgr_img


def main():
    help_text = """
    ==================================================
    Simulation Control Help
    ==================================================
    This script starts a simulator with a virtual camera,
    processes images, and lets the robot gaze at objects.
    You can interact with the simulation through the
    following keyboard controls, pressed over the OpenCV window:

      General:
        q or ESC  - Quit the simulation
        Arrow keys - Report pressed arrow key (LEFT, UP, RIGHT, DOWN)

      Head gestures:
        a - Perform a "yes" gesture (nodding)
        b - Perform a "no" gesture (shaking head)

      Camera and gaze:
        c - Print current mirror eyes data
        g - Print the current camera model

      Pupil control:
        d - Set pupil speed weight to 0.0
        e - Set pupil speed weight to 1.0
        f - Set pupil speed weight to 0.9

    Notes:
    - The simulator shows an RGB window of the virtual camera.
    - Press 'l' in the simulator window to make the robot gaze
      at the object under the mouse pointer.
    - The ROI (Region of Interest) window shows the cropped
      bounding box of the currently gazed object.
    ==================================================
    """
    print(help_text)

    loop_count = 0
    count = -1
    sim_manager = SimulatorManager(scene="g_attentive_support.xml")
    sim_manager.setup("build")
    sim = sim_manager.sim
    #sim.addComponentArgument("-virtual_image_tracking -virtual_image_tracking.width 640 -virtual_image_tracking.height 480 -virtual_image_tracking.camera_type AzureKinect_WFOV")
    sim.addComponentArgument("-image_tracking -skeleton_tracking")
    sim.addLandmarkRouter(camera_name="camera_0")
    sim.init(True)
    sim.callEvent("Start")
    sim.callEvent("Process")

    # Lets the robot look at the human agent
    sim.setGazeTarget("Head_Daniel")

    try:
        while loop_count >= 0:

            # Simulation step (100Hz usually)
            logger.debug("Step")
            sim.step()

            # skip until the 10-th step
            loop_count += 1
            #if loop_count % 10 != 0:  
                #continue

            # Capture virtual camera image (stored as jpg-compressed base64 data)
            new_count, image_b64 = sim.getCameraImage(count)

            if new_count <= count or new_count <= 0:
                continue

            count = new_count

            # Make rgb-image out of it
            image = convert_base64_image(image_b64)
            cv2.namedWindow("RGB", cv2.WINDOW_AUTOSIZE)
            cv2.imshow("RGB", image)
            
            # Get bounding box of object that is currently looked at. Press 'l' in simlator window
            # to let robot gaze at object under mouse tip
            bb = sim.getGazeObjectBoundingBox()

            # Show cropped image of object that is looked at
            if (len(bb) == 4) and image is not None:
                roi = crop_bbox(image, bb, margin=10, clip=True, round_coords=True, inclusive_max=False)
                roi_resized = resize_min_dim(roi, target_min=160)
                cv2.namedWindow("ROI", cv2.WINDOW_AUTOSIZE) # autosized window to avoid distortion
                cv2.imshow("ROI", roi_resized)

            # Expose a few API calls through key presses
            key = cv2.waitKey(1)   # pause 1 ms
            if key != -1:
                k = key & 0xFF                    # lowest byte is portable
                if k in (81, 82, 83, 84):         # arrow keys in many backends
                    names = {81: "LEFT", 82: "UP", 83: "RIGHT", 84: "DOWN"}
                    logger.info(f"Pressed {names[k]}")
                elif k == 27 or k == ord('q'):    # ESC or 'q' to quit
                    logger.info("Quit key pressed")
                    loop_count = -1
                elif 32 <= k <= 126:              # printable ASCII
                    logger.info(f"Pressed '{chr(k)}' (code={k})")
                    if k == ord('a'):
                        sim.setHeadGesture("yes", 3.14*5.0/180.0, 3)   
                    elif k == ord('b'):
                        sim.setHeadGesture("no", 3.14*5.0/180.0, 3)   
                    elif k == ord('c'):
                        logger.info(f"{sim.getMirrorEyesData()}")
                    elif k == ord('d'):
                        logger.info("Setting pupil speed weight to 0.0")
                        sim.setPupilSpeedWeight(0.0)
                    elif k == ord('e'):
                        logger.info("Setting pupil speed weight to 1.0")
                        sim.setPupilSpeedWeight(1.0)
                    elif k == ord('f'):
                        logger.info("Setting pupil speed weight to 0.1")
                        sim.setPupilSpeedWeight(0.9)
                    elif k == ord('g'):
                        logger.info(f"{sim.getCameraModel()}")
                    
                else:
                    logger.info(f"Pressed key code {k}")

    except KeyboardInterrupt:
        print("Exiting simulation loop via Ctrl-C...")        
        sim.callEvent("Stop")
        sim.callEvent("Process")
    finally:
        sim_manager.stop()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        logger.info("Interrupted by user. Exiting...")
        sys.exit(0)
    except SystemExit:
        logger.info("Received SystemExit; shutting down.")
        sys.exit(0)
    except Exception as e:
        logger.exception("An unexpected error occurred.")
        sys.exit(1)

