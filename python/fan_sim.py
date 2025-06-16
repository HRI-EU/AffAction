#!/usr/bin/env python3
import sys
import json
import os
import platform
from pathlib import Path
import logging
import cv2
import numpy as np

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
        logger.debug(f"Running on {os_name} from {smile_ws_path}")

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
            
        from pyAffaction import (LlmSim, addResourcePath, setLogLevel)
        logger.debug(f"Setting up the simulator. PATH: {os.environ.get('PATH')}")
        #setLogLevel(-1)
        addResourcePath(str(smile_ws_path / "config"))
        addResourcePath(str(smile_ws_path / "config" / "xml" / "examples"))

        self.sim = LlmSim()
        self.sim.noTextGui = True
        self.sim.speedUp = 1
        self.sim.verbose = False
        self.sim.addVirtualCamera(width=320, height=240, withGui=False)
        self.sim.xmlFileName = self.scene

    def stop(self):
        """
        Stop the simulator if needed. Called after the GUI is closed.
        """
        if self.sim:
            logger.info("Stopping simulator.") 
            self.sim.stop()
            self.sim = None




def main():
    sim_manager = SimulatorManager(scene="g_example_spiderbot.xml")
    sim_manager.setup("build")
    global sim   # For interactive console needed
    sim = sim_manager.sim
    sim.init(True)
    sim.callEvent("Start")
    sim.callEvent("Process")

    sim.plan_fb_nonblock("get bottle_of_olive_oil")

    try:
        while True:
            sim.step()
            color_img = sim.captureColorImageFromFrame("camera_01")
            color_np = np.array(color_img)
            color_bgr = cv2.cvtColor(color_np, cv2.COLOR_RGB2BGR)
            cv2.imwrite("color_image.jpg", color_bgr)
            controls = sim.getControls(["hand_robot_left_1", "hand_robot_left_2", "hand_robot_right_1", "hand_robot_right_2"])
            logger.info("Controls:\n%s", json.dumps(controls, indent=2))
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

