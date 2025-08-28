#!/usr/bin/env python3
import sys
import json
import os
import time
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
        setLogLevel(-1)
        addResourcePath(str(smile_ws_path / "config"))
        addResourcePath(str(smile_ws_path / "config" / "xml" / "examples"))

        self.sim = LlmSim()
        self.sim.noTextGui = True
        self.sim.speedUp = 1
        self.sim.verbose = False
        self.sim.addVirtualCamera("", width=320, height=240)
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

def pour_into(SIMULATION, source_container_name: str, target_container_name: str, hand_name: str) -> str:
    """
    Pour a source container into a target container. You do not have to grasp the source container before 
    pouring it. You hold it in your hand after finishing.

    :param source_container_name: The name of the container to pour from.
    :param target_container_name: The name of the container to pour into.
    :return: Result message.
    """

    # We get the object if it is not already held in the hand.
    holding_hand = SIMULATION.is_held_by(source_container_name)
    get_command = ""
    if not holding_hand:
        get_command = f"get {source_container_name} {hand_name};"

    # The strings support and support_frame will be remembered so that the object will
    # be put back to the same place where it has been picked up. In cases when the object is
    # already held in the hand, they may be empty. Then, the object is put on the best
    # support location according to the action cost.
    support = SIMULATION.get_parent_entity(source_container_name)
    support_frame = SIMULATION.get_closest_parent_affordance(source_container_name, "Supportable")

    # We move the object above the target container, pour, and just put it
    # somewhere. That's not so nice, but pretty safe and works in most cases.
    action_command = (
        f"{get_command}"
        f"move {source_container_name} above {target_container_name} height 0.15;"
        f"pour_put {source_container_name} {target_container_name} putPlace {support};"
        f"pose default"
    )
    results = SIMULATION.plan_fb_rich(action_command)
    logger.info(f"Planning result: {json.dumps(results, indent=2)}")

    if not results:
        logger.info(f"First planning run failed - trying more robust one ...")
        action_command = (
            f"{get_command}"
            f"move {source_container_name} above {target_container_name} height 0.15;"
            f"pour {source_container_name} {target_container_name};"
            f"put {source_container_name};"
            f"pose default"
        )
        results = SIMULATION.plan_fb_rich(action_command)

    logger.info(f"Planning result: {json.dumps(results, indent=2)}")

    actions_string = '; '.join(results[0]["actions"]) if results else ""
    
    return actions_string



def main():
    sim_manager = SimulatorManager(scene="g_example_opposing_icra.xml")
    sim_manager.setup("build")
    global sim   # For interactive console needed
    sim = sim_manager.sim
    sim.init(True)
    sim.addVirtualCamera("camera_0", 320, 240)
    sim.addVirtualCamera("camera_1", 320, 240)
    sim.addVirtualCamera("camera_2", 320, 240)
    sim.callEvent("Start")
    sim.callEvent("Process")

    # grounded_actions = pour_into(sim, "bottle_of_pesto_sauce", "glass_blue", "hand_robot_left")
    grounded_actions = pour_into(sim, "bottle_of_gin", "glass_green", "hand_robot_right3")
    # grounded_actions = pour_into(sim, "bottle_of_tomato_sauce", "glass_green", "hand_robot_right")
    
    if not grounded_actions:
        logger.info("No solution found")
    else:
        logger.info(f"Executing: {grounded_actions}")
        sim.execute(grounded_actions)

    try:
        while True:
            sim.step()

            # Grab first camera
            color_img0 = sim.captureColorImageFromFrame("camera_0")
            color_np0 = np.array(color_img0)
            color_bgr0 = cv2.cvtColor(color_np0, cv2.COLOR_RGB2BGR)
            #cv2.imwrite("color_image.jpg", color_bgr0)
            cv2.imshow("Screen capture 0", color_bgr0)

            # Grab second camera
            color_img1 = sim.captureColorImageFromFrame("camera_1")
            color_np1 = np.array(color_img1)
            color_bgr1 = cv2.cvtColor(color_np1, cv2.COLOR_RGB2BGR)
            #cv2.imwrite("color_image.jpg", color_bgr1)
            cv2.imshow("Screen capture 1", color_bgr1)

            # Grab third camera
            color_img2 = sim.captureColorImageFromFrame("camera_2")
            color_np2 = np.array(color_img2)
            color_bgr2 = cv2.cvtColor(color_np2, cv2.COLOR_RGB2BGR)
            #cv2.imwrite("color_image.jpg", color_bgr2)
            cv2.imshow("Screen capture 2", color_bgr2)

            
            key = cv2.waitKey(1)

            controls = sim.getControls(["hand_robot_left3", "hand_robot_right3"])
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

