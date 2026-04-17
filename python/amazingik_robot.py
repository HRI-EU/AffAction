from __future__ import annotations

import sys
import time
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from lerobot.robots import Robot, RobotConfig

log = logging.getLogger(__name__)


@RobotConfig.register_subclass("amazingik_robot")
@dataclass
class AmazingIKRobotConfig(RobotConfig):
    build_directory: Path = Path("/hri/localdisk/asochop/workspace/robot_affaction/build")
    config_directory: Path = Path("config/xml/Franka")
    robo_computer_ip: str = "192.168.1.101"
    xml_file_name: str = "c_robo_datacollection_sim.xml"
    input_type: str = "Wrench"
    debug_level: int = 0
    camera_name: str = "camera_0"
    camera_width: int = 224
    camera_height: int = 224

    # Optional: if you want a stable user-visible id for calibration files
    id: str = "amazingik_default"


class AmazingIKBackend:
    """
    Thin backend wrapper around your existing pyTeleOp-based simulator/control object.
    This is intentionally NOT the LeRobot Robot subclass.
    """

    def __init__(self, config: AmazingIKRobotConfig):
        self.config = config
        self.sim = None
        self._connected = False

        # Will be populated in connect()
        self._policy_to_command = None
        self._get_log_level = None

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        if self._connected:
            return

        smile_ws_path = Path(self.config.build_directory).resolve()
        sys.path.append(str(smile_ws_path / "bin"))
        sys.path.append(str(smile_ws_path / "lib"))

        from pyTeleOp import TeleOp, setLogLevel, getLogLevel, policy_to_command, addResourcePath, printResourcePath

        self._policy_to_command = policy_to_command
        self._get_log_level = getLogLevel
        setLogLevel(self.config.debug_level)
        addResourcePath(str(smile_ws_path / "config"))
        addResourcePath(str(smile_ws_path / "config" / "franka_description" / "meshes"))
        addResourcePath(str(smile_ws_path / "config" / "xml" / self.config.config_directory))
        printResourcePath()

        self.sim = TeleOp()
        self.sim.xmlFileName = self.config.xml_file_name

        if self.config.config_directory.exists():
            self.sim.configDirectory = str(self.config.config_directory)
        else:
            self.sim.configDirectory = str(self.config.build_directory / self.config.config_directory)

        if not Path(self.sim.configDirectory).exists():
            raise FileNotFoundError(f"Build directory does not exist: {self.config.build_directory=} {self.config.config_directory=}")
        
        self.sim.configDirectory = str(self.config.build_directory / self.config.config_directory)
        self.sim.enableRealGraphVisualization = False
        self.sim.noLimits = False
        self.sim.withScene = False
        self.sim.inputType = self.config.input_type

        self.sim.init(True)
        self.sim.addVirtualCamera(
            self.config.camera_name,
            "Logitech_C910",
            self.config.camera_width,
            self.config.camera_height,
        )
        self.sim.callEvent("Start")
        self.sim.callEvent("Process")
        self.sim.step()

        self._connected = True
        log.info("AmazingIK backend connected.")

    def disconnect(self) -> None:
        if not self._connected:
            return

        try:
            self.sim.callEvent("Stop")
            self.sim.stop()
        finally:
            self.sim = None
            self._connected = False
            log.info("AmazingIK backend disconnected.")

    def configure(self) -> None:
        # Put one-time runtime configuration here if needed.
        # For your current simulator setup, connect() already does most of it.
        pass

    def get_proprioception(self) -> np.ndarray:
        raw_data = self.sim.getCollectedData()
        # Keeping your current choice:
        return np.asarray(raw_data[5][42:49], dtype=np.float32)

    def get_camera_image(self) -> np.ndarray:
        color_img = self.sim.captureColorImageFromFrame(self.config.camera_name)
        rgb = np.asarray(color_img, dtype=np.uint8)
        return rgb

    def send_wrench(self, wrench: np.ndarray) -> np.ndarray:
        """
        wrench: shape (6,), [fx, fy, fz, tx, ty, tz]
        Returns the actual wrench sent.
        """
        wrench = np.asarray(wrench, dtype=np.float32).reshape(6)

        self.sim.setWrench(
            float(wrench[0]),
            float(wrench[1]),
            float(wrench[2]),
            float(wrench[3]),
            float(wrench[4]),
            float(wrench[5]),
        )
        self.sim.step()

        if self._get_log_level is not None and self._get_log_level() > 0:
            input("Hit enter")

        return wrench

    def get_current_ee_wrench(self) -> np.ndarray:
        return np.asarray(self.sim.getEndEffectorWrench(), dtype=np.float32)


class AmazingIKRobot(Robot):
    """
    LeRobot-compatible robot interface around AmazingIKBackend.
    """

    config_class = AmazingIKRobotConfig
    name = "amazingik_robot"

    def __init__(self, config: AmazingIKRobotConfig):
        super().__init__(config)
        self.backend = AmazingIKBackend(config)

    @property
    def observation_features(self) -> dict[str, type | tuple]:
        return {
            "observation.state": (7,),
            "observation.images.camera_0": (
                self.config.camera_height,
                self.config.camera_width,
                3,
            ),
        }

    @property
    def action_features(self) -> dict[str, type]:
        return {
            "action.wrench.fx": float,
            "action.wrench.fy": float,
            "action.wrench.fz": float,
            "action.wrench.tx": float,
            "action.wrench.ty": float,
            "action.wrench.tz": float,
        }

    @property
    def is_connected(self) -> bool:
        return self.backend.is_connected

    def connect(self, calibrate: bool = True) -> None:
        self.backend.connect()

        if calibrate and not self.is_calibrated:
            self.calibrate()

        self.configure()
        log.info("%s connected.", self)

    @property
    def is_calibrated(self) -> bool:
        # If AmazingIK / your simulator does not require LeRobot-style calibration,
        # returning True is acceptable per the docs.
        return True

    def calibrate(self) -> None:
        # No-op for now. Add file-backed calibration here later if needed.
        pass

    def configure(self) -> None:
        self.backend.configure()

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected.")

        obs = {
            "observation.state": self.backend.get_proprioception(),
            "observation.images.camera_0": self.backend.get_camera_image(),
        }
        return obs

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected.")

        wrench = np.array(
            [
                action["action.wrench.fx"],
                action["action.wrench.fy"],
                action["action.wrench.fz"],
                action["action.wrench.tx"],
                action["action.wrench.ty"],
                action["action.wrench.tz"],
            ],
            dtype=np.float32,
        )

        sent = self.backend.send_wrench(wrench)

        return {
            "action.wrench.fx": float(sent[0]),
            "action.wrench.fy": float(sent[1]),
            "action.wrench.fz": float(sent[2]),
            "action.wrench.tx": float(sent[3]),
            "action.wrench.ty": float(sent[4]),
            "action.wrench.tz": float(sent[5]),
        }

    def disconnect(self) -> None:
        self.backend.disconnect()
        
