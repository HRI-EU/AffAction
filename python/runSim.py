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

"""
Example Python script demonstrating a simulator GUI and an interactive console.
"""

import sys
import os
import code
import threading
import logging
import time
from pathlib import Path


# Global Constants
SMILE_WS_PATH = Path("build")
CFG_ROOT_DIR = SMILE_WS_PATH / "config"
CFG_DIR = CFG_ROOT_DIR / "xml" / "examples"

# Local package imports
sys.path.append(str(SMILE_WS_PATH / "lib"))
sys.path.append(str(SMILE_WS_PATH / "lib" / "Release"))
sys.path.append(str(SMILE_WS_PATH / "bin" / "Release"))

from pyAffaction import (
    LlmSim,
    addResourcePath,
    setLogLevel,
)

# Add resource paths
addResourcePath(str(CFG_ROOT_DIR))
addResourcePath(str(CFG_DIR))
        
# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s'
)
logger = logging.getLogger(__name__)


class SimulatorManager:
    """
    Manages the lifecycle of the LlmSim simulator, including initialization,
    starting the GUI, and shutting down.
    """
    def __init__(self, scene: str = "g_attentive_support.xml", tts: str = "native"):
        """
        Create a SimulatorManager with default configurations.

        :param scene: Name of the XML configuration file for the simulator.
        :param tts: Text-to-speech setting (default: "native").
        """
        self.scene = scene
        self.tts = tts
        self.sim = None

    def setup(self):
        """
        Setup and configure the simulator instance.
        """
        logger.info("Setting up the simulator.")
        self.sim = LlmSim()
        self.sim.noTextGui = True
        self.sim.unittest = False
        self.sim.speedUp = 3
        self.sim.noLimits = False
        self.sim.verbose = False
        self.sim.xmlFileName = self.scene

    def start_gui_blocking(self):
        """
        Start the simulator GUI in a blocking manner.
        Must be called from the main thread.
        """
        if not self.sim:
            raise RuntimeError("Simulator not set up. Call setup() first.")
        logger.info("Starting simulator GUI in blocking mode.")
        self.sim.initBlocking(True)

    def quit_blocking(self):
        """
        Request the simulator to quit in a blocking way.
        This typically closes the GUI.
        """
        if self.sim:
            logger.info("Requesting simulator to quit...")
            self.sim.quitBlocking()

    def stop(self):
        """
        Stop the simulator if needed. Called after the GUI is closed.
        """
        if self.sim:
            logger.info("Stopping simulator.")
            self.sim.stop()

    def cleanup(self):
        """
        Clean up references to the simulator, allowing Python to GC it.
        """
        logger.info("Cleaning up simulator references.")
        self.sim = None


def interactive_console_thread(sim_manager: SimulatorManager):
    """
    Run an interactive console in a separate thread. This console provides
    commands that can interact with the simulator manager.

    :param sim_manager: The SimulatorManager instance controlling the simulator.
    """

    def raise_sys_exit():
        """
        Quit the simulator and forcibly exit this console thread
        by raising SystemExit.
        """
        logger.info("User requested exit from console, raising SystemExit.")
        sim_manager.quit_blocking()
        raise SystemExit

    banner = (
        "Interactive console started. "
        "Type 'quit()' or 'exit()' to shut down the simulator and console.\n"
        "The simulator is accessible as 'sim'. For example: sim.someMethod()"
    )

    # Provide the 'quit()' and 'exit()' commands that forcibly end the console,
    # and also expose the simulator instance for user interaction.
    local_vars = {
        "quit": raise_sys_exit,
        "exit": raise_sys_exit,
        "sim": sim_manager.sim,          # Direct reference to the simulator
        "manager": sim_manager           # If you want direct manager access
    }

    console = code.InteractiveConsole(locals=local_vars)

    try:
        console.interact(banner=banner, exitmsg="Goodbye!")
    finally:
        logger.info("Interactive console has been terminated.")
        # In case the user closed the console without typing quit():
        sim_manager.quit_blocking()




def main():
    """
    Main entry point for the script. Sets up the simulator, starts an interactive
    console in a thread, and launches the simulator GUI in the main thread.
    """
    logger.info(f"VIRTUAL_ENV: {os.environ.get('VIRTUAL_ENV')}")
    logger.info(f"PATH: {os.environ.get('PATH')}")
    logger.info(f"CFG_DIR: {CFG_DIR}")

    # Create and set up the simulator manager
    sim_manager = SimulatorManager()
    sim_manager.setup()

    # Start the interactive console in a separate thread
    console_thread = threading.Thread(
        target=interactive_console_thread,
        args=(sim_manager,),
        daemon=True
    )
    console_thread.start()

    # Start the GUI in the main thread (blocking)
    sim_manager.start_gui_blocking()
    time.sleep(0.2) # Let threads finish
    
    # After GUI closes, wait briefly for console thread to exit
    logger.info("Waiting for interactive console thread to join...")
    console_thread.join(timeout=5)
    logger.info("Console thread joined or timed out.")

    # Stop the simulator if it hasn't been stopped already
    sim_manager.stop()
    sim_manager.cleanup()

    logger.info("Script execution completed.")


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

