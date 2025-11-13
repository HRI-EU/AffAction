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

import zmq
import json
import time
import math
import signal
import sys



running = True

def signal_handler(sig, frame):
    global running
    print("\n[Ctrl-C] Stopping subscriber...")
    running = False

signal.signal(signal.SIGINT, signal_handler)





def _send_payload_once(payload: dict, endpoint: str) -> None:
    """Helper to send one JSON message via ZeroMQ PUB socket."""
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.connect(endpoint)
    time.sleep(0.1)  # Allow subscribers to connect (PUB-SUB quirk)
    pub.send_string(json.dumps(payload, indent=4))
    pub.close()
    ctx.term()


def ptu_command_once(pan_in_degrees: float, tilt_in_degrees: float, endpoint: str = "tcp://localhost:40007") -> None:
    """
    Send a single PTU pan/tilt position command.

    Parameters
    ----------
    pan_in_degrees : float
        Desired pan angle in degrees.
    tilt_in_degrees : float
        Desired tilt angle in degrees.
    endpoint : str, optional
        ZeroMQ PUB socket endpoint (default: "tcp://localhost:5560").
    """
    payload = {
        "robot_name": "PW70",
        "timestamp": 0,
        "actuators": [],
        "quit": False
    }

    payload["actuators"].append({
        "id": "joint_1",
        "type": "joint",
        "index": 0,
        "position": math.radians(pan_in_degrees),
        "no_vmax": 0.2,
        "no_tmc": 0.1
    })

    payload["actuators"].append({
        "id": "joint_2",
        "type": "joint",
        "index": 1,
        "position": math.radians(tilt_in_degrees),
        "no_vmax": 0.2,
        "no_tmc": 0.1
    })
    
    _send_payload_once(payload, endpoint)



def ptu_feedback_loop(endpoint: str = "tcp://localhost:40006"):
    """Subscribe to a ZMQ feedback stream and print the incoming data."""
    global running
    
    # Create ZeroMQ subscriber
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(endpoint)
    sub.setsockopt_string(zmq.SUBSCRIBE, "")

    print(f"[feedback_subscriber] Connected to {endpoint}")

    try:
        while running:
            try:
                # Non-blocking receive
                try:
                    msg = sub.recv(flags=zmq.NOBLOCK)
                except zmq.Again:
                    time.sleep(0.01)
                    continue

                # Parse JSON safely
                try:
                    j = json.loads(msg.decode('utf-8'))
                    print(json.dumps(j, indent=2))
                except json.JSONDecodeError as e:
                    print(f"JSON parse error: {e}")
                    continue

            except Exception as e:
                print(f"Error while receiving feedback: {e}")
                time.sleep(0.01)
                continue

            time.sleep(0.01)

    finally:
        print(f"feedback loop says goodbye")
        sub.close()
        ctx.term()
        running = True


def ptu_feedback_once(endpoint: str = "tcp://localhost:40006"):
    """Receive a single JSON feedback message and print it to the console."""
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(endpoint)
    sub.setsockopt_string(zmq.SUBSCRIBE, "")

    try:
        msg = sub.recv()
        j = json.loads(msg.decode("utf-8"))
        print(json.dumps(j, indent=2))
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sub.close()
        ctx.term()
        

def ptu_quit(endpoint: str = "tcp://localhost:40007") -> None:
    """
    Send a 'quit' command to terminate the PTU control loop.

    Parameters
    ----------
    endpoint : str, optional
        ZeroMQ PUB socket endpoint (default: "tcp://localhost:40007").
    """
    payload = {"quit": True}
    _send_payload_once(payload, endpoint)
