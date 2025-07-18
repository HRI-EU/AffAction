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
        "joints": {
          "pan": { "index": 0, "position_command": math.radians(pan_in_degrees), "novmax": 0.2, "notmc": 0.1 },
          "tilt": { "index": 1, "position_command": math.radians(tilt_in_degrees), "novmax": 0.2, "notmc": 0.1 }
        },
        "quit": False
    }
    _send_payload_once(payload, endpoint)


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
