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
#      "rotation":{
#         "x":0.306903929,
#         "y":0.176378131,
#         "z":-0.0579901077,
#         "w":0.9334548
#      }


default_text = '''
{
   "Head":{
      "position":{
         "x":0.0,
         "y":1.7,
         "z":0.0
      },
      "rotation":{
         "x":-0.0579901077,
         "y":0.9334548,
         "z":0.306903929,
         "w":0.176378131
      }
   },
   "LeftEye":{
      "position":{
         "x":-0.07225178,
         "y":1.43196809,
         "z":0.009275467
      },
      "rotation":{
         "x":-0.0579901077,
         "y":0.9334548,
         "z":0.306903929,
         "w":0.176378131
      }
   },
   "RightEye":{
      "position":{
         "x":0.0434882864,
         "y":1.43196809,
         "z":0.05463351
      },
      "rotation":{
         "x":-0.0579901077,
         "y":0.9334548,
         "z":0.306903929,
         "w":0.176378131
      }
   },
   "LeftHand":{
      "position":{
         "x":-0.213651448,
         "y":1.19429386,
         "z":0.2773032
      },
      "rotation":{
         "x":0.306908131,
         "y":0.176378921,
         "z":-0.05799081,
         "w":0.9334532
      }
   },
   "RightHand":{
      "position":{
         "x":0.361321181,
         "y":1.16164911,
         "z":0.09951973
      },
      "rotation":{
         "x":0.306908131,
         "y":0.176378921,
         "z":-0.05799081,
         "w":0.9334532
      }
   },
   "LeftEarAngle":330.445648,
   "RightEarAngle":326.061768
}
'''




import websocket
from websocket import create_connection
import sys

# Print received command-line arguments
print("Command-line arguments:", sys.argv)

# Check if an argument was provided, else use a default message
if len(sys.argv) > 1:
    test_command = sys.argv[1]
else:
    print("No command-line argument provided. Using default message")
    test_command = default_text

try:
    # Connect to the WebSocket server
    socket = create_connection("ws://localhost:35000")
    print(f"Connected to ws://localhost:35000, sending: {test_command}")

    # Send message
    socket.send(test_command)

    # Receive response
    #result = socket.recv()
    #print("Received:", result)

    # Close connection
    socket.close()
except Exception as e:
    print(f"WebSocket connection error: {e}")
