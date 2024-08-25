#!/bin/env python
import asyncio
import websockets
import json
from http.server import SimpleHTTPRequestHandler
from socketserver import TCPServer
import rospy

from frost_msgs.msg import PowertrainCommand

pub = None

async def echo(websocket, path):

    msg = PowertrainCommand()
    async for message in websocket:
        data = json.loads(message)
        print(f"Received data: {data}")

        msg.mode = 1
        msg.enabled = int(data['motorsOn'])
        msg.trans_vel = float(data['translationalSpeed'])
        msg.rot_vel = float(data['rotationalSpeed'])

        pub.publish(msg)

# Simple HTTP Server to serve static files
class HTTPServer(TCPServer):
    allow_reuse_address = True

def start_http_server():
    handler = SimpleHTTPRequestHandler
    httpd = HTTPServer(("0.0.0.0", 8000), handler)
    print("HTTP server is running at http://localhost:8000")
    httpd.serve_forever()

async def main():
    global pub

    pub = rospy.Publisher('chatter', PowertrainCommand, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    # Start HTTP server in a new thread
    from threading import Thread
    http_server_thread = Thread(target=start_http_server, daemon=True)
    http_server_thread.start()

    async with websockets.serve(echo, "0.0.0.0", 8765):
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    asyncio.run(main())
