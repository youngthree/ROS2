import os
import asyncio
import json
from typing import Dict, Any
from fastapi import FastAPI, HTTPException, Request, Response, status
from fastapi.responses import StreamingResponse, JSONResponse
from pydantic import BaseModel
import httpx
import uvicorn

# ROS2 Python Client Libraries
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

# RTSP --> HTTP MJPEG Stream
import cv2
import threading

# --- Environment Variables ---
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8000"))
ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID", "0")
ROS_MIDDLEWARE_PORT = os.environ.get("ROS_MIDDLEWARE_PORT", "11811")
RTSP_URL = os.environ.get("RTSP_URL", f"rtsp://{DEVICE_IP}/video")
RTSP_HTTP_STREAM_PORT = int(os.environ.get("RTSP_HTTP_STREAM_PORT", "8080"))
UDP_CONTROL_PORT = int(os.environ.get("UDP_CONTROL_PORT", "9001"))

os.environ["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID

# --- FastAPI App ---
app = FastAPI(title="Jueying Lite3 Pro Device Driver")

# --- ROS2 Client Node for Sensor Data ---
class SensorDataStore:
    def __init__(self):
        self.latest = {
            "leg_odom": None,
            "imu": None,
            "joint_states": None,
            "ultrasound_distance": None,
            "leg_odom2": None
        }
        self.lock = asyncio.Lock()

    async def set(self, key, value):
        async with self.lock:
            self.latest[key] = value

    async def get_all(self):
        async with self.lock:
            return dict(self.latest)

sensor_store = SensorDataStore()

class ROS2Bridge(Node):
    def __init__(self):
        super().__init__("jueying_driver_bridge")
        self.create_subscription(Odometry, "/leg_odom", self.leg_odom_callback, 10)
        self.create_subscription(Odometry, "/leg_odom2", self.leg_odom2_callback, 10)
        self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)
        self.create_subscription(Float64, "/us_publisher/ultrasound_distance", self.ultrasound_distance_callback, 10)

    def leg_odom_callback(self, msg: Odometry):
        data = {
            "pose": {
                "position": {
                    "x": msg.pose.pose.position.x,
                    "y": msg.pose.pose.position.y,
                    "z": msg.pose.pose.position.z
                },
                "orientation": {
                    "x": msg.pose.pose.orientation.x,
                    "y": msg.pose.pose.orientation.y,
                    "z": msg.pose.pose.orientation.z,
                    "w": msg.pose.pose.orientation.w,
                }
            },
            "twist": {
                "linear": {
                    "x": msg.twist.twist.linear.x,
                    "y": msg.twist.twist.linear.y,
                    "z": msg.twist.twist.linear.z,
                },
                "angular": {
                    "x": msg.twist.twist.angular.x,
                    "y": msg.twist.twist.angular.y,
                    "z": msg.twist.twist.angular.z,
                }
            }
        }
        asyncio.create_task(sensor_store.set("leg_odom", data))

    def leg_odom2_callback(self, msg: Odometry):
        data = {
            "pose": {
                "position": {
                    "x": msg.pose.pose.position.x,
                    "y": msg.pose.pose.position.y,
                    "z": msg.pose.pose.position.z
                },
                "orientation": {
                    "x": msg.pose.pose.orientation.x,
                    "y": msg.pose.pose.orientation.y,
                    "z": msg.pose.pose.orientation.z,
                    "w": msg.pose.pose.orientation.w,
                }
            },
            "twist": {
                "linear": {
                    "x": msg.twist.twist.linear.x,
                    "y": msg.twist.twist.linear.y,
                    "z": msg.twist.twist.linear.z,
                },
                "angular": {
                    "x": msg.twist.twist.angular.x,
                    "y": msg.twist.twist.angular.y,
                    "z": msg.twist.twist.angular.z,
                }
            }
        }
        asyncio.create_task(sensor_store.set("leg_odom2", data))

    def imu_callback(self, msg: Imu):
        data = {
            "orientation": {
                "x": msg.orientation.x,
                "y": msg.orientation.y,
                "z": msg.orientation.z,
                "w": msg.orientation.w
            },
            "angular_velocity": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z
            },
            "linear_acceleration": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z
            }
        }
        asyncio.create_task(sensor_store.set("imu", data))

    def joint_states_callback(self, msg: JointState):
        data = {
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort)
        }
        asyncio.create_task(sensor_store.set("joint_states", data))

    def ultrasound_distance_callback(self, msg: Float64):
        data = {
            "distance": msg.data
        }
        asyncio.create_task(sensor_store.set("ultrasound_distance", data))

def ros_spin():
    rclpy.init()
    node = ROS2Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

ros_thread = threading.Thread(target=ros_spin, daemon=True)
ros_thread.start()

# --- MJPEG Streaming Proxy for RTSP Video ---
def mjpeg_generator(rtsp_url):
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        raise RuntimeError("Cannot open RTSP stream")
    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                continue
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
    finally:
        cap.release()

@app.get("/video")
def video_stream():
    return StreamingResponse(mjpeg_generator(RTSP_URL),
                             media_type="multipart/x-mixed-replace; boundary=frame")

# --- API Models ---
class MoveRequest(BaseModel):
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0

class CmdRequest(BaseModel):
    command: str
    parameters: Dict[str, Any] = {}

# --- Sensor Data API ---
@app.get("/sensors", response_class=JSONResponse)
async def get_sensors():
    return await sensor_store.get_all()

# --- Movement Command API ---
@app.post("/move", status_code=200)
async def move(req: MoveRequest):
    # Publish Twist to /cmd_vel
    class CmdVelPublisher(Node):
        def __init__(self):
            super().__init__('cmd_vel_publisher_driver')
            self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
            msg = Twist()
            msg.linear.x = req.linear_x
            msg.linear.y = req.linear_y
            msg.linear.z = req.linear_z
            msg.angular.x = req.angular_x
            msg.angular.y = req.angular_y
            msg.angular.z = req.angular_z
            self.publisher_.publish(msg)
    def publish_cmd():
        rclpy.init()
        pub_node = CmdVelPublisher()
        rclpy.spin_once(pub_node, timeout_sec=0.5)
        pub_node.destroy_node()
        rclpy.shutdown()
    t = threading.Thread(target=publish_cmd)
    t.start()
    t.join()
    return {"status": "Command sent"}

# --- Command Execution API ---
@app.post("/cmd", status_code=200)
async def cmd(req: CmdRequest):
    # Example: Send UDP command or trigger navigation scripts
    import socket
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.settimeout(2)
    if req.command == "SimpleCMD":
        try:
            cmd_code = int(req.parameters.get("cmd_code", 0))
            cmd_value = int(req.parameters.get("cmd_value", 0))
            cmd_type = int(req.parameters.get("type", 0))
            payload = cmd_code.to_bytes(4, "little", signed=True) + \
                      cmd_value.to_bytes(4, "little", signed=True) + \
                      cmd_type.to_bytes(4, "little", signed=True)
            udp_sock.sendto(payload, (DEVICE_IP, UDP_CONTROL_PORT))
        finally:
            udp_sock.close()
        return {"status": "SimpleCMD sent"}
    elif req.command in ["start_nav", "start_lslidar", "start_slam"]:
        # Simulating command trigger: in a real device, this would signal a ROS service or topic
        return {"status": f"{req.command} triggered (placeholder)"}
    else:
        udp_sock.close()
        raise HTTPException(status_code=400, detail="Unknown command")
        
# --- Main Entrypoint ---
if __name__ == "__main__":
    uvicorn.run(app, host=SERVER_HOST, port=SERVER_PORT)