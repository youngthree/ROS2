import os
import asyncio
import json
from fastapi import FastAPI, Request, Response, status
from fastapi.responses import StreamingResponse, JSONResponse, PlainTextResponse
from fastapi.middleware.cors import CORSMiddleware
from starlette.background import BackgroundTask
from typing import AsyncGenerator
import uvicorn
import struct
import socket

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import Twist
except ImportError:
    rclpy = None

# Configuration from environment variables
SERVER_HOST = os.getenv('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.getenv('SERVER_PORT', '8080'))
ROS_DOMAIN_ID = os.getenv("ROS_DOMAIN_ID")
ROS_IMU_TOPIC = os.getenv('ROS_IMU_TOPIC', '/imu/data')
ROS_CMD_VEL_TOPIC = os.getenv('ROS_CMD_VEL_TOPIC', '/cmd_vel')
ROS_NAMESPACE = os.getenv('ROS_NAMESPACE', '')
UDP_IMU_HOST = os.getenv('UDP_IMU_HOST', None)
UDP_IMU_PORT = int(os.getenv('UDP_IMU_PORT', '0')) if os.getenv('UDP_IMU_PORT') else None

# FastAPI app
app = FastAPI(
    title="Jueying Lite3 Pro HTTP Device Driver",
    description="HTTP Server driver for Jueying Lite3 Pro Robot, serving IMU data and control endpoints.",
    version="1.0.0"
)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global ROS2 Node objects
class ImuListener(Node):
    def __init__(self, topic_name):
        super().__init__('imu_listener')
        self.imu_data = None
        self.sub = self.create_subscription(
            Imu,
            topic_name,
            self.imu_callback,
            10
        )

    def imu_callback(self, msg: Imu):
        # Save the latest IMU reading
        self.imu_data = msg

class CmdVelPublisher(Node):
    def __init__(self, topic_name):
        super().__init__('cmd_vel_publisher')
        self.publisher = self.create_publisher(Twist, topic_name, 10)

    def publish(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = linear_z
        msg.angular.x = angular_x
        msg.angular.y = angular_y
        msg.angular.z = angular_z
        self.publisher.publish(msg)

ros_context = {
    "imu_listener": None,
    "cmd_vel_publisher": None,
    "ros_thread": None,
    "ros_inited": False
}

def start_ros2():
    if rclpy is None:
        return
    if ros_context["ros_inited"]:
        return
    if ROS_DOMAIN_ID is not None:
        os.environ["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID
    rclpy.init(args=None)
    imu_listener = ImuListener(ROS_IMU_TOPIC)
    cmd_vel_publisher = CmdVelPublisher(ROS_CMD_VEL_TOPIC)
    ros_context["imu_listener"] = imu_listener
    ros_context["cmd_vel_publisher"] = cmd_vel_publisher
    ros_context["ros_inited"] = True

    def spin_nodes():
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(imu_listener)
        executor.add_node(cmd_vel_publisher)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            imu_listener.destroy_node()
            cmd_vel_publisher.destroy_node()
            rclpy.shutdown()
    thread = asyncio.get_event_loop().run_in_executor(None, spin_nodes)
    ros_context["ros_thread"] = thread

@app.on_event("startup")
async def startup_event():
    # Start ROS2 node if available
    if rclpy is not None:
        start_ros2()

def imu_msg_to_dict(msg: Imu):
    if msg is None:
        return None
    return {
        "header": {
            "stamp": {
                "sec": msg.header.stamp.sec,
                "nanosec": msg.header.stamp.nanosec
            },
            "frame_id": msg.header.frame_id
        },
        "orientation": {
            "x": msg.orientation.x,
            "y": msg.orientation.y,
            "z": msg.orientation.z,
            "w": msg.orientation.w
        },
        "orientation_covariance": list(msg.orientation_covariance),
        "angular_velocity": {
            "x": msg.angular_velocity.x,
            "y": msg.angular_velocity.y,
            "z": msg.angular_velocity.z
        },
        "angular_velocity_covariance": list(msg.angular_velocity_covariance),
        "linear_acceleration": {
            "x": msg.linear_acceleration.x,
            "y": msg.linear_acceleration.y,
            "z": msg.linear_acceleration.z
        },
        "linear_acceleration_covariance": list(msg.linear_acceleration_covariance)
    }

async def get_imu_data_from_ros():
    imu_listener = ros_context["imu_listener"]
    if imu_listener is None:
        return None
    # Wait a little if no data yet
    for _ in range(10):
        if imu_listener.imu_data is not None:
            return imu_msg_to_dict(imu_listener.imu_data)
        await asyncio.sleep(0.05)
    return None

async def get_imu_data_from_udp():
    if not UDP_IMU_HOST or not UDP_IMU_PORT:
        return None
    # Example: expecting a struct with float32[6] (accel x,y,z, gyro x,y,z)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(1.0)
            sock.sendto(b'imu', (UDP_IMU_HOST, UDP_IMU_PORT))
            data, _ = sock.recvfrom(1024)
            if len(data) >= 24:
                values = struct.unpack('!6f', data[:24])
                return {
                    "acceleration": {"x": values[0], "y": values[1], "z": values[2]},
                    "gyroscope": {"x": values[3], "y": values[4], "z": values[5]}
                }
            return {"raw": data.hex()}
    except Exception as e:
        return {"error": str(e)}

@app.get("/imu", response_class=JSONResponse)
async def get_imu():
    """
    Fetch real-time sensor data from the IMU.
    """
    if rclpy is not None and ros_context["imu_listener"] is not None:
        data = await get_imu_data_from_ros()
        if data:
            return data
    if UDP_IMU_HOST and UDP_IMU_PORT:
        data = await get_imu_data_from_udp()
        if data:
            return data
    return JSONResponse({"error": "IMU data source unavailable"}, status_code=503)

@app.post("/local", response_class=PlainTextResponse)
async def activate_localization(request: Request):
    """
    Activate the localization routine to update the robot's pose.
    """
    # For demonstration, pretend to activate localization
    # In practice, this would send a service call or topic message in ROS
    return PlainTextResponse("Localization routine triggered", status_code=status.HTTP_200_OK)

@app.get("/move", response_class=PlainTextResponse)
async def move():
    """
    Move API (demonstrative).
    """
    # Example: Just return a placeholder
    return PlainTextResponse("Move API called", status_code=status.HTTP_200_OK)

@app.post("/map", response_class=PlainTextResponse)
async def start_mapping(request: Request):
    """
    Initiate the mapping process by triggering SLAM routines.
    """
    # For demonstration, pretend to start mapping
    # In practice, this would call a ROS service or publish a message
    return PlainTextResponse("SLAM mapping started", status_code=status.HTTP_200_OK)

if __name__ == "__main__":
    uvicorn.run("main:app", host=SERVER_HOST, port=SERVER_PORT, reload=False)
