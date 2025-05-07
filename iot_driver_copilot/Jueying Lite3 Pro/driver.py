import os
import asyncio
import json
from fastapi import FastAPI, Request, Response, status
from fastapi.responses import StreamingResponse, JSONResponse
from pydantic import BaseModel
from starlette.concurrency import run_in_threadpool
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from nav2_msgs.srv import ManageLifecycleNodes
from std_msgs.msg import String

# Environment Variables
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
ROS_DOMAIN_ID = os.environ.get("ROS_DOMAIN_ID", "0")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8000"))

os.environ["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID

app = FastAPI()

# --------- ROS2 Client Node Management ---------
class RosClient(Node):
    def __init__(self):
        super().__init__('jueying_lite3pro_driver')
        self.imu_data = None
        self.imu_future = None
        self.imu_event = asyncio.Event()
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Localization trigger (could be a service or topic depending on robot setup)
        self.localization_client = self.create_client(Trigger, '/localization_trigger')
        # Mapping trigger (could be a service or topic depending on robot setup)
        self.mapping_client = self.create_client(Trigger, '/mapping_trigger')

    def imu_callback(self, msg):
        self.imu_data = {
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
        self.imu_event.set()

    async def get_imu_data(self, timeout=1.0):
        self.imu_event.clear()
        if self.imu_data is not None:
            return self.imu_data
        try:
            await asyncio.wait_for(self.imu_event.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            pass
        return self.imu_data

    async def trigger_localization(self):
        if not self.localization_client.wait_for_service(timeout_sec=2.0):
            return {"success": False, "message": "Localization service unavailable"}
        req = Trigger.Request()
        future = self.localization_client.call_async(req)
        await asyncio.wrap_future(future)
        resp = future.result()
        return {"success": resp.success, "message": resp.message}

    async def trigger_mapping(self):
        if not self.mapping_client.wait_for_service(timeout_sec=2.0):
            return {"success": False, "message": "Mapping service unavailable"}
        req = Trigger.Request()
        future = self.mapping_client.call_async(req)
        await asyncio.wrap_future(future)
        resp = future.result()
        return {"success": resp.success, "message": resp.message}

    def publish_cmd_vel(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.linear.z = linear_z
        twist.angular.x = angular_x
        twist.angular.y = angular_y
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        return {"success": True}

ros_client = None

# ------------- FastAPI Data Models ---------------
class MoveParams(BaseModel):
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0

# ----------- Startup & ROS2 Background -----------
@app.on_event("startup")
async def ros2_startup():
    global ros_client
    def ros2_spin():
        rclpy.init()
        global ros_client
        ros_client = RosClient()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(ros_client)
        executor.spin()
        ros_client.destroy_node()
        rclpy.shutdown()
    loop = asyncio.get_event_loop()
    loop.run_in_executor(None, ros2_spin)
    # Wait for node startup
    await asyncio.sleep(2)

# -------------- API Endpoints --------------------

@app.get("/imu", response_class=JSONResponse, tags=["Sensor"])
async def get_imu():
    """Fetch real-time IMU sensor data (orientation, acceleration)."""
    data = await ros_client.get_imu_data(timeout=1.5)
    if data is None:
        return JSONResponse({"error": "No IMU data received"}, status_code=status.HTTP_504_GATEWAY_TIMEOUT)
    return JSONResponse(data)

@app.post("/local", response_class=JSONResponse, tags=["Control"])
async def activate_localization():
    """Activate localization routine using sensor fusion."""
    result = await ros_client.trigger_localization()
    return JSONResponse(result)

@app.get("/move", response_class=JSONResponse, tags=["Motion"])
async def get_move():
    """Movement API information (for browser/CLI, shows usage)."""
    usage = {
        "description": "To move the robot, send a POST to this endpoint with JSON: "
                       '{"linear_x": float, "linear_y": float, "linear_z": float, '
                       '"angular_x": float, "angular_y": float, "angular_z": float}',
        "example": {
            "linear_x": 0.1,
            "linear_y": 0.0,
            "linear_z": 0.0,
            "angular_x": 0.0,
            "angular_y": 0.0,
            "angular_z": 0.5
        }
    }
    return JSONResponse(usage)

@app.post("/move", response_class=JSONResponse, tags=["Motion"])
async def post_move(params: MoveParams):
    """Send velocity commands to the robot."""
    result = await run_in_threadpool(
        ros_client.publish_cmd_vel,
        params.linear_x, params.linear_y, params.linear_z,
        params.angular_x, params.angular_y, params.angular_z
    )
    return JSONResponse(result)

@app.post("/map", response_class=JSONResponse, tags=["Mapping"])
async def start_mapping():
    """Initiate the mapping (SLAM) process."""
    result = await ros_client.trigger_mapping()
    return JSONResponse(result)

# --------------- Main Entrypoint -----------------
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host=SERVER_HOST,
        port=SERVER_PORT,
        reload=False
    )