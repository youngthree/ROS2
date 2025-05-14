import os
import asyncio
import json
from typing import Optional
from fastapi import FastAPI, Request, Response, HTTPException
from fastapi.responses import StreamingResponse, JSONResponse
from pydantic import BaseModel
import uvicorn

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool

# Environment variables
DEVICE_IP = os.environ.get("DEVICE_IP", "127.0.0.1")
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", 8000))

# ROS Topics/Services (can also be made configurable)
TOPIC_CMD_VEL = os.environ.get("TOPIC_CMD_VEL", "/cmd_vel")
SERVICE_MAPPING = os.environ.get("SERVICE_MAPPING", "/mapping")
SERVICE_NAV = os.environ.get("SERVICE_NAV", "/navigation")
SERVICE_LOC = os.environ.get("SERVICE_LOC", "/localization")
SERVICE_TASK = os.environ.get("SERVICE_TASK", "/task_exec")
SERVICE_WP = os.environ.get("SERVICE_WP", "/waypoint_record")

# -------- ROS2 Async Bridge --------

class Ros2Bridge(Node):
    def __init__(self):
        super().__init__("jueying_http_bridge")
        self.cmd_vel_pub = self.create_publisher(Twist, TOPIC_CMD_VEL, 10)
        self.cli_mapping = self.create_client(SetBool, SERVICE_MAPPING)
        self.cli_nav = self.create_client(SetBool, SERVICE_NAV)
        self.cli_loc = self.create_client(SetBool, SERVICE_LOC)
        self.cli_task = self.create_client(Trigger, SERVICE_TASK)
        self.cli_wp = self.create_client(Trigger, SERVICE_WP)

    async def send_cmd_vel(self, linear_x: float, linear_y: float, linear_z: float, angular_x: float, angular_y: float, angular_z: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = linear_z
        msg.angular.x = angular_x
        msg.angular.y = angular_y
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
        return {"status": "ok"}

    async def call_setbool(self, client, action: str):
        if not client.service_is_ready():
            await self.wait_for_service(client)
        req = SetBool.Request()
        if action.lower() == "start":
            req.data = True
        elif action.lower() == "stop":
            req.data = False
        else:
            return {"error": "Invalid action"}
        future = client.call_async(req)
        while not future.done():
            await asyncio.sleep(0.001)
        resp = future.result()
        return {"success": resp.success, "message": resp.message}

    async def call_trigger(self, client):
        if not client.service_is_ready():
            await self.wait_for_service(client)
        req = Trigger.Request()
        future = client.call_async(req)
        while not future.done():
            await asyncio.sleep(0.001)
        resp = future.result()
        return {"success": resp.success, "message": resp.message}

    async def wait_for_service(self, client):
        while not client.service_is_ready():
            await asyncio.sleep(0.1)

# ---------- FastAPI HTTP Server ----------

app = FastAPI()
ros2_bridge: Optional[Ros2Bridge] = None

# Models
class MapAction(BaseModel):
    action: str

class NavAction(BaseModel):
    action: str

class LocAction(BaseModel):
    action: str

class VelCmd(BaseModel):
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0

class TaskRequest(BaseModel):
    task: Optional[str] = None

@app.on_event("startup")
async def on_startup():
    global ros2_bridge
    rclpy.init(args=None)
    ros2_bridge = Ros2Bridge()

@app.on_event("shutdown")
async def on_shutdown():
    global ros2_bridge
    if ros2_bridge is not None:
        ros2_bridge.destroy_node()
        rclpy.shutdown()

@app.post("/map")
async def control_mapping(map_action: MapAction):
    global ros2_bridge
    if ros2_bridge is None:
        raise HTTPException(status_code=503, detail="ROS2 bridge not initialized")
    resp = await ros2_bridge.call_setbool(ros2_bridge.cli_mapping, map_action.action)
    return JSONResponse(content=resp)

@app.post("/velcmd")
async def send_velocity_command(cmd: VelCmd):
    global ros2_bridge
    if ros2_bridge is None:
        raise HTTPException(status_code=503, detail="ROS2 bridge not initialized")
    resp = await ros2_bridge.send_cmd_vel(
        cmd.linear_x, cmd.linear_y, cmd.linear_z,
        cmd.angular_x, cmd.angular_y, cmd.angular_z
    )
    return JSONResponse(content=resp)

@app.post("/nav")
async def control_navigation(nav_action: NavAction):
    global ros2_bridge
    if ros2_bridge is None:
        raise HTTPException(status_code=503, detail="ROS2 bridge not initialized")
    resp = await ros2_bridge.call_setbool(ros2_bridge.cli_nav, nav_action.action)
    return JSONResponse(content=resp)

@app.post("/loc")
async def control_localization(loc_action: LocAction):
    global ros2_bridge
    if ros2_bridge is None:
        raise HTTPException(status_code=503, detail="ROS2 bridge not initialized")
    resp = await ros2_bridge.call_setbool(ros2_bridge.cli_loc, loc_action.action)
    return JSONResponse(content=resp)

@app.post("/task")
async def execute_task(task_req: TaskRequest):
    global ros2_bridge
    if ros2_bridge is None:
        raise HTTPException(status_code=503, detail="ROS2 bridge not initialized")
    resp = await ros2_bridge.call_trigger(ros2_bridge.cli_task)
    return JSONResponse(content=resp)

@app.post("/wp")
async def record_waypoint():
    global ros2_bridge
    if ros2_bridge is None:
        raise HTTPException(status_code=503, detail="ROS2 bridge not initialized")
    resp = await ros2_bridge.call_trigger(ros2_bridge.cli_wp)
    return JSONResponse(content=resp)

# --------- Run the HTTP Server ---------

def main():
    uvicorn.run(
        "main:app",
        host=SERVER_HOST,
        port=SERVER_PORT,
        reload=False,
        log_level="info"
    )

if __name__ == "__main__":
    main()