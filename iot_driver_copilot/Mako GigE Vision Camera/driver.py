import os
import io
import threading
import time
from typing import Optional, Dict, Any

from fastapi import FastAPI, Request, Response, UploadFile, File, Form, status
from fastapi.responses import StreamingResponse, JSONResponse, PlainTextResponse
from pydantic import BaseModel
import uvicorn

import numpy as np
from PIL import Image

# =================== ENV VARS ===================

CAMERA_IP = os.environ.get("CAMERA_IP", "192.168.1.2")
CAMERA_PORT = int(os.environ.get("CAMERA_PORT", "3956"))  # Standard GigE Vision port
SERVER_HOST = os.environ.get("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("SERVER_PORT", "8080"))

# =================== MOCKED CAMERA INTERFACE ===================
# For the purpose of this driver, a mock camera interface is provided.
# In real deployment, replace CameraInterface with the actual GenICam/GigE Vision communication logic.

class CameraStatus(BaseModel):
    exposure: float
    gain: float
    roi: Dict[str, int]
    temperature: float
    trigger_state: str
    pixel_format: str

class CameraInterface:
    def __init__(self, ip: str, port: int):
        self.ip = ip
        self.port = port
        self._exposure = 10000.0  # in us
        self._gain = 1.0
        self._roi = {"x": 0, "y": 0, "width": 640, "height": 480}
        self._temperature = 36.5
        self._trigger_state = "Idle"
        self._pixel_format = "Mono8"
        self._last_frame = None
        self.lock = threading.Lock()

    def set_network(self, config: Dict[str, Any]):
        # Normally, set IP/Gateway/Netmask via GenICam
        # Here, just pretend to set
        with self.lock:
            if "ip" in config:
                self.ip = config["ip"]
            if "port" in config:
                self.port = config["port"]
        return True

    def set_exposure(self, exposure: float, auto: Optional[bool] = False):
        with self.lock:
            self._exposure = float(exposure)
            # auto flag ignored in mock
        return True

    def set_gain(self, gain: float, auto: Optional[bool] = False):
        with self.lock:
            self._gain = float(gain)
        return True

    def set_roi(self, x: int, y: int, width: int, height: int):
        with self.lock:
            self._roi = {"x": x, "y": y, "width": width, "height": height}
        return True

    def get_status(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "ip": self.ip,
                "port": self.port,
                "exposure": self._exposure,
                "gain": self._gain,
                "roi": self._roi,
                "temperature": self._temperature,
                "trigger_state": self._trigger_state,
                "pixel_format": self._pixel_format,
                "uptime": int(time.time())
            }

    def trigger_capture(self) -> np.ndarray:
        # Simulate image capture (a gray image with a moving square)
        with self.lock:
            w, h = self._roi["width"], self._roi["height"]
            img = np.full((h, w), 120, dtype=np.uint8)
            t = int(time.time() * 2) % min(w, h)
            img[t:t+32, t:t+32] = 255
            self._last_frame = img
            self._trigger_state = "Triggered"
        time.sleep(0.1)
        with self.lock:
            self._trigger_state = "Idle"
        return img

    def get_last_frame(self) -> Optional[np.ndarray]:
        with self.lock:
            return self._last_frame

camera = CameraInterface(CAMERA_IP, CAMERA_PORT)

# =================== FASTAPI APP ===================

app = FastAPI(title="Mako GigE Vision Camera HTTP Driver")

# --------- Network Configuration (PUT /netcfg) ---------

class NetCfgRequest(BaseModel):
    ip: str
    port: int

@app.put("/netcfg")
async def put_netcfg(cfg: NetCfgRequest):
    success = camera.set_network(cfg.dict())
    return {"success": success, "ip": camera.ip, "port": camera.port}

# --------- Exposure Setting (PUT /expose) ---------

class ExposureRequest(BaseModel):
    exposure: float
    auto: Optional[bool] = False

@app.put("/expose")
async def put_expose(cfg: ExposureRequest):
    camera.set_exposure(cfg.exposure, cfg.auto)
    return {"success": True, "exposure": camera._exposure}

# --------- Gain Setting (PUT /gain) ---------

class GainRequest(BaseModel):
    gain: float
    auto: Optional[bool] = False

@app.put("/gain")
async def put_gain(cfg: GainRequest):
    camera.set_gain(cfg.gain, cfg.auto)
    return {"success": True, "gain": camera._gain}

# --------- ROI Setting (PUT /roi) ---------

class ROIRequest(BaseModel):
    x: int
    y: int
    width: int
    height: int

@app.put("/roi")
async def put_roi(cfg: ROIRequest):
    camera.set_roi(cfg.x, cfg.y, cfg.width, cfg.height)
    return {"success": True, "roi": camera._roi}

# --------- Status (GET /status) ---------

@app.get("/status")
async def get_status():
    status = camera.get_status()
    return status

# --------- Capture (POST /capture) ---------
# Returns latest image as JPEG (browser/CLI compatible)

@app.post("/capture")
async def post_capture():
    arr = camera.trigger_capture()
    img = Image.fromarray(arr)
    buf = io.BytesIO()
    img.save(buf, format="JPEG")
    buf.seek(0)
    headers = {
        "Content-Type": "image/jpeg",
        "Content-Disposition": "inline; filename=capture.jpg"
    }
    return Response(content=buf.getvalue(), headers=headers, media_type="image/jpeg")

# --------- MJPEG Streaming (GET /stream) ---------
# Live MJPEG stream for browser/CLI

def mjpeg_stream_generator():
    while True:
        frame = camera.trigger_capture()
        img = Image.fromarray(frame)
        buf = io.BytesIO()
        img.save(buf, format="JPEG")
        jpg = buf.getvalue()
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
        time.sleep(0.1)  # ~10 FPS

@app.get("/stream")
async def stream():
    return StreamingResponse(mjpeg_stream_generator(),
                             media_type="multipart/x-mixed-replace; boundary=frame")

# =================== MAIN ===================

if __name__ == "__main__":
    uvicorn.run(
        "main:app",
        host=SERVER_HOST,
        port=SERVER_PORT,
        reload=False,
        workers=1
    )