import os
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

# Environment variable configuration
ROS_MASTER_URI = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
ROS_HOSTNAME = os.environ.get('ROS_HOSTNAME', 'localhost')
DEVICE_NAMESPACE = os.environ.get('DEVICE_NAMESPACE', '')
HTTP_HOST = os.environ.get('HTTP_HOST', '0.0.0.0')
HTTP_PORT = int(os.environ.get('HTTP_PORT', '8080'))

ODOM_TOPIC = os.environ.get('ODOM_TOPIC', '/odom')
IMU_TOPIC = os.environ.get('IMU_TOPIC', '/imu')
CMD_VEL_TOPIC = os.environ.get('CMD_VEL_TOPIC', '/cmd_vel')

# ROS initialization flag
ros_initialized = False

# Thread-safe latest messages
latest_odom = {'msg': None, 'stamp': 0}
latest_imu = {'msg': None, 'stamp': 0}
latest_cmd_vel = {'msg': None, 'stamp': 0}

def ros_init():
    global ros_initialized
    if not ros_initialized:
        import sys
        import rospy
        # Set ROS environment variables (for ROS_MASTER_URI, ROS_HOSTNAME)
        os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
        os.environ['ROS_HOSTNAME'] = ROS_HOSTNAME
        rospy.init_node('agilex_device_http_driver', anonymous=True, disable_signals=True)
        ros_initialized = True

def odom_callback(msg):
    latest_odom['msg'] = msg
    latest_odom['stamp'] = time.time()

def imu_callback(msg):
    latest_imu['msg'] = msg
    latest_imu['stamp'] = time.time()

def cmd_vel_callback(msg):
    latest_cmd_vel['msg'] = msg
    latest_cmd_vel['stamp'] = time.time()

def ros_spin_thread():
    # Ensure ROS is initialized
    ros_init()
    rospy.Subscriber(ODOM_TOPIC, Odometry, odom_callback)
    rospy.Subscriber(IMU_TOPIC, Imu, imu_callback)
    rospy.Subscriber(CMD_VEL_TOPIC, Twist, cmd_vel_callback)
    rospy.spin()

def ros_msg_to_dict(msg):
    # Recursively convert ROS messages to Python dicts for JSON serialization
    if hasattr(msg, '__slots__'):
        result = {}
        for slot in msg.__slots__:
            value = getattr(msg, slot)
            result[slot] = ros_msg_to_dict(value)
        return result
    elif isinstance(msg, (list, tuple)):
        return [ros_msg_to_dict(x) for x in msg]
    else:
        return msg

class AgileXHTTPRequestHandler(BaseHTTPRequestHandler):
    def _send_json(self, data, status=200):
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode('utf-8'))

    def do_GET(self):
        if self.path == '/odom':
            if latest_odom['msg'] is not None:
                self._send_json(ros_msg_to_dict(latest_odom['msg']))
            else:
                self._send_json({'error': 'No odometry data received yet'}, status=503)
        elif self.path == '/imu':
            if latest_imu['msg'] is not None:
                self._send_json(ros_msg_to_dict(latest_imu['msg']))
            else:
                self._send_json({'error': 'No IMU data received yet'}, status=503)
        elif self.path == '/cmd_vel':
            if latest_cmd_vel['msg'] is not None:
                self._send_json(ros_msg_to_dict(latest_cmd_vel['msg']))
            else:
                self._send_json({'error': 'No velocity command received yet'}, status=503)
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b'Not Found')

    def do_POST(self):
        if self.path == '/cmd_vel':
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            try:
                data = json.loads(body.decode('utf-8'))
                # Construct Twist message from JSON
                twist = Twist()
                for part in ('linear', 'angular'):
                    if part in data:
                        sub = getattr(twist, part)
                        for axis in ('x', 'y', 'z'):
                            if axis in data[part]:
                                setattr(sub, axis, float(data[part][axis]))
                ros_init()
                pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
                # Wait for publisher to be registered
                timeout = time.time() + 2.0
                while pub.get_num_connections() < 1 and time.time() < timeout:
                    time.sleep(0.05)
                pub.publish(twist)
                self._send_json({'status': 'Command sent'})
            except Exception as e:
                self._send_json({'error': str(e)}, status=400)
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b'Not Found')

def main():
    # Start ROS spin in a background thread
    ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
    ros_thread.start()
    # Start HTTP server
    server = HTTPServer((HTTP_HOST, HTTP_PORT), AgileXHTTPRequestHandler)
    print(f"AgileX Device HTTP Driver running at http://{HTTP_HOST}:{HTTP_PORT}")
    server.serve_forever()

if __name__ == '__main__':
    main()