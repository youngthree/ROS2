import os
import json
import threading
from flask import Flask, request, Response, jsonify
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import ros_numpy
import numpy as np

# Env Vars
ROS_MASTER_URI = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
ROS_IP = os.environ.get("ROS_IP", "127.0.0.1")
ROBOT_HOST = os.environ.get("DRIVER_HOST", "0.0.0.0")
ROBOT_PORT = int(os.environ.get("DRIVER_PORT", "8080"))
LIDAR_TOPIC = os.environ.get("LIDAR_TOPIC", "/points_raw")
ODOM_TOPIC = os.environ.get("ODOM_TOPIC", "/odom")
MAP_TOPIC = os.environ.get("MAP_TOPIC", "/map")
POSE_TOPIC = os.environ.get("POSE_TOPIC", "/base_link")
SLAM_SERVICE = os.environ.get("SLAM_SERVICE", "/save_map")
DRIVE_TOPIC = os.environ.get("DRIVE_TOPIC", "/cmd_vel")
GOAL_TOPIC = os.environ.get("GOAL_TOPIC", "/move_base_simple/goal")

# Flask App
app = Flask(__name__)

# ROS Node Thread
def ros_spin():
    rospy.spin()

def init_ros():
    if not rospy.core.is_initialized():
        rospy.init_node("scout_driver_http", anonymous=True, disable_signals=True)

# LIDAR - PointCloud2 or LaserScan to JSON
def pointcloud2_to_dict(msg):
    pc = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    points = np.zeros((pc.shape[0], 3))
    points[:, 0] = pc['x']
    points[:, 1] = pc['y']
    points[:, 2] = pc['z']
    return {
        "header": {
            "stamp": msg.header.stamp.to_sec(),
            "frame_id": msg.header.frame_id
        },
        "points": points.tolist()
    }

def laserscan_to_dict(msg):
    ranges = np.array(msg.ranges)
    return {
        "header": {
            "stamp": msg.header.stamp.to_sec(),
            "frame_id": msg.header.frame_id
        },
        "angle_min": msg.angle_min,
        "angle_max": msg.angle_max,
        "angle_increment": msg.angle_increment,
        "time_increment": msg.time_increment,
        "scan_time": msg.scan_time,
        "range_min": msg.range_min,
        "range_max": msg.range_max,
        "ranges": ranges.tolist(),
        "intensities": list(msg.intensities)
    }

@app.route("/lidar", methods=["GET"])
def get_lidar():
    init_ros()
    msg_type, topic = None, None
    pointcloud_msg = [None]
    def pc_cb(msg):
        pointcloud_msg[0] = msg
        rospy.signal_shutdown("got lidar msg")
    try:
        msg_type = rospy.get_param("~lidar_msg_type", "PointCloud2")
    except:
        msg_type = "PointCloud2"
    topic = LIDAR_TOPIC
    if msg_type == "PointCloud2":
        sub = rospy.Subscriber(topic, PointCloud2, pc_cb)
    else:
        sub = rospy.Subscriber(topic, LaserScan, pc_cb)
    try:
        rospy.sleep(0.5)
        while not rospy.is_shutdown() and pointcloud_msg[0] is None:
            rospy.sleep(0.05)
    except:
        pass
    sub.unregister()
    if pointcloud_msg[0] is None:
        return jsonify({"error": "No LIDAR data available"}), 504
    if msg_type == "PointCloud2":
        data = pointcloud2_to_dict(pointcloud_msg[0])
    else:
        data = laserscan_to_dict(pointcloud_msg[0])
    return jsonify(data)

@app.route("/pose", methods=["GET"])
def get_pose():
    init_ros()
    odom_msg = [None]
    def cb(msg):
        odom_msg[0] = msg
        rospy.signal_shutdown("got pose msg")
    sub = rospy.Subscriber(ODOM_TOPIC, Odometry, cb)
    try:
        rospy.sleep(0.5)
        while not rospy.is_shutdown() and odom_msg[0] is None:
            rospy.sleep(0.05)
    except:
        pass
    sub.unregister()
    if odom_msg[0] is None:
        return jsonify({"error": "No odometry data available"}), 504
    msg = odom_msg[0]
    data = {
        "header": {
            "stamp": msg.header.stamp.to_sec(),
            "frame_id": msg.header.frame_id
        },
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
                "w": msg.pose.pose.orientation.w
            }
        },
        "twist": {
            "linear": {
                "x": msg.twist.twist.linear.x,
                "y": msg.twist.twist.linear.y,
                "z": msg.twist.twist.linear.z
            },
            "angular": {
                "x": msg.twist.twist.angular.x,
                "y": msg.twist.twist.angular.y,
                "z": msg.twist.twist.angular.z
            }
        }
    }
    return jsonify(data)

@app.route("/map", methods=["GET"])
def get_map():
    init_ros()
    map_msg = [None]
    def cb(msg):
        map_msg[0] = msg
        rospy.signal_shutdown("got map msg")
    sub = rospy.Subscriber(MAP_TOPIC, OccupancyGrid, cb)
    try:
        rospy.sleep(0.5)
        while not rospy.is_shutdown() and map_msg[0] is None:
            rospy.sleep(0.05)
    except:
        pass
    sub.unregister()
    if map_msg[0] is None:
        return jsonify({"error": "No map data available"}), 504
    msg = map_msg[0]
    data = {
        "header": {
            "stamp": msg.header.stamp.to_sec(),
            "frame_id": msg.header.frame_id
        },
        "info": {
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": msg.info.resolution,
            "origin": {
                "position": {
                    "x": msg.info.origin.position.x,
                    "y": msg.info.origin.position.y,
                    "z": msg.info.origin.position.z
                },
                "orientation": {
                    "x": msg.info.origin.orientation.x,
                    "y": msg.info.origin.orientation.y,
                    "z": msg.info.origin.orientation.z,
                    "w": msg.info.origin.orientation.w
                }
            }
        },
        "data": list(msg.data)
    }
    return jsonify(data)

@app.route("/drive", methods=["POST"])
def post_drive():
    init_ros()
    body = request.get_json(force=True)
    import geometry_msgs.msg
    twist = geometry_msgs.msg.Twist()
    twist.linear.x = float(body.get("linear_x", 0.0))
    twist.linear.y = float(body.get("linear_y", 0.0))
    twist.linear.z = float(body.get("linear_z", 0.0))
    twist.angular.x = float(body.get("angular_x", 0.0))
    twist.angular.y = float(body.get("angular_y", 0.0))
    twist.angular.z = float(body.get("angular_z", 0.0))
    pub = rospy.Publisher(DRIVE_TOPIC, geometry_msgs.msg.Twist, queue_size=1)
    r = rospy.Rate(10)
    for _ in range(5):
        pub.publish(twist)
        r.sleep()
    return jsonify({"status": "ok"})

@app.route("/goal", methods=["POST"])
def post_goal():
    init_ros()
    body = request.get_json(force=True)
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = body.get("frame_id", "map")
    pose.pose.position.x = float(body.get("x", 0.0))
    pose.pose.position.y = float(body.get("y", 0.0))
    pose.pose.position.z = float(body.get("z", 0.0))
    pose.pose.orientation.x = float(body.get("qx", 0.0))
    pose.pose.orientation.y = float(body.get("qy", 0.0))
    pose.pose.orientation.z = float(body.get("qz", 0.0))
    pose.pose.orientation.w = float(body.get("qw", 1.0))
    pub = rospy.Publisher(GOAL_TOPIC, PoseStamped, queue_size=1)
    r = rospy.Rate(5)
    for _ in range(2):
        pub.publish(pose)
        r.sleep()
    return jsonify({"status": "ok"})

@app.route("/slam", methods=["POST"])
def post_slam():
    init_ros()
    from std_srvs.srv import Empty
    rospy.wait_for_service(SLAM_SERVICE, timeout=5)
    slam_srv = rospy.ServiceProxy(SLAM_SERVICE, Empty)
    try:
        slam_srv()
        return jsonify({"status": "map_saved"})
    except Exception as e:
        return jsonify({"status": "error", "msg": str(e)}), 500

def main():
    os.environ["ROS_MASTER_URI"] = ROS_MASTER_URI
    os.environ["ROS_IP"] = ROS_IP
    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.daemon = True
    try:
        init_ros()
    except:
        pass
    ros_thread.start()
    app.run(host=ROBOT_HOST, port=ROBOT_PORT, threaded=True)

if __name__ == "__main__":
    main()
