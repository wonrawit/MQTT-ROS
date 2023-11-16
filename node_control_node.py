#!/usr/bin/env python3
import signal
import rospy
import paho.mqtt.client as mqtt
import ssl
import sys
import json
import cv2
import math
import time
import numpy as np
import os
import yaml
import base64
import json
import actionlib
from actionlib_msgs.msg import GoalStatus
import threading
import subprocess
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Twist

# This directory use for store map file
map_dir = "/home/ubuntu/map/"  

# MQTT broker details
broker_address = "6f8f754a43094b74bbda31621c2316ec.s1.eu.hivemq.cloud"
port = 8883
username = "wonrawit"
password = "Howitzer23092001"

# position and map setting
pose = PoseWithCovarianceStamped()
pose.header.frame_id = "map" 
map_data = None

map_width = 384
map_height = 384
xmin = -5
ymin = -5
xmax = 5
ymax = 5

# Add tuple for a list of navigation goals
goals = []

# Callback when the MQTT client successfully connects to the broker
def on_connect(client, userdata, flags, rc):
    rospy.loginfo("Connected to HiveMQ MQTT broker with result code " + str(rc))
    # Subscribe to the desired topic after connecting
    client.subscribe('move_manual')  # Control via "w a s d x"
    client.subscribe('move_joy') #Control analog joy from application
    client.subscribe('navigation_goal') #Send position for navigation
    client.subscribe('2DEstimate') # Set 2D pose Estimate when start navigation
    client.subscribe('save_map') # Pass this agr for save by name file
    client.subscribe('navigation_run') #Send xml.file name for run rviz navigation
    client.subscribe('gmapping_run') # Run Gmapping for make map
    client.subscribe('delete_map') # Send xaml.file name for delete .xaml and .pgm file
    client.subscribe('get_data') # topic get_map, request_all_map_name, get_map_info
    client.subscribe('request/all_map_name')
    client.subscribe('drawKeepOutZone')
    client.subscribe('new_map_KOZ')
    client.subscribe('new_thread')
    client.subscribe('new_threads')
        
def on_disconnect(client, userdata, rc):
    if rc != 0:
        rospy.logwarn("MQTT connection disconnected with result code: " + str(rc))
        #current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        #print("Disconnected at: " + current_time)

# Callback when a message is received from the MQTT broker
def on_message(client, userdata, messages):
    rospy.loginfo("Received message on topic " + messages.topic + ": " + messages.payload.decode())
    message_str = messages.payload.decode()
    
    if messages.topic == "get_data":
        try:
            if message_str == "get_map":
                send_image()
            elif message_str == "get_map_info":

                map_info_json = {
                    "map_width": map_width,
                    "map_height": map_height,
                    "xmin": xmin,
                    "ymin": ymin,
                    "xmax": xmax,
                    "ymax": ymax,
                    "resolution": map_resolution,
                }
                
                payload_json = json.dumps(map_info_json)
                client.publish("map_info", payload_json, qos=1)
                
            elif message_str == "multi_navigation":
                multi_navigation_goal_thread()
                
            elif message_str == "stop_navigation":
                stop_navigation()
                
            elif message_str == "clear_goals":
                goals.clear()
            
        except Exception as e:
            rospy.logwarn(f"An unexpected error occurred: {e}")
        
    elif messages.topic == "new_threads":
        try:
            json_data = json.loads(message_str)
            thread_type = json_data.get("type")
            
            if thread_type == "time_stop":
                time_stop = int(json_data.get("time"))
                thread = (0, time_stop)
                goals.append(thread)           
                
            elif thread_type == "navigation":
                x = float(json_data.get("x"))
                y = float(json_data.get("y"))
                degree = float(json_data.get("degree"))
                thread = (1, x, y, degree)
                goals.append(thread)

        except Exception as e:
            rospy.logwarn(f"An unexpected error occurred: {e}")
        finally:
            print(goals)                      

    elif messages.topic == "request/all_map_name":
        try:
            list_file_name()
            yaml_res = res        
            formatted_string = ",".join(map(str, yaml_res))
            payload = formatted_string
            client.publish("send/all_map_name", payload, qos=1)
        except Exception as e:
            rospy.logwarn(f"An unexpected error occurred: {e}")
            
    elif messages.topic == "move_manual":
        cmd_vel_pub(message_str)
    
    elif messages.topic == "navigation_goal":
        try:
            if validateJSON(message_str):
                json_data = json.loads(message_str)
                x_navigation = json_data.get("x")
                y_navigation = json_data.get("y")
                degree_navigation = json_data.get("degree")
                navigation_goal_thread(x_navigation, y_navigation, degree_navigation)
            else:
                rospy.logwarn("Decoding JSON has failed")
        except Exception as e:
            rospy.logwarn(f"An unexpected error occurred: {e}")
    
    elif messages.topic == "2DEstimate":
        try:
            if message_str == "default":
                x = -2.0
                y = -0.5
                orientation_quaternion = Quaternion(
                    x=0,
                    y=0,
                    z=0,
                    w=1
                    )
                set_pose_estimate(x, y, orientation_quaternion)
            else:
                if validateJSON(message_str):
                    json_data = json.loads(message_str)
                    x = json_data.get("x")
                    y = json_data.get("y")
                    degree = json_data.get("degree")
                    
                    yaw = math.radians(degree)
                    quaternion = Quaternion()
                    quaternion.z = math.sin(yaw / 2.0)
                    quaternion.w = math.cos(yaw / 2.0)
                    set_pose_estimate(x, y, quaternion)
                else:
                    rospy.logwarn("Decoding JSON has failed")
        except Exception as e:
            rospy.logwarn(f"An unexpected error occurred: {e}")

    elif messages.topic == "save_map": 
        try:
            list_file_name()
            if (message_str+".yaml") not in res:
                os.system("cd "+ map_dir)
                save_map_name = "rosrun map_server map_saver -f " + map_dir + message_str
                os.system(save_map_name)
            else:
                rospy.logwarn("Try another name")
        except Exception as e:
            rospy.logwarn(f"An unexpected error occurred: {e}")
    
    elif messages.topic == "navigation_run":
        try:
            list_file_name()
            run_navi_command = "export TURTLEBOT3_MODEL=burger\n roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=" + map_dir + message_str
            kill_rviz_command = "killall -9 rviz"
            
            if is_yaml_file(message_str):
                if message_str in res:
                    run_command_in_new_gnome_terminal(kill_rviz_command)
                    run_command_in_new_gnome_terminal(run_navi_command)
                else:
                    rospy.logwarn("%s Don't have this xaml filename", message_str)
            else:
                rospy.logwarn("Please Enter xaml filename")
        except Exception as e:
            rospy.logwarn(f"An unexpected error occurred: {e}")

    elif messages.topic == "gmapping_run":
        try:
            if message_str == "open_gmapping":
                run_gmapping_command = "export TURTLEBOT3_MODEL=burger\n roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping"
                kill_rviz_command = "killall -9 rviz"
                run_command_in_new_gnome_terminal(kill_rviz_command)
                run_command_in_new_gnome_terminal(run_gmapping_command)
        except Exception as e:
            rospy.logwarn(f"An unexpected error occurred: {e}")
    
    elif messages.topic == "delete_map":
        try:
            list_file_name()
            if message_str in res:
                yaml_messages = message_str
                pgm_messages = messages.replace(".yaml", ".pgm")
                delete_map_command = "rm " + map_dir + yaml_messages + "\n" + "rm " + map_dir + pgm_messages
                run_command_in_new_gnome_terminal(delete_map_command)
                rospy.logwarn("Delete file ", yaml_messages, " and ", pgm_messages)
            else:
                rospy.logwarn("%s doesn't have exist file", message_str)
        except Exception as e:
            rospy.logwarn(f"An unexpected error occurred: {e}")
    
    elif messages.topic == "new_map_KOZ":
        try:
            if validateJSON(message_str):
                json_data = json.loads(message_str)
                original_map = json_data.get("original_map")
                new_map = json_data.get("new_map")
                
                if original_map in res :        
                    if original_map != None and new_map != None:
                        original_path = map_dir + original_map
                        new_map_path = map_dir + new_map
                        koz_image = cv2.imread(original_path)
                        image = cv2.cvtColor(koz_image, cv2.COLOR_BGR2GRAY)
                        cv2.imwrite(new_map_path, image)
                        read_and_modify_one_block_of_yaml_data(original_path.replace(".pgm", ""), new_map_path.replace(".pgm", ""), key='image', value=new_map_path)
                        rospy.loginfo("Create new map for %s is done", original_map)
                    else:
                        rospy.logwarn("Please Enter map that you want to copy and Enter name that want to create")
                else:
                    rospy.logwarn("%s doesn't have exist file", message_str)
            else:
                rospy.logwarn("Decoding JSON has failed")
        except Exception as e:
            rospy.logwarn(f"An unexpected error occurred: {e}")

    elif messages.topic == "drawKeepOutZone":
        try:
            if validateJSON(message_str):
                json_data = json.loads(message_str)
                map_name = json_data.get("map")
                draw_type = json_data.get("draw_type")
                x1 = json_data.get("x1")
                y1 = json_data.get("y1")
                x2 = json_data.get("x2")
                y2 = json_data.get("y2")
                msg_thickness = json_data.get("thickness")
                pgm_color = json_data.get("color") 

                start_point = (int(x1), int(y1))
                end_point = (int(x2), int(y2))
                thickness = int(msg_thickness)
                
                if map_name != None :
                    koz_path = map_dir + map_name
                    koz_image = cv2.imread(koz_path)
                    image = cv2.cvtColor(koz_image, cv2.COLOR_BGR2GRAY)
                    
                    if pgm_color == "gray":
                        color = (100,100,100)
                    elif pgm_color == "black":
                        color = (0,0,0)
                    elif pgm_color == "white":
                        color = (255,255,255)
                    else:
                        rospy.logwarn("Only have three color: gray, black and white")
                
                    if draw_type == "RECT":
                        if thickness == -1:
                            cv2.rectangle(image, start_point, end_point, color, thickness=cv2.FILLED)
                        elif thickness > 0 :
                            cv2.rectangle(image, start_point, end_point, color, thickness)
                        else:
                            rospy.logwarn("Keepoutzone draw Rectangle, thickness must be positive number or -1 if you want to fill sharp")
                        cv2.imwrite(koz_path, image)
                        rospy.loginfo("draw rectangle success")
                        
                    elif draw_type == "WALL":
                        if thickness>0 :
                            cv2.line(image, start_point, end_point, color, thickness)
                        else:
                            rospy.logwarn("Keepoutzone draw wall, thickness must be positive number")
                        cv2.imwrite(koz_path, image)
                        rospy.loginfo("draw wall success")
                    else :
                        rospy.logwarn("Can't create Keep out Zone, Format not correct")
                else:
                    rospy.logwarn("Please create Keepoutzone file")
            else:
                rospy.logwarn("Decoding JSON has failed")
        except Exception as e:
            rospy.logwarn(f"An unexpected error occurred: {e}")

# Handle keyboard interrupts gracefully
def signal_handler(signal, frame):
    client.disconnect()
    rospy.loginfo("MQTT client disconnected.")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
# Create an MQTT client instance
client = mqtt.Client()
client.on_disconnect = on_disconnect

# Set the username and password
client.username_pw_set(username, password)

# Set up TLS for a secure connection
client.tls_set(tls_version=ssl.PROTOCOL_TLS)

# Attempt to connect to the MQTT broker
client.connect(broker_address, port=port)

def main():
    # Initialize the ROS node
    rospy.init_node('mqtt_ros_node', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
    rospy.Subscriber('/move_base/NavfnROS/plan', Path, path_callback) 
    current_time = time.strftime("%H:%M:%S", time.localtime())

    rospy.loginfo("Start Node")
    # Set the callbacks
    client.on_connect = on_connect
    client.on_message = on_message

    while not rospy.is_shutdown():
        try:
            # Start the MQTT client's network loop
            client.loop_start()
            break  # Break the loop if connection is successful
        except Exception as e:
            rospy.logwarn("Connection failed. Retrying in 5 seconds...")
            rospy.sleep(5)
        
    try:
        # Publish messages or perform other ROS-related tasks
        list_file_name()
        rospy.spin()
    except Exception as e:
        rospy.logerr("An error occurred: " + str(e))

   # Check the MQTT client's connection status
    if not client.is_connected():
        rospy.logwarn("Lost connection to the HiveMQ MQTT broker. Attempting to reconnect...")
        # Attempt to reconnect to the MQTT broker
        client.reconnect()
        rospy.sleep(5)  # Wait for a moment before continuing
    
def map_callback(data):
    global map_width, map_height, xmin, ymin, xmax, ymax, map_resolution


    file_name = "image.png"
    file_path = os.path.join(map_dir, file_name)
    output_path = file_path
    plot_and_save_occupancy_grid(data, output_path)

    map_width = data.info.width
    map_height = data.info.height
    map_resolution = data.info.resolution
    xmin = data.info.origin.position.x
    ymin = data.info.origin.position.y
    xmax = math.ceil(xmin + map_width*map_resolution)
    ymax = math.ceil(ymin + map_height*map_resolution)

    map_info_json = {
        "map_width": map_width,
        "map_height": map_height,
        "xmin": xmin,
        "ymin": ymin,
        "xmax": xmax,
        "ymax": ymax,
        "resolution": map_resolution,
    }
    
    payload_json = json.dumps(map_info_json)
    # payload = "{"+str(map_width)+"},{"+str(map_height)+"},{"+str(xmin)+"},{"+str(ymin)+"},{"+str(xmax)+"},{"+str(ymax)+"}"
    if client.is_connected:
        client.publish("map_info", payload_json, qos=1)
    else:
        rospy.logwarn("Connection MQTT failed. Can't publish map_info")

def plot_and_save_occupancy_grid(occupancy_grid, output_path):
    # Convert OccupancyGrid.data to NumPy array
    occupancy_data = np.asarray(occupancy_grid.data).reshape((occupancy_grid.info.height, occupancy_grid.info.width))
    mirror_occupancy = occupancy_data[::-1]
    mirror_occupancy[mirror_occupancy == -1] = 99
    mirror_occupancy[mirror_occupancy == 100] = -1
    mirror_occupancy[mirror_occupancy == 0] = 255
    cv2.imwrite(output_path, mirror_occupancy)

def path_callback(data):
    x = [pose.pose.position.x for pose in data.poses]
    y = [pose.pose.position.y for pose in data.poses]

    path_data = {
        "x": x,
        "y": y,
    }

    path_json = json.dumps(path_data)
    if client.is_connected :
        client.publish("path_topic", payload=path_json, qos=0)
    else:    
        rospy.logwarn("Connection MQTT failed. Can't publish path")

def odom_callback(data):
    orientation = data.pose.pose.orientation
    if orientation is not None:
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        degree = yaw * 180.0 / 3.141592653589793
    orientation = data.pose.pose.orientation
    if orientation is not None:
        _, _, robot_yaw = quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
    else:
        rospy.logwarn("Invalid orientation data received in odom_callback.")

    odom_data = {
        "x": round(data.pose.pose.position.x, 6),
        "y": round(data.pose.pose.position.y, 6),
        "degree": round(degree, 6),
        "linear_velocity": round(data.twist.twist.linear.x, 6),
        "angular_velocity": round(data.twist.twist.angular.z, 6),
    }

    odom_json = json.dumps(odom_data)
    if client.is_connected :
        client.publish("odom", odom_json, qos=0)
    else:    
        rospy.logwarn("Connection MQTT failed. Can't publish Odom")

def amcl_pose_callback(data):
    orientation = data.pose.pose.orientation
    if orientation is not None:
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        degree = yaw * 180.0 / 3.141592653589793
    orientation = data.pose.pose.orientation
    if orientation is not None:
        _, _, robot_yaw = quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w)
    else:
        rospy.logwarn("Invalid orientation data received in amcl_pose_callback.")

    amcl_pose_data = {
        "x": round(data.pose.pose.position.x, 6),
        "y": round(data.pose.pose.position.y, 6),
        "degree": round(degree, 6),
    }

    amcl_pose_json = json.dumps(amcl_pose_data)
    if client.is_connected :
        client.publish("amcl_pose", amcl_pose_json, qos=1)
    else:    
        rospy.logwarn("Connection MQTT failed. Can't publish AMCL_POSE")

def navigation_goal(x, y, degree):
    global move_base_client
    # Initialize the move_base action client
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base_client.wait_for_server()

    # Create a MoveBaseGoal with the target position and orientation
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # Convert degree to radians for orientation
    yaw = math.radians(degree)
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw / 2.0)
    quaternion.w = math.cos(yaw / 2.0)
    goal.target_pose.pose.orientation = quaternion
    
    move_base_client.send_goal(goal)
    rospy.loginfo("Navigation x: %s y: %s quaternion: %s", x, y, quaternion.w) 
    
    client_state = move_base_client.get_state()
    if client_state == GoalStatus.PENDING:
        rospy.loginfo("The robot is waiting for the operation to start.")
    elif client_state == GoalStatus.ACTIVE:
        rospy.loginfo("The robot is actively navigating towards the goal.")
    elif client_state == GoalStatus.SUCCEEDED:
        rospy.loginfo("The robot successfully reached the goal.")
    elif client_state == GoalStatus.ABORTED:
        rospy.loginfo("The navigation was canceled or encountered an error.")
    elif client_state == GoalStatus.REJECTED:
        rospy.loginfo("The goal was rejected.")
    elif client_state == GoalStatus.RECALLED:
        rospy.loginfo("The navigation was recalled by the navigation plan.")
    elif client_state == GoalStatus.LOST:
        rospy.loginfo("The robot lost communication with the action server.")

def navigation_goal_thread(x, y, degree):
    thread = threading.Thread(target=navigation_goal, args=(x, y, degree))
    thread.start()
    
def multi_navigation_goal():
    
    for i in range(len(goals)):
        
        if goals[i][0] == 1 :
            x, y, degree = goals[i][1], goals[i][2], goals[i][3]
            navigation_goal(x, y, degree)
            
            # Wait until the goal reaches the "SUCCEEDED" status
            while True:
                client_state = move_base_client.get_state()
                if client_state == GoalStatus.SUCCEEDED:
                    #rospy.loginfo("The robot successfully reached the goal.")
                    break
                rospy.sleep(1)  # Adjust the sleep duration as needed
        
        elif goals[i][0] == 0 :
            time_stop = goals[i][1]
            rospy.sleep(time_stop)
            rospy.loginfo("Time stop %s", time_stop)

    rospy.loginfo("All navigation goals completed.")

def multi_navigation_goal_thread():
    thread = threading.Thread(target=multi_navigation_goal, args=())
    thread.start()
    
def stop_navigation():
    # Check if the move_base client is initialized
    if move_base_client:
        # Cancel the current navigation goal if it's still active
        if move_base_client.get_state() == GoalStatus.ACTIVE:
            move_base_client.cancel_goal()
            rospy.loginfo("Navigation goal canceled.")

        # Wait for the goal to be canceled
        move_base_client.wait_for_result()

        # Check the status of the canceled goal
        status = move_base_client.get_state()
        if status == GoalStatus.PREEMPTED:
            rospy.loginfo("Navigation goal preempted (canceled).")
        else:
            rospy.loginfo("Navigation goal canceled and completed.")

def set_pose_estimate(x, y, orientation_quaternion):
    pose.header.stamp = rospy.Time.now()
    pose.pose.pose.position.x = x
    pose.pose.pose.position.y = y

    # Convert quaternion to Euler angles (roll, pitch, yaw)
    (roll, pitch, yaw) = euler_from_quaternion([
        orientation_quaternion.x,
        orientation_quaternion.y,
        orientation_quaternion.z,
        orientation_quaternion.w
    ])

    # Convert Euler yaw to quaternion (for publishing)
    quat = quaternion_from_euler(0, 0, yaw)
    pose.pose.pose.orientation.x = quat[0]
    pose.pose.pose.orientation.y = quat[1]
    pose.pose.pose.orientation.z = quat[2]
    pose.pose.pose.orientation.w = quat[3]

    # Publish the pose estimate
    pub = rospy.Publisher('/amcl_pose', PoseWithCovarianceStamped, queue_size=1)
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    pub.publish(pose)

def cmd_vel_pub(message_str):
    # Initialize the ROS node and publisher
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    try:
        if validateJSON(message_str):
            json_data = json.loads(message_str)
            linear = json_data.get("linear", {})
            angular = json_data.get("angular", {})
            
            # Create a Twist message to control linear and angular velocity
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = linear.get("x", 0.0)
            cmd_vel_msg.linear.y = linear.get("y", 0.0)
            cmd_vel_msg.linear.z = linear.get("z", 0.0)
            cmd_vel_msg.angular.x = angular.get("x", 0.0)
            cmd_vel_msg.angular.y = angular.get("y", 0.0)
            cmd_vel_msg.angular.z = angular.get("z", 0.0)

            # Publish cmd_vel message
            cmd_vel_pub.publish(cmd_vel_msg)
        else:
            rospy.logwarn("Decoding JSON has failed")
    except Exception as e:
        rospy.logwarn(f"An unexpected error occurred: {e}")
    
def map_number(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def read_and_modify_one_block_of_yaml_data(filename,write_file, key,value):
    with open(f'{filename}.yaml', 'r') as f:
        data = yaml.safe_load(f)
        data[f'{key}'] = f'{value}'
        print(data)
    with open(f'{write_file}.yaml', 'w') as file:
        yaml.dump(data,file,sort_keys=False)
    print('edit yaml done!') 

def quaternion_to_euler( x, y, z, w):
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    return roll, pitch, yaw  # return to Tuple

def send_image():
    file_name = "image.png"
    file_path = str(map_dir + file_name)
    # open image from file
    with open(file_path, "rb") as image_file:
        image_data = image_file.read()
        base64_image = base64.b64encode(image_data).decode()
        client.publish("send_map", base64_image, qos=1)

def run_command_in_new_gnome_terminal(command):
    subprocess.run(["gnome-terminal", "--", "bash", "-c", command])

def list_file_name ():
    global res 
    dir_path = os.path.expandvars(map_dir)
    res = []
    for file_path in os.listdir(dir_path):
        if os.path.isfile(os.path.join(dir_path, file_path)):
            res.append(file_path)

def is_positive_integer(num):
    if isinstance(num, int) and num > 0:
        return True
    else:
        return False
    
def is_yaml_file(filename):
    return filename.endswith(".yaml")

def validateJSON(jsonData):
    try:
        json.loads(jsonData)
    except ValueError as err:
        return False
    return True

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException: 
        pass