import rospy
import json
import copy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
import math

# GLOBALS 
pose2d_sparki_odometry = None  # Pose2D message object, contains x,y,theta members in meters and radians
# TODO: Track servo angle in radians
servo_angle = 0
map_size = 10
sensor_reading = 0
state_dict = {}

# TODO: Track IR sensor readings (there are five readings in the array: we've been using indices 1,2,3 for left/center/right)
ir_readings = [0, 0, 0, 0, 0]  # 0 is far left, 4 is far right

# TODO: Create data structure to hold map representation
array = [[0 for i in range(map_size)] for j in range(map_size)]

# TODO: Use these variables to hold your publishers and subscribers
publisher_motor = None
publisher_odom = None
publisher_ping = None
publisher_servo = None
publisher_sim = None
subscriber_odometry = None
subscriber_state = None

# CONSTANTS
IR_THRESHOLD = 300  # IR sensor threshold for detecting black track. Change as necessary.
CYCLE_TIME = 0.05  # In seconds


def main():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry


    # TODO: Init your node to register it with the ROS core
    rospy.init_node('obstaclefinder', anonymous=True)
    init()

    while not rospy.is_shutdown():  # TODO: Implement CYCLE TIME
        begin_time = time.time()

        # TODO: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))

        msg = Float32MultiArray()
        print(ir_readings)
        if (ir_readings[1] < IR_THRESHOLD):  # TURN LEFT
            print("LEFT")
            msg.data = [0.0, 1.0]

        elif ir_readings[3] < IR_THRESHOLD:
            print("RIGHT")
            msg.data = [1.0, 0.0]

        elif ir_readings[2] < IR_THRESHOLD and ir_readings[1] > IR_THRESHOLD and ir_readings[3] > IR_THRESHOLD:
            print("STRAIGHT")
            msg.data = [1.0, 1.0]
        #msg.data = [1.0, 1.0]
        publisher_motor.publish(msg)
        publisher_sim.publish(Empty())
        publisher_ping.publish(Empty())
        #publisher_odom.publish(Empty())

        # TODO: Implement loop closure here
        if pose2d_sparki_odometry == [-1, 0, 0]:
            rospy.loginfo("Loop Closure Triggered")
            rospy.signal_shutdown()

        convert_robot_coords_to_world()
        populate_map_from_ping()

        # TODO: Implement CYCLE TIME
        end_time = time.time()
        delay_time = end_time - begin_time
        if delay_time <= CYCLE_TIME:
            rospy.sleep(CYCLE_TIME - delay_time)
        else:
            print(delay_time)
            print("Time of cycle exceeded .5 seconds")


def init():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, publisher_sim
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry
    global servo_angle

    # TODO: Set up your publishers and subscribers
    publisher_motor = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size=10)
    publisher_ping = rospy.Publisher('/sparki/ping_command', Empty, queue_size=10)
    publisher_servo = rospy.Publisher('/sparki/set_servo', Int16, queue_size=10)
    publisher_odom = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=10)
    publisher_sim = rospy.Publisher('/sparki/render_sim', Empty, queue_size=10)
    subscriber_odometry = rospy.Subscriber("/sparki/odometry", Pose2D, callback_update_odometry)
    subscriber_state = rospy.Subscriber("/sparki/state", String, callback_update_state)
    rospy.sleep(0.5)

    # TODO: Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    pose2d_sparki_odometry = Pose2D()

    # TODO: Set sparki's servo to an angle pointing inward to the map (e.g., 45)

    publisher_servo.publish(int(servo_angle))
    #publisher_servo.publish(45)
    publisher_sim.publish(Empty())
    rospy.sleep(0.5)


def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry

    # TODO: Copy this data into your local odometry variable
    pose2d_sparki_odometry = data


def callback_update_state(data):
    global ir_readings, state_dict
    state_dict = json.loads(data.data)  # Creates a dictionary object from the JSON string received from the state topic
    # TODO: Load data into your program's local state variables
    ir_readings = state_dict['light_sensors']
    # publisher_servo = state_dict['servo']


def convert_ultrasonic_to_robot_coords():
    # TODO: Using US sensor reading and servo angle, return value in robot-centric coordinates
    # Make things way easier if return sensor_reading
    global sensor_reading, servo_angle, state_dict

    if 'ping' in state_dict:
        sensor_reading = state_dict['ping']
        x_r, y_r = sensor_reading * math.sin(servo_angle), sensor_reading * math.cos(servo_angle)
        print(x_r, y_r, "xr,yr readings", sensor_reading)
        #return x_r, y_r
        return sensor_reading

    else:
        return None

def convert_robot_coords_to_world():
    # TODO: Using odometry, convert robot-centric coordinates into world coordinates
    x_w, y_w = 0,0

    global pose2d_sparki_odometry
    if pose2d_sparki_odometry != None:
        x_w, y_w = pose2d_sparki_odometry.x, pose2d_sparki_odometry.y

    return x_w, y_w

def populate_map_from_ping():
    # TODO: Given world coordinates of an object detected via ping, fill in the corresponding part of the map
    global pose2d_sparki_odometry

    sensor_reading = convert_ultrasonic_to_robot_coords()
    x_ping, y_ping = 0, 0

    if pose2d_sparki_odometry != None:
        x, y, t = pose2d_sparki_odometry.x, pose2d_sparki_odometry.y, pose2d_sparki_odometry.theta
        x_ping, y_ping = x + sensor_reading*math.sin(servo_angle+t),y+sensor_reading * math.cos(servo_angle + t)

    #populate map with x_ping, y_ping



def display_map():
    # TODO: Display the map
    for i in range(map_size):
        for j in range(map_size):
            print(map[i][j] + " ")
        print("\n")

    pass


def ij_to_cell_index(i, j):
    # TODO: Convert from i,j coordinates to a single integer that identifies a grid cell
    return 0


def cell_index_to_ij(cell_index):
    # TODO: Convert from cell_index to (i,j) coordinates
    return 0, 0


def cost(cell_index_from, cell_index_to):
    # TODO: Return cost of traversing from one cell to another
    return 0


if __name__ == "__main__":
    main()
