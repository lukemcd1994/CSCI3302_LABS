import rospy
import json
import copy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
import math

# GLOBALS 
pose2d_sparki_odometry = None  # Pose2D message object, contains x,y,theta members in meters and radians
sensor_reading = 0
state_dict = {}
max_x, max_y = 1.8, 1.2
starting_x, starting_y = 0, 0
cycle_count, cost_ij = 0, 0

# TODO: Use these variables to hold your publishers and subscribers
publisher_motor = None
publisher_odom = None
publisher_ping = None
publisher_servo = None
publisher_sim = None
subscriber_odometry = None
subscriber_state = None

# CONSTANTS
#IR_THRESHOLD = 300  # IR sensor threshold for detecting black track. Change as necessary.
#CYCLE_TIME = 0.01  # In seconds
M_PI = 3.14159265

def to_radians( deg):
  return  deg * 3.14159/180.

def to_degrees( rad):
  return  rad * 180/3.14159

def trim_angle(rad):
    if (rad > M_PI):
        return rad - 2 * M_PI
    if (rad < -M_PI):
        return rad + 2 * M_PI
    return rad

def main(path):
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry
    global starting_x, starting_y, cycle_count, cost_ij, x_from, y_from, x_to, y_to

    # TODO: Init your node to register it with the ROS core
    rospy.init_node('obstaclefinder', anonymous=True)

    init()

    #generate engine commands
    #SPARKI_SPEED = 0.0278  # 100% speed in m/s
    SPARKI_SPEED = 0.0056
    #SPARKI_SPEED = 0.0099
    SPARKI_AXLE_DIAMETER = 0.085  # Distance between wheels, meters

    engine_command = []

    p_x,p_y = path[0]
    theta = 0

    #init robot position to initial location
    msg = pose2d_sparki_odometry
    msg.x, msg.y, msg.theta = p_x, p_y, theta
    publisher_odom.publish(msg)

    msg = Float32MultiArray()
    #print("x,y, t:", p_x, p_y, theta)
    for x, y in path[1:]:
        print ("x,y,px,py",x, y, p_x, p_y)
        dist = math.sqrt(pow(p_x - x, 2) + pow(p_y - y, 2))
        if dist > 0:
            b_err = math.atan2((y - p_y) , (x - p_x)) - theta
            b_err = trim_angle(b_err)
        else:
            b_err = 0
        # engine_command measured in [turning time, forward time]
        #engine_command.append([b_err * SPARKI_AXLE_DIAMETER / 2 / SPARKI_SPEED, dist / SPARKI_SPEED])
        rot_time, fow_time = b_err * SPARKI_AXLE_DIAMETER / 2 / SPARKI_SPEED, dist / SPARKI_SPEED

        #p_x, p_y, theta = x, y, theta + b_err
        print("b_err,dist:", b_err, dist)

    #first correct b_err
        if rot_time < 0:
            msg.data = [-1.0, 1.0]
        else:
            msg.data = [1.0, -1.0]
        publisher_motor.publish(msg)
        t0 = rospy.Time.now().to_sec()
        t1 = t0
        while t1-t0 < abs(rot_time):
            publisher_sim.publish(Empty())
            t1 = rospy.Time.now().to_sec()

    #correct b_err twice because this simulator is ridiculous
        b_err = math.atan2((y - p_y), (x - p_x)) - pose2d_sparki_odometry.theta
        b_err = trim_angle(b_err)
        rot_time = b_err * SPARKI_AXLE_DIAMETER / 2 / SPARKI_SPEED
        if rot_time < 0:
            msg.data = [-1.0, 1.0]
        else:
            msg.data = [1.0, -1.0]
        publisher_motor.publish(msg)
        t0 = rospy.Time.now().to_sec()
        t1 = t0
        while t1 - t0 < abs(rot_time):
            publisher_sim.publish(Empty())
            t1 = rospy.Time.now().to_sec()

    #correct distance error
        msg.data = [2.0, 2.0]
        publisher_motor.publish(msg)
        t0 = rospy.Time.now().to_sec()
        t1 = t0
        while t1 - t0 < fow_time/2:
            publisher_sim.publish(Empty())
            t1 = rospy.Time.now().to_sec()

        p_x = pose2d_sparki_odometry.x
        p_y = pose2d_sparki_odometry.y
        theta = pose2d_sparki_odometry.theta
        print("end of loop: x,y,t:", p_x, p_y, theta)

    msg.data = [0.0, 0.0]
    publisher_motor.publish(msg)
    publisher_sim.publish(Empty())
    print("done")


def init():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, publisher_sim
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry
    global servo_angle, starting_x, starting_y

    # TODO: Set up your publishers and subscribers
    publisher_motor = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size=10)
    #publisher_ping = rospy.Publisher('/sparki/ping_command', Empty, queue_size=10)
    #publisher_servo = rospy.Publisher('/sparki/set_servo', Int16, queue_size=10)
    publisher_odom = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=10)
    publisher_sim = rospy.Publisher('/sparki/render_sim', Empty, queue_size=10)
    subscriber_odometry = rospy.Subscriber("/sparki/odometry", Pose2D, callback_update_odometry)
    #subscriber_state = rospy.Subscriber("/sparki/state", String, callback_update_state)
    rospy.sleep(0.5)

    pose2d_sparki_odometry = Pose2D()

    publisher_sim.publish(Empty())
    rospy.sleep(0.5)

def callback_update_odometry(data):
    global pose2d_sparki_odometry
    pose2d_sparki_odometry = data

def callback_update_state(data):
    global ir_readings, state_dict
    state_dict = json.loads(data.data)


if __name__ == "__main__":
    path = [[0.5, 0.5], [0.5, 0.6]]
    #path = [[0.5, 0.5], [0.6, 0.5], [0.7, 0.5]]
    main(path)
