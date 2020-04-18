import rospy
import json
import copy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16
import math

from geometry_msgs.msg import Twist
PI = 3.1415926535897


# GLOBALS 
pose2d_sparki_odometry = None  # Pose2D message object, contains x,y,theta members in meters and radians
# TODO: Track servo angle in radians
servo_angle = 60
map_size = 10
sensor_reading = 0
state_dict = {}
max_x, max_y = 1.72, 1.12
starting_x, starting_y = 0, 0
cycle_count, cost_ij = 0, 0

# TODO: Track IR sensor readings (there are five readings in the array: we've been using indices 1,2,3 for left/center/right)
ir_readings = [0, 0, 0, 0, 0]  # 0 is far left, 4 is far right

# TODO: Create data structure to hold map representation
col_size = 36
row_size = 24

# INPUT FOR COST FUNCTION
x_from, y_from, x_to, y_to = 26,2,28,22
array = [[0 for j in range(col_size)] for i in range(row_size)]

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
CYCLE_TIME = 0.01  # In seconds

def to_radians(deg):
  return  deg * 3.14159/180.

def to_degrees(rad):
  return  rad * 180/3.14159

def move(path): #assumes an array of [ [x1, y1], [x2, y2], etc]
    global pose2d_sparki_odometry
    for coord in path:
# <<<<<<< HEAD
	x = pose2d_sparki_odometry.x
	y = pose2d_sparki_odometry.y
	b_err = math.atan2(coord[1] - y, coord[0] - x)

	#move sparki to the berring error
	rotate(b_err)
	
	#move to the new x and y
	new_pose = Pose2D()
	new_pose = pose2d_sparki_odometry
	new_pose.x = coord[0]
	new_pose.y = coord[1]
	publisher_odom.publish(new_pose)
    	publisher_sim.publish(Empty())
	rospy.sleep(5)
	
def rotate(angle):
# =======
#         x = pose2d_sparki_odometry.x
#         y = pose2d_sparki_odometry.y
#         theta = pose2d_sparki_odometry.theta
#         b_err = math.atan2(coord[1] - y, coord[0] - x)
#         dis = math.sqrt(pow(2,coord[0] - x) + pow(2, coord[1] - y))
#
#         #move sparki to the berring error
#         msg = Float32MultiArray()
#         msg.data = [4.0, 4.0]
#
#         rotate(180)
#
# 	'''current_distance = 0
# 	t0 = rospy.Time.now().to_sec()
# 	while current_distance < dis:
# 	     publisher_motor.publish(msg)
#              publisher_sim.publish(Empty())
# 	     t1 = rospy.Time.now().to_sec()
#              current_distance = 8.0 * (t1 - t0)
# 	print(pose2d_sparki_odometry)
# 	msg.data = [0.0, 0.0]
#     	publisher_motor.publish(msg)'''
#
#
# def rotate(angle):
#     #rotate for angel/speed seconds
# >>>>>>> 301051774c8b59f794f0d4de6b865faeaf20933d

    #get and calc sparkis new theta
    theta = pose2d_sparki_odometry.theta
    goal_theta = theta + to_radians(angle)
   
    #make a Pose2D() message and publish new odometry with new theta
    new_pose = Pose2D()
    new_pose = pose2d_sparki_odometry
    new_pose.theta = goal_theta
    publisher_odom.publish(new_pose)
    publisher_sim.publish(Empty())

    rospy.sleep(5)
    
# <<<<<<< HEAD
# =======
#     while(t1 - t0 <= time_to_rotate):
#         publisher_motor.publish(msg)
#         publisher_sim.publish(Empty())
#         t1 = rospy.Time.now().to_sec()
#
#     print(pose2d_sparki_odometry.theta)
#     print("ALL DONE ROATING")
#     msg.data = [0.0, 0.0]
#     publisher_motor.publish(msg)
#     publisher_sim.publish(Empty())
#     rospy.sleep(10)
# >>>>>>> 301051774c8b59f794f0d4de6b865faeaf20933d

def main():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry
    global starting_x, starting_y, cycle_count, cost_ij, x_from, y_from, x_to, y_to

    # TODO: Init your node to register it with the ROS core
    rospy.init_node('obstaclefinder', anonymous=True)
    init()

    while not rospy.is_shutdown():  # TODO: Implement CYCLE TIME
        begin_time = time.time()
        #print("")
        # TODO: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))
	'''
       	msg = Float32MultiArray()     
	msg.data = [1.0,1.0]
	print(pose2d_sparki_odometry.theta)


        arr = [[1.5, 0], [1.7, 0]]
        movement move(arr)

       	msg = Float32MultiArray()
        msg.data = [1.0,1.0]
        print(pose2d_sparki_odometry.theta)




        publisher_motor.publish(msg)
        publisher_sim.publish(Empty())
        #publisher_odom.publish(Empty())

        # TODO: Implement loop closure here
        x,y = pose2d_sparki_odometry.x, pose2d_sparki_odometry.y
        #print("sxsy", starting_x, starting_y, x,y)

	'''
        arr = [[1.4, 1], [1.7, 0]]
        move(arr)
        #print(pose2d_sparki_odometry)
        #rotate(90)
# =======
# >>>>>>> 301051774c8b59f794f0d4de6b865faeaf20933d
      


        # TODO: Implement CYCLE TIME
        end_time = time.time()
        delay_time = end_time - begin_time
        #print(end_time - begin_time)
        if delay_time <= CYCLE_TIME:
            rospy.sleep(CYCLE_TIME - delay_time)
        else:
            print("Cycle time exceeded: ", delay_time)


def init():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, publisher_sim, velocity_publisher
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry
    global servo_angle, starting_x, starting_y

    # TODO: Set up your publishers and subscribers
    publisher_motor = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size=10)
    publisher_ping = rospy.Publisher('/sparki/ping_command', Empty, queue_size=10)
    publisher_servo = rospy.Publisher('/sparki/set_servo', Int16, queue_size=10)
    publisher_odom = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=10)
    publisher_sim = rospy.Publisher('/sparki/render_sim', Empty, queue_size=10)
    subscriber_odometry = rospy.Subscriber("/sparki/odometry", Pose2D, callback_update_odometry)
    subscriber_state = rospy.Subscriber("/sparki/state", String, callback_update_state)

    velocity_publisher = rospy.Publisher('/sparki/cmd_vel', Twist, queue_size=10)
    rospy.sleep(0.5)

    # TODO: Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    pose2d_sparki_odometry = Pose2D()

    # TODO: Set sparki's servo to an angle pointing inward to the map (e.g., 45)

    publisher_servo.publish(int(servo_angle))
    publisher_sim.publish(Empty())
    rospy.sleep(0.5)
    starting_x, starting_y = pose2d_sparki_odometry.x, pose2d_sparki_odometry.y


def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry

    # TODO: Copy this data into your local odometry variable
    pose2d_sparki_odometry = data


def callback_update_state(data):
    # TODO: Load data into your program's local state variables
    global ir_readings, state_dict
    state_dict = json.loads(data.data)  # Creates a dictionary object from the JSON string received from the state topic
    ir_readings = state_dict['light_sensors']
    #publisher_servo = state_dict['servo']


def convert_ultrasonic_to_robot_coords():
    # TODO: Using US sensor reading and servo angle, return value in robot-centric coordinates
    # Make things way easier if return sensor_reading
    global sensor_reading, servo_angle, state_dict, cycle_count

    sar = to_radians(servo_angle)

    if 'ping' in state_dict:
        sensor_reading = state_dict['ping']
        x_r, y_r = sensor_reading * math.cos(sar), sensor_reading * math.sin(sar)
        if cycle_count%200 == 2:
            print("x_r,y_r, ping_reading:",x_r, y_r, sensor_reading)
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
    #print(x_w, y_w, "world coord")
    return x_w, y_w

def populate_map_from_ping():
    # TODO: Given world coordinates of an object detected via ping, fill in the corresponding part of the map

    global pose2d_sparki_odometry

    sensor_reading = convert_ultrasonic_to_robot_coords()
    x_ping, y_ping = 0, 0
    sar = to_radians(servo_angle)
    if pose2d_sparki_odometry != None and sensor_reading > 0:
        x, y, t = pose2d_sparki_odometry.x, pose2d_sparki_odometry.y, pose2d_sparki_odometry.theta
        x_ping, y_ping = x + sensor_reading*math.cos(sar+t),y+sensor_reading * math.sin(sar + t)


    #populate map with x_ping, y_ping
    if x_ping > 0 and y_ping > 0:
        global col_size, row_size, array

        #x_ping, y_ping = x_ping/col_size*(col_size-1), y_ping/row_size*(row_size-1)
        i,j = int(y_ping/max_y*row_size), int(x_ping/max_x*col_size)
        array[i][j] = 1


def display_map():
    # TODO: Display the map
    global pose2d_sparki_odometry
    #robot location x,y
    x, y = int(pose2d_sparki_odometry.y / max_y * row_size), int(pose2d_sparki_odometry.x / max_x * col_size)
    symbol = ["_", "X"]
    for i in reversed(range(row_size)):
        for j in range(col_size):
            if i == x and j == y:
                print "R" ,
            elif i == y_from and j == x_from:
                print "S" ,
            elif i == y_to and j == x_to:
                print "T" ,
            else:
                print symbol[array[i][j]] ,
        print("")


def ij_to_cell_index(i, j):
    # TODO: Convert from i,j coordinates to a single integer that identifies a grid cell
    #cell index starts from 0 to row*col-1
    global col_size
    return i*col_size+j


def cell_index_to_ij(cell_index):
    # TODO: Convert from cell_index to (i,j) coordinates
    global col_size
    return int(cell_index/col_size), cell_index%col_size


def cost(cell_index_from, cell_index_to):
    # TODO: Return cost of traversing from one cell to another
    global array
    m = copy.deepcopy(array)
    for i, row in enumerate(m):
        for j, col in enumerate(row):
            if col == 1:
                m[i][j] = 9999
            if col == 0:
                m[i][j] = 999
    x1,y1 = cell_index_to_ij(cell_index_from)
    x2,y2 = cell_index_to_ij(cell_index_to)

    #check if from/to valid
    try:
        if m[x1][y1] > 1000:
            print("Starting index on a wall")
            return 0
    except:
        print("Starting index out of bound")
        return 0
    try:
        if m[x2][y2] > 1000:
            print("Target index on a wall")
            return 0
    except:
        print("Target index out of bound")
        return 0

    #start of magic algorithm
    queue = [[x1,y1]]
    visited = []

    #cost of starting coord will be 0
    m[x1][y1] = -1

    while len(queue) > 0:
        new_queue = []

        for i,j in queue:
            #if in visited or wall do nothing, else run main algorithm
            if [i,j] not in visited:
                visited.append([i, j])
                if m[i][j] < 1000:
                    neighbor = [[i,j]]
                    val = m[i][j]
                    if i > 0: neighbor.append([i-1, j])
                    if j > 0: neighbor.append([i, j-1])
                    if i < row_size-1: neighbor.append([i + 1, j])
                    if j < col_size-1: neighbor.append([i , j+1])

                    #find best cost among neighbors
                    m[i][j] = min([m[x][y] + 1 for x, y in neighbor])

                    #check if there is better path for neighbor
                    if i > 0 and m[i - 1][j] > val + 1 and m[i - 1][j] < 999:
                        m[i - 1][j] = val + 1
                    if j > 0 and m[i][j - 1] > val + 1 and m[i][j - 1] < 999:
                        m[i][j - 1] = val + 1
                    if i < row_size - 1 and m[i + 1][j] > val + 1 and m[i + 1][j] < 999:
                        m[i + 1][j] = val + 1
                    if j < col_size - 1 and m[i][j + 1] > val + 1 and m[i][j + 1] < 999:
                        m[i][j + 1] = val + 1

                    for coord in neighbor:
                        if coord not in visited:
                            new_queue.append(coord)

        queue = new_queue

    #no path found if queue never reach x2, y2
    if m[x2][y2] == 999:
        print("no path found")
    return m[x2][y2]

if __name__ == "__main__":
    main()
