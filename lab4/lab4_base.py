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
CYCLE_TIME = 0.05  # In seconds

def to_radians( deg):
  return  deg * 3.14159/180.

def to_degrees( rad):
  return  rad * 180/3.14159

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

        # TODO: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))

        msg = Float32MultiArray()
        #print(ir_readings)
        if (ir_readings[1] < IR_THRESHOLD):  # TURN LEFT
            print("RIGHT")
            msg.data = [0.0, 1.0]

        elif ir_readings[3] < IR_THRESHOLD:
            print("LEFT")
            msg.data = [1.0, 0.0]

        elif ir_readings[2] < IR_THRESHOLD and ir_readings[1] > IR_THRESHOLD and ir_readings[3] > IR_THRESHOLD:
            print("STRAIGHT")
            msg.data = [1.0, 1.0]

        publisher_motor.publish(msg)
        publisher_sim.publish(Empty())
        publisher_ping.publish(Empty())
        #publisher_odom.publish(Empty())

        # TODO: Implement loop closure here
        x,y = pose2d_sparki_odometry.x, pose2d_sparki_odometry.y
        #print("sxsy", starting_x, starting_y, x,y)

        if abs(y-starting_y) < 0.02 and x < starting_x and abs(x-starting_x) < 0.02:
            publisher_motor.publish([0,0])
            rospy.loginfo("Loop Closure Triggered")
            rospy.signal_shutdown("Complete")

        convert_robot_coords_to_world()
        populate_map_from_ping()
        display_map()

        #printing cost from 0,0 to 30,40, recount every 1000 loop to avoid lagging
        i_from, i_to = ij_to_cell_index(y_from,x_from), ij_to_cell_index(y_to,x_to)
        if cycle_count%1000 == 1:
            cost_ij = cost(i_from, i_to)
        print ("cost from ",i_from," to ",i_to," is: ", cost_ij)
        print ("next update in ", 1000-cycle_count%1000, " cycles")
        cycle_count += 1

        # TODO: Implement CYCLE TIME
        end_time = time.time()
        delay_time = end_time - begin_time
        if delay_time <= CYCLE_TIME:
            rospy.sleep(CYCLE_TIME - delay_time)
        else:
            print(delay_time)
            print("Time of cycle exceeded .02 seconds")

    #final message
    i_from, i_to = ij_to_cell_index(y_from,x_from), ij_to_cell_index(y_to,x_to)
    print ("Final cost from ", i_from, " to ", i_to, " is: ", cost(i_from, i_to))


def init():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom, publisher_sim
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
    global sensor_reading, servo_angle, state_dict

    sar = to_radians(servo_angle)

    if 'ping' in state_dict:
        sensor_reading = state_dict['ping']
        x_r, y_r = sensor_reading * math.cos(sar), sensor_reading * math.sin(sar)
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
    return j*row_size+i


def cell_index_to_ij(cell_index):
    # TODO: Convert from cell_index to (i,j) coordinates
    return cell_index%row_size, int(cell_index/row_size)


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
                    neighbor_d = []
                    if i > 0:
                        neighbor.append([i-1, j])
                        if j>0: neighbor_d.append([i-1, j-1])
                        if j < col_size - 1: neighbor_d.append([i-1, j+1])
                    if j > 0: neighbor.append([i, j-1])
                    if i < row_size-1:
                        neighbor.append([i + 1, j])
                        if j>0: neighbor_d.append([i+1, j-1])
                        if j < col_size - 1: neighbor_d.append([i+1, j+1])
                    if j < col_size-1: neighbor.append([i , j+1])

                    m[i][j] = min([m[x][y]+1 for x,y in neighbor]+[m[x][y]+1.4 for x,y in neighbor_d])

                    for coord in neighbor+neighbor_d:
                        if coord not in visited:
                            new_queue.append(coord)

        queue = new_queue
        #print(queue)

    #no path found if queue never reach x2, y2
    if m[x2][y2] == 999:
        print("no path found")
    return m[x2][y2]

if __name__ == "__main__":
    main()
