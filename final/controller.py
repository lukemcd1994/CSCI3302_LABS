import argparse
import rospy
from geometry_msgs.msg import PoseStamped
import math
import time


#robot controller take agent, publisher

#current loop time: 0.2
#odometry format: {x:float, y:float, theta:float, mode:int}
#modes: SEEK = 0, THROW = 1, OWN = 2

#version 1: walk to the ball and throw at goal
class Agent_v1:
    def __init__(self, agent_odom, goal_loc):
        self.odom = agent_odom
        self.loc = agent_odom[:2]
        self.goal_loc = goal_loc
        self.mode = 0
        self.wait_timer = 0
        self.wait_limit = 10

        self.PICKUP_RANGE = 1
        self.MAX_ANGULAR_ERR = 0.1
        # modes
        self.SEEK = 0
        self.THROW = 1
        self.WAIT = 2

    def calc_dist(self, loc1, loc2):
        return math.sqrt((loc1[0] - loc2[0]) ** 2 + (loc1[1] - loc2[1]) ** 2)
    def calc_angle(self, loc1, loc2):
        return math.atan2((loc2[1] - loc1[1]) , (loc2[0] - loc1[0]))

    #if SEEK move toward ball, if THROW aim toward goal, if WAIT wait
    #return format: [x,y,theta], T/F to_throw
    def act(self, ball_loc, self_odometry):
        x, y, theta = self_odometry.x, self_odometry.y, self_odometry.theta
        self.loc = x,y
        self.odom = x, y, theta
        if self.mode == self.SEEK:
            if self.calc_dist(self.loc, ball_loc) <= self.PICKUP_RANGE:
                return ball_loc + [None], False
            else:
                self.mode = self.THROW
                return None, self.calc_angle(self.loc, ball_loc), False
        elif self.mode == self.THROW:
            if abs(theta - self.calc_dist(self.loc, ball_loc))>self.MAX_ANGULAR_ERR:
                return self.loc + [self.calc_angle(self.loc, ball_loc)], False
            else:
                self.mode = self.WAIT
                return self.odom, True
        elif self.mode == self.WAIT:
            if self.wait_timer >= self.wait_limit:
                self.mode == self.SEEK
            else:
                self.wait_timer += 1
            return self.odom, False

#controllers
def controller_v1(agent, publisher):
    #using slow cycle time because all the publishing take quiet some time
    CYCLE_TIME = 1

    MAX_GOAL_ERR = 0.5

    #init agent and publisher
    agen = Agent_v1([1, 1], [0, 6])
    publ = Publisher_v1()

    #TODO: how to get odometry
    odom = get_odometry()
    ball_odom = odom.ball
    agent_odom = odom.agent
    goal_loc = [6, 0]

    while agen.calc_dist(ball_odom[:2], goal_loc) > MAX_GOAL_ERR:
        begin_time = time.time()

        odom = get_odometry()
        ball_odom = odom.ball
        agent_odom = odom.agent
        command, throw = agent.act(ball_odom, agent_odom)
        publ.publish_move_command(&command)
        if throw: publ.publish_throw()

        end_time = time.time()
        delay_time = end_time - begin_time
        if delay_time <= CYCLE_TIME:
            rospy.sleep(CYCLE_TIME - delay_time)
        else:
            print("Cycle time exceeded: ", delay_time)


def controller_t1(agent, publisher):
    # using slow cycle time because all the publishing take quiet some time
    CYCLE_TIME = 1

    MAX_GOAL_ERR = 0.5

    # init agent and publisher
    agen = Agent_v1([1, 1], [0, 6])
    publ = Publisher_v1()

    agent_odom = [1,1,0]
    goal_loc = [6,0]

    while agen.calc_dist(ball_odom[:2], goal_loc) > MAX_GOAL_ERR:
        begin_time = time.time()

        odom = get_odometry()
        ball_odom = odom.ball
        agent_odom = odom.agent
        agent.act(ball_odom, agent_odom)

        end_time = time.time()
        delay_time = end_time - begin_time
        if delay_time <= CYCLE_TIME:
            rospy.sleep(CYCLE_TIME - delay_time)
        else:
            print("Cycle time exceeded: ", delay_time)


###
class Publisher_v1:
    def __init__(self):
        self.publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.init_node('taskmaster', anonymous=True)
        rospy.sleep(1)
        self.p = PoseStamped()
        self.p.header.frame_id = 'map'
        self.p.header.seq = 0

    def publish_move_command(self, x, y, theta):
        p, publisher = self.p, self.publisher

        p.header.stamp = rospy.Time.now()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.w = math.cos(theta / 2)

        publisher.publish(p)

    def publish_throw(self):
        #TODO: how to throw
        pass

#TODO: get_odometry
def get_odometry():
    return []

###MY QUESTIONS
# How do agent get self location
# What happens if you repeat same command over time
# How do you get ball location

##test
if __name__ == "__main__":
    agen = Agent_v1([1,1], [0,6])
    publ = Publisher_v1()

    #publisher test
    #publ.publish_move_command(5, 0, 3.1)

    #agent test