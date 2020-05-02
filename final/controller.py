import argparse
import rospy
from geometry_msgs.msg import PoseStamped
import math


#robot controller take agent name, and dict of agents' odometry

#current loop time: 0.2
#odometry format: {x:float, y:float, theta:float, mode:int}
#modes: SEEK = 0, THROW = 1, OWN = 2

#problem notes:
#1, avoid catching ball right after a throw

#version 1: walk to the ball and throw at goal
class Agent_v1:
    def __init__(self, agent_loc, goal_loc):
        self.loc = agent_loc
        self.goal_loc = goal_loc
        self.mode = 0
        self.wait_timer = 0
        self.wait_limit = 10

        self.PICKUP_RANGE = 1
        self.AIM_ANGULAR_TOR = 0.1
        # modes
        self.SEEK = 0
        self.THROW = 1
        self.WAIT = 2

    def calc_dist(self, loc1, loc2):
        return math.sqrt((loc1[0] - loc2[0]) ** 2 + (loc1[1] - loc2[1]) ** 2)
    def calc_angle(self, loc1, loc2):
        return math.atan2((loc2[1] - loc1[1]) , (loc2[0] - loc1[0]))

    #if SEEK move toward ball, if THROW aim toward goal, if WAIT wait
    #return format: [target_loc, turn_angle?]
    def move(self, ball_loc, turret_angle):
        if self.mode == self.SEEK:
            if self.calc_dist(self.loc, ball_loc) <= self.PICKUP_RANGE:
                return ball_loc, None
            else:
                self.mode = self.THROW
                return None, self.calc_angle(self.loc, ball_loc)
        elif self.mode == self.THROW:
            if abs(turret_angle - self.calc_dist(self.loc, ball_loc))>self.AIM_ANGULAR_TOR:
                return None, self.calc_angle(self.loc, ball_loc)
            else:
                self.mode = self.WAIT
                self.throw()
                return None, None
        elif self.mode == self.WAIT:
            if self.wait_timer >= self.wait_limit:
                self.mode == self.SEEK
            else:
                self.wait_timer += 1
            return None, None

    def throw(self):
        #TODO: throw ball
        pass


def controller_v1(name, odometries, parameters):
    mod = odometries[name][mode]
    #OWN Mode
    if mod == 2:
        #angle from goal to current loc
        r_x, r_y = odometries[name][x], odometries[name][y]
        g_x, g_y = parameters["goal"][x], parameters["goal"][y]
        goal_direction = math.atan2(g_x-r_x, g_y-r_y)



###MY QUESTIONS
# How do agent get self location
# What happens if you repeat same command over time
# How do you get ball location
