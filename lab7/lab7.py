import argparse
import rospy
from geometry_msgs.msg import PoseStamped
from math import sin,cos
if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Dijkstra on image file")
  parser.add_argument('-x', '--goal_x', type=float, nargs=1, default=[1], help='Goal x coordinate')
  parser.add_argument('-y', '--goal_y', type=float, nargs=1, default=[1], help='Goal y coordinate')
  parser.add_argument('-theta', '--goal_theta', type=float, nargs=1, default=[.5], help='Goal theta orientation')
  # parser.add_argument('-g','--dest_coordinates', nargs=2, default=[1, 3], help='Goal x, y location in world coords')
  # parser.add_argument('-o','--obstacles', nargs='?', type=str, default='obstacles_test1.png', help='Black and white image showing the obstacle locations')
  args = parser.parse_args()

  publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
  rospy.init_node('taskmaster', anonymous=True)
  p = PoseStamped()
  # p.header.stamp = rospy.Time.now()


  p.header.frame_id = 'map'
  p.pose.position.x = args.goal_x[0]
  p.pose.position.y = args.goal_y[0]
  p.pose.orientation.x = sin(args.goal_theta[0]/2)
  p.pose.orientation.y = sin(args.goal_theta[0]/2)
  p.pose.orientation.z = sin(args.goal_theta[0]/2)
  p.pose.orientation.w = cos(args.goal_theta[0]/2)

  publisher.publish(p)

  rospy.sleep(30)
