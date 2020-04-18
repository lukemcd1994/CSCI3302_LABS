import argparse
import rospy
from geometry_msgs.msg import PoseStamped
from math import sin,cos
if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Dijkstra on image file")
  parser.add_argument('-x', '--goal_x', type=float, nargs=1, default=[1], help='Goal x coordinate')
  parser.add_argument('-y', '--goal_y', type=float, nargs=1, default=[.5], help='Goal y coordinate')
  parser.add_argument('-theta', '--goal_theta', type=float, nargs=1, default=[.5], help='Goal theta orientation')
  args = parser.parse_args()

  publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
  rospy.init_node('taskmaster', anonymous=True)
  rospy.sleep(1)
  p = PoseStamped()

  p.header.frame_id = 'map'
  p.header.seq = 0
  p.header.stamp = rospy.Time.now()
  p.pose.position.x = args.goal_x[0]
  p.pose.position.y = args.goal_y[0]
  p.pose.orientation.w = cos(args.goal_theta[0]/2)

  publisher.publish(p)

  rospy.sleep(30)

sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers

