import rospy
from geometry_msgs.msg import Twist

class TurtleBot:
    def __init__(self): 
        rospy.init_node('turtlebot', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.shaft_length = 0.16

    def executeActions(self, actions_list):
        rate = rospy.Rate(1)

        # open loop controller
        while not rospy.is_shutdown(): 
            for action in actions_list:
                cmd_vel_msg = self.actionToCmdVel(action)
                self.cmd_vel_pub.publish(cmd_vel_msg)
                rate.sleep()
             

    def actionToCmdVel(self, action): 
         left_v, right_v = action
         cmd_vel_msg = Twist() 
         cmd_vel_msg.linear.x = 0.5 * (right_v + left_v)
         cmd_vel_msg.angular.z = (right_v - left_v) / self.shaft_length

         return cmd_vel_msg

if __name__ == '__main__':
    turtlebot = TurtleBot()
    actions_list = [(0.1, 0.1)] * 5 + \
                   [(0.2, 0.2)] * 5 + \
                   [(0.3, 0.3)] * 5 + \
                   [(0.2, 0.2)] * 5 + \
                   [(0.1, 0.1)] * 5 + \
                   [(0, 0)] * 5 

    try:
        turtlebot.executeActions(actions_list)
    except rospy.ROSInterruptException:
        pass
         

