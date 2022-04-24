import rospy
from geometry_msgs.msg import Twist

class TurtleBot:
    def __init__(self): 
        rospy.init_node('turtlebot', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.shaft_length = 0.16

    def executeActions(self, actions_list):
        # open loop controller
        while not rospy.is_shutdown(): 
            for i in range(3): 
                cmd_vel_msg = self.actionToCmdVel((0, 0))
                self.cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(1)

            for i, action in enumerate(actions_list):
                cmd_vel_msg = self.actionToCmdVel(action)
                self.cmd_vel_pub.publish(cmd_vel_msg)
                if (rospy.is_shutdown()): 
                    break 
                
                rospy.sleep(0.5)

                # if cmd_vel_msg.angular.z != 0:  
                #     rospy.sleep(2.2)
                # else:
                #     rospy.sleep(1.95)

            for i in range(3): 
                cmd_vel_msg = self.actionToCmdVel((0, 0))
                self.cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(1)
             
            rospy.loginfo("Finish"); 
            break; 


             

    def actionToCmdVel(self, action): 
         left_v, right_v = action
         left_v, right_v = left_v / 100, right_v / 100
         cmd_vel_msg = Twist() 
         cmd_vel_msg.linear.x = 0.5 * (right_v + left_v)
         cmd_vel_msg.angular.z = (right_v - left_v) / self.shaft_length
         linear_log = "linear velocity: %f" % cmd_vel_msg.linear.x
         angular_log = "angular velocity: %f" % cmd_vel_msg.angular.z
         rospy.loginfo(linear_log)
         rospy.loginfo(angular_log)

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
         

