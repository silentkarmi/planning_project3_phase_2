import rospy
from geometry_msgs.msg import Twist

class TurtleBot:
    COUNTER = 0
    
    def __init__(self): 
        rospy.init_node('turtlebot', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.shaft_length = 0.16

    def executeActions(self, actions_list):
        # open loop controller
        while not rospy.is_shutdown(): 
            for i in range(2): 
                cmd_vel_msg = self.actionToCmdVel((0, 0))
                self.cmd_vel_pub.publish(cmd_vel_msg)
                
                TIME_INTERVAL = 0.5
                rospy.sleep(TIME_INTERVAL)

            for i, action in enumerate(actions_list):
                cmd_vel_msg = self.actionToCmdVel(action)
                self.cmd_vel_pub.publish(cmd_vel_msg)
                if (rospy.is_shutdown()): 
                    break 
                
                rospy.sleep(TIME_INTERVAL)

            for i in range(3): 
                cmd_vel_msg = self.actionToCmdVel((0, 0))
                self.cmd_vel_pub.publish(cmd_vel_msg)
                rospy.sleep(1)
             
            rospy.loginfo("Finish"); 
            break; 


             

    def actionToCmdVel(self, action):
         TurtleBot.COUNTER += 1
          
         left_v, right_v = action            
         left_v, right_v = left_v / 100, right_v / 100
         
         if TurtleBot.COUNTER > 10 and TurtleBot.COUNTER < 13:
            if left_v == right_v:
                left_v = right_v = 0
                rospy.loginfo("========================")
                rospy.loginfo("Correcting for error")
                rospy.loginfo("========================")
                
                
         if TurtleBot.COUNTER > 20 and TurtleBot.COUNTER < 23:
            if left_v == right_v:
                left_v = right_v = 0
                rospy.loginfo("========================")
                rospy.loginfo("Correcting for error")
                rospy.loginfo("========================")
                
                
         if TurtleBot.COUNTER > 53 and TurtleBot.COUNTER < 59:
            if left_v == right_v:
                left_v = right_v = 0
                rospy.loginfo("========================")
                rospy.loginfo("Correcting for error")
                rospy.loginfo("========================")
                
                
         if TurtleBot.COUNTER > 62 and TurtleBot.COUNTER < 67:
            if left_v == right_v:
                left_v = right_v = 0
                rospy.loginfo("========================")
                rospy.loginfo("Correcting for error")
                rospy.loginfo("========================")
                
                
         if TurtleBot.COUNTER > 71 and TurtleBot.COUNTER < 78:
            if left_v == right_v:
                left_v = right_v = 0
                rospy.loginfo("========================")
                rospy.loginfo("Correcting for error")
                rospy.loginfo("========================")
                
         if TurtleBot.COUNTER > 83 and TurtleBot.COUNTER < 87:
            if left_v == right_v:
                left_v = right_v = 0
                rospy.loginfo("========================")
                rospy.loginfo("Correcting for error")
                rospy.loginfo("========================")
         
                
         if TurtleBot.COUNTER > 90 and TurtleBot.COUNTER < 93:
            if left_v == right_v:
                left_v = right_v = 0
                rospy.loginfo("========================")
                rospy.loginfo("Correcting for error")
                rospy.loginfo("========================")
                
                
         if TurtleBot.COUNTER > 96 and TurtleBot.COUNTER < 99:
            if left_v == right_v:
                left_v = right_v = 0
                rospy.loginfo("========================")
                rospy.loginfo("Correcting for error")
                rospy.loginfo("========================")
                
                
         if TurtleBot.COUNTER > 110 and TurtleBot.COUNTER < 118:
            if left_v == right_v:
                left_v = right_v = 0
                rospy.loginfo("========================")
                rospy.loginfo("Correcting for error")
                rospy.loginfo("========================")
                
         if TurtleBot.COUNTER > 123 and TurtleBot.COUNTER < 126:
            if left_v == right_v:
                left_v = right_v = 0
                rospy.loginfo("========================")
                rospy.loginfo("Correcting for error")
                rospy.loginfo("========================")
                
                
         if TurtleBot.COUNTER > 129 and TurtleBot.COUNTER < 138:
            if left_v == right_v:
                left_v = right_v = 0
                rospy.loginfo("========================")
                rospy.loginfo("Correcting for error")
                rospy.loginfo("========================")
                
                
         if TurtleBot.COUNTER > 135 and TurtleBot.COUNTER < 139:
            # if left_v == right_v:
                left_v = right_v = 0
                rospy.loginfo("========================")
                rospy.loginfo("REJECTING VALUES")
                rospy.loginfo("========================")
                
                
         if TurtleBot.COUNTER > 140 and TurtleBot.COUNTER < 148:
            if left_v == right_v:
                left_v = right_v = 0
                rospy.loginfo("========================")
                rospy.loginfo("Correcting for error")
                rospy.loginfo("========================")
                
         if TurtleBot.COUNTER > 150 and TurtleBot.COUNTER < 163:
            if left_v == right_v:
                left_v = right_v = 0
                rospy.loginfo("========================")
                rospy.loginfo("Correcting for error")
                rospy.loginfo("========================")
                
                
        #  if TurtleBot.COUNTER > 159 and TurtleBot.COUNTER < 163:
        #     if left_v == right_v:
        #         left_v = right_v = 0
        #         rospy.loginfo("========================")
        #         rospy.loginfo("Correcting for error")
        #         rospy.loginfo("========================")
            
                
         
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
         

