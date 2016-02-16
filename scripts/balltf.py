#!/usr/bin/env python  
    import roslib
    roslib.load_manifest('learning_tf')
    import rospy
    
    import tf

    geometry_msgs.msg import TransformStamped
    
    
    def handle_ball_pose(msg, turtlename):
       br = tf.TransformBroadcaster()
       self.tfMessage = TransformStamped()

       br.sendTransform((msg.x, msg.y, 0),
       tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                        rospy.Time.now(),
                        ballname,
                        "world")
   
     if __name__ == '__main__':
       rospy.init_node('ball_tf_broadcaster')
       ballname = rospy.get_param('~ball')
       rospy.Subscriber('/%s/pose' % ballname,
                        geometry_msgs.msg.TransformStamped,
                        handle_ball_pose,ballname)
	
       rospy.spin()

                    
