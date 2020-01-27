import rospy
from std_msgs.msg._Bool import Bool

rospy.init_node('test_publisher')
move_to_tag_pub = rospy.Publisher('move_to_tag_start_driving', Bool, queue_size=1)

rate = rospy.Rate(2)
msg_str = Bool()
msg_str = True

i = 0

while i < 2 and not rospy.is_shutdown():

    pub.publish(msg_str)
    print "test"
    i = i + 1
    rate.sleep()

#pub.publish(msg_str)
#print "test"