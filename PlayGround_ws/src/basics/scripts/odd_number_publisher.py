#!/usr/bin/python3.8
## Node to publish to a odd number topic.

import rospy
from basics.msg import odd_number

def odd_number_publisher():
    si_publisher = rospy.Publisher('odd_number_topic', odd_number, queue_size = 10)
    rospy.init_node('odd_number_publisher', anonymous=False)
    rate = rospy.Rate(1)

    # Create a new object and fill in its contents.
    number = odd_number()
    
    counter = 1
    while not rospy.is_shutdown():
        # Fill in the number.
        number.odd_number_name = counter
        counter += 2

        # Publish the sensor information on the /sensor_info topic.
        si_publisher.publish(number)

        # Print log message if all went well.
        rospy.loginfo("All went well. {} is published".format(number.odd_number_name))
        rate.sleep()

if __name__ == '__main__':
    rospy.loginfo("All went well.")
    try:
        odd_number_publisher()
    except rospy.ROSInterruptException:
        pass
