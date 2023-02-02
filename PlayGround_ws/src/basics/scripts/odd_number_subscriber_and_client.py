#!/usr/bin/python3.8

## Node to subscribe to odd_number_topic and client to positive_to_negative.

import rospy
from basics.msg import odd_number

import sys
from basics.srv import convert_positive_to_negative, convert_positive_to_negativeRequest, convert_positive_to_negativeResponse


def positive_to_negative_client(x):

    # First wait for the service to become available.
    rospy.loginfo("Waiting for service...")
    rospy.wait_for_service('positive_to_negative')
    try:
        # Create a service proxy.
        positive_to_negative = rospy.ServiceProxy('positive_to_negative', convert_positive_to_negative)

        # Call the service here.
        service_response = positive_to_negative(x)

        # Return the response to the calling function.
        return service_response

    except rospy.ServiceException:
        print("Service call failed. ServiceException error")


# Topic callback function.
def SubscriberCallback(data):

    even_number = data.odd_number_name + 1
    rospy.loginfo('Positive even number: {}'.format(even_number))

    #####################################################
    # client
    rospy.loginfo("Requesting conversion of pozitive %d to negative."%(even_number))

    # Call the service client function.
    service_response = positive_to_negative_client(even_number)
    
    rospy.sleep(1)
    # Process the service response and display log messages accordingly.
    if(service_response.success):
        rospy.loginfo("Conversion successful! Negative response is : {}".format(service_response.negative_number))
    else:
        rospy.logerr("Conversion unsuccessful! Requested number should be a positive real number.")
    #####################################################


def Listener():
    # subscriber
    rospy.init_node('odd_number_subscriber_and_positive_to_negative_client')
    rospy.loginfo("waiting for odd_number_topic to publish ...")
    rospy.Subscriber('odd_number_topic' , odd_number, SubscriberCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    Listener()
