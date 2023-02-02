#!/usr/bin/python3.8
# node to transform positive number to negative

from basics.srv import convert_positive_to_negative, convert_positive_to_negativeRequest, convert_positive_to_negativeResponse
import rospy

# Service callback function.
def process_service_request(req):

    # Instantiate the response message object.
    res = convert_positive_to_negativeResponse()

    # Perform sanity check. Allow only positive real numbers.
    # Compose the response message accordingly.
    if(req.positive_number <= 0):
        res.success = False
        res.negative_number = 0
    else:
        res.negative_number = -req.positive_number
        res.success = True

    #Return the response message.
    return res

def positive_to_negative_server():
    # ROS node for the service server.
    rospy.init_node('positive_to_negative_server', anonymous = False)

    # Create a ROS service type.
    service = rospy.Service('positive_to_negative', convert_positive_to_negative, process_service_request)

    # Log message about service availability.
    rospy.loginfo('Convert positive to negative service is now available.')
    rospy.spin()

if __name__ == "__main__":
    positive_to_negative_server()
