#!/usr/bin/python3.8

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the order_online_pizza action, including the
# goal message and the result message.
from basics.msg import order_online_pizzaAction, order_online_pizzaGoal


def feedback_cb(feedback_msg):
    print('Feedback received:', feedback_msg)


def order_online_pizza_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (order_online_pizzaAction) to the constructor.
    client = actionlib.SimpleActionClient('order_pizza', order_online_pizzaAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for action server to come up...")
    client.wait_for_server()

    pizza_counts = 10

    # Creates a goal to send to the action server.
    goal = order_online_pizzaGoal()
    goal.pizza_count = pizza_counts
    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=feedback_cb)

    rospy.loginfo("Goal has been sent to the action server. {} pizzas are ordered. Delicious!!".format(pizza_counts))

    # Waits for the server to finish performing the action.
    # client.wait_for_result()

    
    # You can something else while the action is being done:


    client.wait_for_result()
    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('order_pizza_action_client')
        result = order_online_pizza_client()
        rospy.loginfo("{}".format(result))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
