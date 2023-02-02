#!/usr/bin/python3.8

# This code has been adapted from the ROS Wiki actionlib tutorials to the context
# of this course.
# (http://wiki.ros.org/hrwros_msgs/Tutorials)

import rospy

import actionlib

from basics.msg import order_online_pizzaAction, order_online_pizzaActionFeedback, order_online_pizzaActionResult

class OrderOnlinePizzaActionClass(object):

    # create messages that are used to publish feedback/result
    _feedback = order_online_pizzaActionFeedback()
    _result = order_online_pizzaActionResult()

    def __init__(self): 
        self.action_name = "order_pizza"
        # Create a simple action server of the newly defined action type and
        # specify the execute callback where the goal will be processed.
        self.a_server = actionlib.SimpleActionServer(
            self.action_name, order_online_pizzaAction, execute_cb=self.execute_cb, auto_start=False)

        # Start the action server.
        self.a_server.start()
        rospy.loginfo("Action server started...")

    def execute_cb(self, goal):

        time_for_baking_one_pizza = 1.0

        success = True

        rate = rospy.Rate(1)

        for i in range(0, goal.pizza_count):
            if self.a_server.is_preempt_requested():
                self.a_server.set_preempted()
                success = False
                break

            self._feedback.feedback.baking_status = str(i+1) + ". pizza baked !!"
            self.a_server.publish_feedback(self._feedback.feedback)
            rate.sleep()

        if success:
            self._result.result.result_message = "All pizzas are ready. Come and pick up !!"
            rospy.loginfo('%s: Succeeded' % self.action_name)
            self.a_server.set_succeeded(self._result.result)

if __name__ == '__main__':
    # Initialize a ROS node for this action server.
    rospy.init_node('order_pizza_action_server')

    # Create an instance of the action server here.
    action_server = OrderOnlinePizzaActionClass()
    rospy.spin()
