#!/usr/bin/env python
import rospy
import time
import random
from math import pi, sin, cos, acos
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


# topics for publishing:

# type: std_msgs/Float64
# msg: float64 data
# topics:
#     1st leg:
#         /gurdy/1_1_joint_position_controller/command  head & yaw
#         /gurdy/1_2_joint_position_controller/command  yaw & upper
#         /gurdy/1_3_joint_position_controller/command  upper & lower
#     2nd leg:
#         /gurdy/2_1_joint_position_controller/command  head & yaw
#         /gurdy/2_2_joint_position_controller/command  yaw & upper
#         /gurdy/2_3_joint_position_controller/command  upper & lower
#     3rd leg:
#         /gurdy/3_1_joint_position_controller/command  head & yaw
#         /gurdy/3_2_joint_position_controller/command  yaw & upper
#         /gurdy/3_3_joint_position_controller/command  upper & lower

# topics for subscribing:

# type: sensor_msgs/JointState
# msg:
#     Header header
#     string[] name
#     float64[] position
#     float64[] velocity
#     float64[] effort
# topics: /gurdy/joint_states


class gurdy_joint_mover():

    def __init__(self):

        # initiate a node controlling gurdy movement
        rospy.init_node('move_gurdy', anonymous=True)
        rospy.loginfo("Moving procedure is initiating")

        # define publishers for joints
        # attention:
        #   when using topic '/gurdy/1_1_joint_position_controller/command' as others,
        #   the msg get an 'Float64' value into 'data'
        #   this is absolute position of a joint
        #   i.e. if given value is -1.0, then the joint will reach the 'position' of -1.0
        #   not move a 'step' with length of 1.0
        self.pub_1st_head_yaw_joint = rospy.Publisher(
            '/gurdy/1_1_joint_position_controller/command', Float64, queue_size=1)
        self.pub_1st_yaw_upper_joint = rospy.Publisher(
            '/gurdy/1_2_joint_position_controller/command', Float64, queue_size=1)
        self.pub_1st_upper_lower_joint = rospy.Publisher(
            '/gurdy/1_3_joint_position_controller/command', Float64, queue_size=1)
        self.pub_2nd_head_yaw_joint = rospy.Publisher(
            '/gurdy/2_1_joint_position_controller/command', Float64, queue_size=1)
        self.pub_2nd_yaw_upper_joint = rospy.Publisher(
            '/gurdy/2_2_joint_position_controller/command', Float64, queue_size=1)
        self.pub_2nd_upper_lower_joint = rospy.Publisher(
            '/gurdy/2_3_joint_position_controller/command', Float64, queue_size=1)
        self.pub_3rd_head_yaw_joint = rospy.Publisher(
            '/gurdy/3_1_joint_position_controller/command', Float64, queue_size=1)
        self.pub_3rd_yaw_upper_joint = rospy.Publisher(
            '/gurdy/3_2_joint_position_controller/command', Float64, queue_size=1)
        self.pub_3rd_upper_lower_joint = rospy.Publisher(
            '/gurdy/3_3_joint_position_controller/command', Float64, queue_size=1)

        # define subscriber
        self.sub_joint_state = rospy.Subscriber(
            '/gurdy/joint_states', JointState, self.gurdy_joints_callback)
        gurdy_joints_data = None
        rate = rospy.Rate(2)
        while gurdy_joints_data is None:
            try:
                # in function "rospy.wait_for_massage()" there is a subscriber topic defined
                # and it return the massage from callback() of this topic
                # the content of returned massage is the same as msg constructure of this topic
                # in this case, the topic is '/gurdy/joint_states'
                # whose msg constructure is shown above
                # we only reserve two parts in this msg: "name" and "position"
                # details see "Go to definition"
                gurdy_joints_data = rospy.wait_for_message(
                    '/gurdy/joint_states', JointState, timeout=5)
            except:
                rospy.logwarn("Subscriber Topic time out: /gurdy/joint_states")
                pass
            rate.sleep()

        # save the msg content, when topic '/gurdy/joint_states' firstly has content
        self.gurdy_joint_dict = dict(
            zip(gurdy_joints_data.name, gurdy_joints_data.position))

    def move_gurdy_all_joints_with_angle(self, angle_1_1, angle_1_2, angle_1_3, angle_2_1, angle_2_2, angle_2_3, angle_3_1, angle_3_2, angle_3_3):

        # direct give the angles needed to be rotate in each joint
        # 1st leg
        move_1_1 = Float64()
        move_1_1.data = angle_1_1
        self.pub_1st_head_yaw_joint.publish(move_1_1)

        move_1_2 = Float64()
        move_1_2.data = angle_1_2
        self.pub_1st_yaw_upper_joint.publish(move_1_2)

        move_1_3 = Float64()
        move_1_3.data = angle_1_3
        self.pub_1st_upper_lower_joint.publish(move_1_3)

        # 2nd leg
        move_2_1 = Float64()
        move_2_1.data = angle_2_1
        self.pub_2nd_head_yaw_joint.publish(move_2_1)

        move_2_2 = Float64()
        move_2_2.data = angle_2_2
        self.pub_2nd_yaw_upper_joint.publish(move_2_2)

        move_2_3 = Float64()
        move_2_3.data = angle_2_3
        self.pub_2nd_upper_lower_joint.publish(move_2_3)

        # 3rd leg
        move_3_1 = Float64()
        move_3_1.data = angle_3_1
        self.pub_3rd_head_yaw_joint.publish(move_3_1)

        move_3_2 = Float64()
        move_3_2.data = angle_3_2
        self.pub_3rd_yaw_upper_joint.publish(move_3_2)

        move_3_3 = Float64()
        move_3_3.data = angle_3_3
        self.pub_3rd_upper_lower_joint.publish(move_3_3)

    def gurdy_joints_callback(self, msg):

        # update the content of reservation, when everytime the toipc '/gurdy/joint_states' has new content
        self.gurdy_joint_dict = dict(zip(msg.name, msg.position))

    def convert_angle_into_2pi(self, angle):

        # the desired angle of movement of joints are given as radian
        # e.g. move joint 90 degree clockwisely, given angle is -1.57
        # but sometimes given angles may be out of scare of [0, 2*pi]
        # e.g. move joint 540 degree is equal to move 180 degree,
        # and given angle 9.42 (3*pi) is equal to given 3.14 (pi)

        complete_rev = 2 * pi
        # e.g. 540 = 360 * 1 + 180 --> 180
        # extra = 1
        extra = int(angle / complete_rev)
        # clean_angle = 180
        clean_angle = angle - extra * complete_rev

        if clean_angle < 0:
            # make the result angle in [0, 2*pi]
            clean_angle += 2 * pi

        return clean_angle

    def assertAlmostEqualAngles(self, x, y,):
        c2 = (sin(x) - sin(y)) ** 2 + (cos(x) - cos(y)) ** 2
        angle_diff = acos((2.0 - c2) / 2.0)
        return angle_diff

    def check_if_arrive_desirede_angle(self, joint_name, desired_angle, error=0.1):

        # through joint_name, check the current state (angle) of this joint
        joint_state = self.gurdy_joint_dict.get(joint_name)
        similar = False

        # if there is nothing in this joint state
        if not joint_state:
            print("self.gurdy_joint_dict=" + str(self.gurdy_joint_dict))
            print("joint_name: " + str(joint_name))
            print("No data about this joint.")

        # check whether current state is equal enough to the desired angle
        clean_desired_angle = self.convert_angle_into_2pi(desired_angle)
        clean_joint_state = self.convert_angle_into_2pi(joint_state)
        clean_diff = self.assertAlmostEqualAngles(
            clean_desired_angle, clean_joint_state)
        if clean_diff <= error:
            similar = True

        return similar

    def movement_look(self, angle_1_1, angle_1_2, angle_1_3, angle_2_1, angle_2_2, angle_2_3, angle_3_1, angle_3_2, angle_3_3):

        # the joints:
        #     head_upperlegM1_yaw_joint  1_1
        #     head_upperlegM1_joint      1_2
        #     upperleg_lowerleg_M1_joint 1_3
        #     head_upperlegM2_yaw_joint  2_1
        #     head_upperlegM2_joint      2_2
        #     upperleg_lowerleg_M2_joint 2_3
        #     head_upperlegM3_yaw_joint  3_1
        #     head_upperlegM3_joint      3_2
        #     upperleg_lowerleg_M3_joint 3_3

        # the current states are not fulfilled at the beginning
        similar_1_1 = False
        similar_1_2 = False
        similar_1_3 = False
        similar_2_1 = False
        similar_2_2 = False
        similar_2_3 = False
        similar_3_1 = False
        similar_3_2 = False
        similar_3_3 = False

        # check 5 times in one second
        rate = rospy.Rate(5)

        while not (similar_1_1 and similar_1_2 and similar_1_3 and similar_2_1 and similar_2_2 and similar_2_3 and similar_3_1 and similar_3_2 and similar_3_3):

            # move all joints
            self.move_gurdy_all_joints_with_angle(
                angle_1_1, angle_1_2, angle_1_3, angle_2_1, angle_2_2, angle_2_3, angle_3_1, angle_3_2, angle_3_3)

            # check whether the joints fulfill the desired angle
            similar_1_1 = self.check_if_arrive_desirede_angle(
                'head_upperlegM1_yaw_joint', angle_1_1)
            similar_1_2 = self.check_if_arrive_desirede_angle(
                'head_upperlegM1_joint', angle_1_2)
            similar_1_3 = self.check_if_arrive_desirede_angle(
                'upperleg_lowerleg_M1_joint', angle_1_3)
            similar_2_1 = self.check_if_arrive_desirede_angle(
                'head_upperlegM2_yaw_joint', angle_2_1)
            similar_2_2 = self.check_if_arrive_desirede_angle(
                'head_upperlegM2_joint', angle_2_2)
            similar_2_3 = self.check_if_arrive_desirede_angle(
                'upperleg_lowerleg_M2_joint', angle_2_3)
            similar_3_1 = self.check_if_arrive_desirede_angle(
                'head_upperlegM3_yaw_joint', angle_3_1)
            similar_3_2 = self.check_if_arrive_desirede_angle(
                'head_upperlegM3_joint', angle_3_2)
            similar_3_3 = self.check_if_arrive_desirede_angle(
                'upperleg_lowerleg_M3_joint', angle_3_3)

            rate.sleep()

    def initial_movement(self):

        # head yaw joint:
        #     limit lower="-0.7" upper="0.7"
        # head upper joints:
        #     limit lower="-1.55" upper="0"
        # upper lower joints:
        #     limit lower="-2.9" upper="1.57"

        # the joints:
        #     head_upperlegM1_yaw_joint  1_1
        #     head_upperlegM1_joint      1_2
        #     upperleg_lowerleg_M1_joint 1_3
        #     head_upperlegM2_yaw_joint  2_1
        #     head_upperlegM2_joint      2_2
        #     upperleg_lowerleg_M2_joint 2_3
        #     head_upperlegM3_yaw_joint  3_1
        #     head_upperlegM3_joint      3_2
        #     upperleg_lowerleg_M3_joint 3_3

        angle_1_1 = 0
        angle_1_2 = -1.55
        angle_1_3 = 0
        angle_2_1 = 0
        angle_2_2 = -1.55
        angle_2_3 = 0
        angle_3_1 = 0
        angle_3_2 = -1.55
        angle_3_3 = 0
        self.movement_look(angle_1_1, angle_1_2, angle_1_3, angle_2_1,
                           angle_2_2, angle_2_3, angle_3_1, angle_3_2, angle_3_3)

        angle_1_3 = -1.55
        angle_2_3 = -1.55
        angle_3_3 = -1.55
        self.movement_look(angle_1_1, angle_1_2, angle_1_3, angle_2_1,
                           angle_2_2, angle_2_3, angle_3_1, angle_3_2, angle_3_3)

    def on_the_knee(self):

        angle_1_1 = 0
        angle_1_2 = -1
        angle_1_3 = 0
        angle_2_1 = 0
        angle_2_2 = -1
        angle_2_3 = 0
        angle_3_1 = 0
        angle_3_2 = -1
        angle_3_3 = 0
        self.movement_look(angle_1_1, angle_1_2, angle_1_3, angle_2_1,
                           angle_2_2, angle_2_3, angle_3_1, angle_3_2, angle_3_3)

        angle_1_3 = -2.9
        angle_2_3 = -2.9
        angle_3_3 = -2.9
        self.movement_look(angle_1_1, angle_1_2, angle_1_3, angle_2_1,
                           angle_2_2, angle_2_3, angle_3_1, angle_3_2, angle_3_3)

    def keep_moving(self):

        rospy.loginfo("Gurdy will move on!!!")
        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():

            self.initial_movement()
            rate.sleep()
            self.on_the_knee()
            rate.sleep()


if __name__ == '__main__':
    gurdy_move_on = gurdy_joint_mover()
    gurdy_move_on.keep_moving()
