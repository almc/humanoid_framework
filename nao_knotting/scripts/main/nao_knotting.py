# -*- encoding: UTF-8 -*-

''' nao_knotting: the nao robot creates different types of knots. '''

import sys, time, argparse
import nao_communication as nc
import math, almath, copy
import motion
import rospy, tf
import operator as op
import numpy as np
from threading import Thread
from nav_msgs.msg import nav_msgs
from geometry_msgs.msg import geometry_msgs
from ar_track_alvar_msgs.msg import AlvarMarkers
from visualization_msgs.msg import Marker
from sensor_msgs.msg import CameraInfo
import IPython

class Loops:
    def __init__(self):
        self.loop_ids = []
        self.loop_markers = []
        self.loop_positions = []
        self.loop_direction = []
        self.loop_path_anchor_pub = rospy.Publisher("/Loop_Paths_Anchor", nav_msgs.msg.Path, queue_size=100)
        self.loop_path_rope_pub = rospy.Publisher("/Loop_Paths_Rope", nav_msgs.msg.Path, queue_size=100)
        self.magn_path_pub = rospy.Publisher("/Magnetic_Paths", nav_msgs.msg.Path, queue_size=100)
        self.alpha = 1.0  # vectorfield parametrization parallel
        self.beta  = 1.0  # vectorfield parametrization perpendicular
        self.gamma = 0.02 # 0.05 / 0.02 anchoring entity / rope loop approach speed
        self.arm_trajectory = []

    # def reset_trajectories_rviz(self):
    #     self.publish_loop_path(1)
    #     self.publish_magnetic_field_path(np.zeros([3,3]))

    def add_loop_id(self, id, vertices, loop_markers, direction):
        self.loop_ids.append(id)
        self.loop_markers.append(loop_markers)
        self.loop_positions.append(vertices)
        self.loop_direction.append(direction)

    def update_loop_position(self, loop_id, vertices):
        loop_index = self.loop_ids.index(loop_id)
        self.loop_positions[loop_index] = vertices

    def publish_loop_path(self, loop_id, entity):
        loop_index = self.loop_ids.index(loop_id)
        vertices = self.loop_positions[loop_index]
        path = nav_msgs.msg.Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = '/base_footprint'
        path_length = np.size(vertices, 0)
        for i in range(path_length+1):
            idx = i%path_length
            pose = geometry_msgs.msg.PoseStamped()
            pose.header.frame_id = '/base_footprint'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position = geometry_msgs.msg.Point(vertices[idx,0], vertices[idx,1], vertices[idx,2])
            pose.pose.orientation = geometry_msgs.msg.Quaternion(0, 0, 0, 1)
            path.poses.append(pose)
        print("publishing loop path")
        if entity == "Anchor":
            self.loop_path_anchor_pub.publish(path)
        elif entity == "Rope":
            self.loop_path_rope_pub.publish(path)


    def publish_magnetic_field_path(self, vertices_list):
        vertices = np.array(vertices_list)
        path = nav_msgs.msg.Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = '/base_footprint'
        print 'field_array', vertices
        path_length = np.size(vertices, 0)
        for i in range(path_length):
            idx = i
            # print i
            pose = geometry_msgs.msg.PoseStamped()
            pose.header.frame_id = '/base_footprint'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position = geometry_msgs.msg.Point(vertices[idx,0], vertices[idx,1], vertices[idx,2])
            pose.pose.orientation = geometry_msgs.msg.Quaternion(0, 0, 0, 1)
            path.poses.append(pose)
        print("publishing magnetic field path")
        self.magn_path_pub.publish(path)

    def perpendicular_vector(self, v):
        if v[0] == v[1] == v[2] == 0:
            raise ValueError('zero-vector')
        if v[0] == 0:
            return np.array([1, 0, 0])
        if v[1] == 0:
            return np.array([0, 1, 0])
        if v[2] == 0:
            return np.array([0, 0, 1])
        return np.array([1, 1, -1.0 * (v[0] + v[1]) / v[2]])

    def planeFit(self, points):
        # p, n = planeFit(points). p, on the plane (the point-cloud centroid), and the normal, n.
        from numpy.linalg import svd
        points = np.reshape(points, (np.shape(points)[0], -1)) # Collapse trialing dimensions
        assert points.shape[0] <= points.shape[1], "There are only {} points in {} dimensions.".format(points.shape[1], points.shape[0])
        ctr = points.mean(axis=1)
        x = points - ctr[:,np.newaxis]
        M = np.dot(x, x.T) # Could also use np.cov(x) here.
        return ctr, svd(M)[0][:,-1]

    def calculate_magnetic_field(self, motionProxy, loop_id, effector, iterations=100, reverse_current=False):
        space      = motion.FRAME_ROBOT
        useSensorValues = True
        posCurrent = motionProxy.getPosition(effector, space, useSensorValues)

        loop_index = self.loop_ids.index(loop_id)
        vertices = self.loop_positions[loop_index]
        if reverse_current:
            vertices = np.flipud(vertices)

        loop_center_point, loop_perp_vector = self.planeFit(np.transpose(vertices))
        loop_paral_vector_1 = self.perpendicular_vector(loop_perp_vector)
        loop_paral_vector_1 = loop_paral_vector_1 / np.linalg.norm(loop_paral_vector_1)
        loop_paral_vector_2 = np.cross(loop_perp_vector, loop_paral_vector_1)
        loop_paral_vector_2 = loop_paral_vector_2 / np.linalg.norm(loop_paral_vector_2)
        # print "printing loop vectors: center, perpendicular, parallel 1 and parallel 2"
        # print loop_center_point
        # print loop_perp_vector
        # print loop_paral_vector_1
        # print loop_paral_vector_2

        print "initiating biot savart calculation"
        p = np.array(posCurrent)[0:3]
        arm_trajectory = []
        arm_trajectory.append(list(p) + [0,0,0])
        magnetic_field_max = 0.0
        for iter in range(iterations):
            magnetic_field = np.array([0.0, 0.0, 0.0])
            path_length = np.size(vertices, 0)
            for i in range(path_length):
                idx0 = i%path_length
                idx1 = (i+1)%path_length
                pl = np.array([np.mean([vertices[idx0,0],vertices[idx1,0]]),
                               np.mean([vertices[idx0,1],vertices[idx1,1]]),
                               np.mean([vertices[idx0,2],vertices[idx1,2]])])
                dl = np.array([vertices[idx1,0]-vertices[idx0,0],
                               vertices[idx1,1]-vertices[idx0,1],
                               vertices[idx1,2]-vertices[idx0,2]])
                r  = p - pl
                cp = np.cross(dl,r) / np.linalg.norm(r)**3
                magnetic_field = magnetic_field + cp;
            magnetic_field_norm = np.linalg.norm(magnetic_field)
            print "magnetic_field_max previous iteration", magnetic_field_max
            print "magnetic_field_norm current iteration", magnetic_field_norm
            # print magnetic_field_norm, magnetic_field_max
            if magnetic_field_norm > magnetic_field_max:
                magnetic_field_max = magnetic_field_norm
                magnetic_field = self.gamma * magnetic_field / np.linalg.norm(magnetic_field);
                p = p + magnetic_field
                arm_trajectory.append(list(p) + [0,0,0])
                print "p", p
            else:
                print "maximum magnetic field intensity reached at iteration:", iter
                break

        self.arm_trajectory = arm_trajectory
        self.publish_magnetic_field_path(arm_trajectory)
        return self.arm_trajectory

    def calculate_position_fixed_loop(self, marker_list, target_loop_id):
        if target_loop_id == 1: # loop_id 1 is defined to be the fixed anchoring entity
            target_loop_index = self.loop_ids.index(target_loop_id)
            marker_id = self.loop_markers[target_loop_index][0] # choose the first marker id of the list
            # print "marker_id", marker_id
            displacement_y = -0.03
            point_1 = np.array([ 0.00, displacement_y, 0.00])
            point_2 = np.array([-0.16, displacement_y, 0.00])
            point_3 = np.array([-0.16, displacement_y, 0.30])
            point_4 = np.array([ 0.00, displacement_y, 0.30])
            point_1 = point_1 + marker_list.get_marker_position(marker_id)
            point_2 = point_2 + marker_list.get_marker_position(marker_id)
            point_3 = point_3 + marker_list.get_marker_position(marker_id)
            point_4 = point_4 + marker_list.get_marker_position(marker_id)
            vertices = np.array([point_1, point_2, point_3, point_4])
            self.update_loop_position(target_loop_id, vertices)
            self.publish_loop_path(target_loop_id, "Anchor")
        elif target_loop_id == 2:
            target_loop_index = self.loop_ids.index(target_loop_id)
            marker_id = self.loop_markers[target_loop_index][0] # choose the first marker id of the list
            marker_position = marker_list.get_marker_position(marker_id)
            loop_radius = 0.08
            angles = np.linspace(0,2*np.pi,40)
            x = loop_radius*np.cos(angles) + marker_position[0] + loop_radius
            y = loop_radius*np.sin(angles) + marker_position[1]
            z = 0*angles                   + marker_position[2]
            vertices = np.column_stack([x,y,z])
            self.update_loop_position(target_loop_id, vertices)
            self.publish_loop_path(target_loop_id, "Rope")
        else:
            print("target_loop_id for this function should be 1 (anchoring entity) or 2 (rope loop)")

    def calculate_position_dynamic_loop(self, marker_list, target_loop_id):
        # use marker_list to define the vertices of the loop.
        # then save it to the corresponding loop_position and publish it
        if target_loop_id == 3: # loop_id 3 is defined to be the dynamic loop
            target_loop_index = self.loop_ids.index(target_loop_id)
            marker_id_1 = self.loop_markers[target_loop_index][0]
            marker_id_2 = self.loop_markers[target_loop_index][1]
            marker_id_3 = self.loop_markers[target_loop_index][2]
            marker_id_4 = self.loop_markers[target_loop_index][3]
            marker_id_5 = self.loop_markers[target_loop_index][4]
            point_1 = marker_list.get_marker_position(marker_id_1)
            point_2 = marker_list.get_marker_position(marker_id_2)
            point_3 = marker_list.get_marker_position(marker_id_3)
            point_4 = marker_list.get_marker_position(marker_id_4)
            point_5 = marker_list.get_marker_position(marker_id_5)
            vertices = np.array([point_1, point_2, point_3, point_4, point_5])
            self.update_loop_position(target_loop_id, vertices)
            self.publish_loop_path(target_loop_id, "Rope")
        else:
            print("target_loop_id for this function should be 3 (dynamic loop)")

class Markers:
    def __init__(self):
        self.marker_ids = []
        self.marker_positions = []
        self.marker_quaternions = []
        self.marker_seqs = []
        self.marker_times = []
        self.marker_frame_ids = []
        self.marker_confidences = []
        self.marker_located = []

        self.viz_marker_xyz_front = []
        self.viz_marker_xyz_bottom = []
        self.viz_marker_xy_front = []
        self.viz_marker_xy_bottom = []
        self.viz_marker_times_front = []
        self.viz_marker_times_bottom = []

        self.camera_info_frame_front = ""
        self.camera_info_frame_bottom = ""
        self.camera_info_K_front = np.zeros([3,3])
        self.camera_info_K_bottom = np.zeros([3,3])

    def add_marker_id(self, id):
        self.marker_ids.append(id)
        self.marker_positions.append(np.zeros(3))
        self.marker_quaternions.append(np.zeros(4))
        self.marker_seqs.append(0)
        self.marker_times.append(rospy.Time.now())
        self.marker_frame_ids.append("/unknown")
        self.marker_confidences.append(0)
        self.marker_located.append(False)
        self.viz_marker_xyz_front.append(np.zeros(3))
        self.viz_marker_xyz_bottom.append(np.zeros(3))
        self.viz_marker_xy_front.append(np.zeros(2))
        self.viz_marker_xy_bottom.append(np.zeros(2))
        self.viz_marker_times_front.append(rospy.Time.now())
        self.viz_marker_times_bottom.append(rospy.Time.now())

    def update_marker(self, marker_id, data):
        marker_index = self.marker_ids.index(marker_id)
        self.marker_positions[marker_index] = np.array([
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            data.pose.pose.position.z])
        self.marker_quaternions[marker_index] = np.array([
            data.pose.pose.orientation.x, data.pose.pose.orientation.y,
            data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        self.marker_seqs[marker_index] = data.header.seq
        self.marker_times[marker_index] = rospy.Time(data.header.stamp.to_time())
        self.marker_frame_ids[marker_index] = data.header.frame_id
        self.marker_confidences[marker_index] = data.confidence
        self.marker_located[marker_index] = True

    def get_marker_position(self, marker_id):
        marker_index = self.marker_ids.index(marker_id)
        return self.marker_positions[marker_index]

    def get_marker_quaternion(self, marker_id):
        marker_index = self.marker_ids.index(marker_id)
        return self.marker_quaternions[marker_index]

    def get_marker_located(self, marker_id):
        marker_index = self.marker_ids.index(marker_id)
        return self.marker_located[marker_index]

    def are_markers_located(self, marker_id_list):
        for marker_id in marker_id_list:
            marker_index = self.marker_ids.index(marker_id)
            if self.marker_located[marker_index] == False:
                return False
        return True

    def are_markers_visible(self, marker_id_list, camera):
        for marker_id in marker_id_list:
            marker_index = self.marker_ids.index(marker_id)
            if camera == "front":
                if (rospy.Time.now() - self.viz_marker_times_front[marker_index]) > rospy.Duration(0.5):
                    return False
            elif camera == "bottom":
                if (rospy.Time.now() - self.viz_marker_times_bottom[marker_index]) > rospy.Duration(0.5):
                    return False
        print('marker is visible')
        return True


def AlvarMarkersCallBack(data):
    if not data.markers:
        return
    else:
        global marker_list
        for marker_msg_index in range(len(data.markers)):
            data_tmp = data.markers[marker_msg_index]
            if data_tmp.id not in marker_list.marker_ids:
                print("confused localization marker detection, not in expected list")
                continue
            else:
                marker_list.update_marker(data_tmp.id, data_tmp)

def VisualizationMarkerCallBack(data):
    global marker_list
    if data.id not in marker_list.marker_ids:
        print("confused visualization marker detection, not in expected list")
        return
    marker_index = marker_list.marker_ids.index(data.id)
    if data.header.frame_id == marker_list.camera_info_frame_front:
        marker_list.viz_marker_xyz_front[marker_index] = np.array([data.pose.position.x,
                                                                   data.pose.position.y,
                                                                   data.pose.position.z])
        pixel = marker_list.camera_info_K_front*(marker_list.viz_marker_xyz_front[marker_index]
                                                /marker_list.viz_marker_xyz_front[marker_index][2])
        marker_list.viz_marker_xy_front[marker_index] = np.array([pixel[0,0], pixel[1,1]])
        marker_list.viz_marker_times_front[marker_index] = rospy.Time(data.header.stamp.to_time())

    if data.header.frame_id == marker_list.camera_info_frame_bottom:
        marker_list.viz_marker_xyz_bottom[marker_index] = np.array([data.pose.position.x,
                                                                   data.pose.position.y,
                                                                   data.pose.position.z])
        pixel = marker_list.camera_info_K_bottom*(marker_list.viz_marker_xyz_bottom[marker_index]
                                                 /marker_list.viz_marker_xyz_bottom[marker_index][2])
        marker_list.viz_marker_xy_bottom[marker_index] = np.array([pixel[0,0], pixel[1,1]])
        marker_list.viz_marker_times_bottom[marker_index] = rospy.Time(data.header.stamp.to_time())

def CameraInfoFrontCallBack(data):
    global marker_list
    marker_list.camera_info_frame_front = data.header.frame_id
    marker_list.camera_info_K_front = np.array([data.K[0:3], data.K[3:6], data.K[6:9]])

def CameraInfoBottomCallBack(data):
    global marker_list
    marker_list.camera_info_frame_bottom = data.header.frame_id
    marker_list.camera_info_K_bottom = np.array([data.K[0:3], data.K[3:6], data.K[6:9]])

def robot_balance(motionProxy, On):
    motionProxy.wbEnable(On)
    motionProxy.wbFootState("Fixed", "Legs")
    motionProxy.wbEnableBalanceConstraint(On, "Legs")

def get_init_pose_angles(motionProxy):
    bodyAngles = [   0.009161949157714844,
                     0.042910099029541016,
                     1.4342480897903442,
                     0.2622721195220947,
                     -1.3745059967041016,
                     -1.0139319896697998,
                     0.018366098403930664,
                     0.2531999945640564,
                     4.1961669921875e-05,
                     -0.0014920234680175781,
                     -0.4494199752807617,
                     0.7009961605072021,
                     -0.3497939109802246,
                     0.0015759468078613281,
                     4.1961669921875e-05,
                     4.1961669921875e-05,
                     -0.4510378837585449,
                     0.7010800838470459,
                     -0.3543119430541992,
                     -0.0014920234680175781,
                     1.418992042541504,
                     -0.25621986389160156,
                     1.3744220733642578,
                     0.9894719123840332,
                     -0.013848066329956055,
                     0.25279998779296875  ]
    return bodyAngles

def get_stretch_pose_angles(motionProxy):
    bodyAngles = [  -0.010779857635498047,
                     0.04137611389160156,
                     1.498676061630249,
                     0.5798101425170898,
                     -1.5785279273986816,
                     -0.780764102935791,
                     -0.08594608306884766,
                     0.6995999813079834,
                     4.1961669921875e-05,
                     4.1961669921875e-05,
                     -0.4494199752807617,
                     0.6994619369506836,
                     -0.35132789611816406,
                     4.1961669921875e-05,
                     4.1961669921875e-05,
                     4.1961669921875e-05,
                     -0.4510378837585449,
                     0.7010800838470459,
                     -0.34970998764038086,
                     4.1961669921875e-05,
                     1.5846638679504395,
                     -0.5522818565368652,
                     1.7333780527114868,
                     0.8130619525909424,
                     0.2929520606994629,
                     0.7139999866485596  ]
    return bodyAngles

def get_regrasp_pose_angles_trefoil_knot(motionProxy):
    leftArmAngles0 = [   0.47856616973876953,
                         0.43254613876342773,
                         -0.6059720516204834,
                         -1.294654130935669,
                         -0.76857590675354,
                         0.9703999757766724 ]

    leftArmAngles1 = [   0.3911280632019043,
                         -0.3141592741012573,
                         -0.3988819122314453,
                         -0.8543961048126221,
                         -0.9127721786499023,
                         0.9703999757766724 ]

    rightArmAngles0 = [  0.6756660461425781,  # necesarry to get closer,
                         0.11807608604431152, #left arm does not reach
                         0.7909219741821289,
                         0.9802680015563965,
                         0.2822141647338867,
                         0.8384000062942505]
    return [leftArmAngles0, leftArmAngles1, rightArmAngles0]

def get_regrasp_pose_angles_zero_knot(motionProxy):
    leftArmAngles0 = [   1.2072160243988037,
                         -0.05373191833496094,
                         -0.9097042083740234,
                         -0.8390560150146484,
                         -0.995607852935791,
                         0.9703999757766724 ]

    leftArmAngles1 = [   0.7899680137634277,
                         -0.3141592741012573,
                         -1.0983858108520508,
                         -0.6626460552215576,
                         -1.2272419929504395,
                         0.9699999690055847 ]

    return [leftArmAngles0, leftArmAngles1]

def robot_enable(motionProxy, postureProxy, speechProxy, handsClosed):
    speechProxy.setVolume(1.0)
    speechProxy.setParameter("pitchShift", 0.0)
    speechProxy.setParameter("doubleVoice", 0.0)
    motionProxy.setFallManagerEnabled(False) # very important
    motionProxy.setStiffnesses("Body", 1.0)
    # motionProxy.wakeUp() # set up motor stiffness and goes to stand init
    if handsClosed:
        close_hand("RHand", motionProxy)
        close_hand("LHand", motionProxy)
        time.sleep(0.5)
    if handsClosed:
        # print("setting body angles in robot enable, close hands")
        set_body_angles(get_init_pose_angles(motionProxy), motionProxy, True, 0.0, 0.0)
    else:
        set_body_angles(get_init_pose_angles(motionProxy), motionProxy, True, 1.0, 1.0) # hands open
    robot_balance(motionProxy, True)
    return motionProxy.getAngles("Body", False)

def robot_disable(motionProxy, postureProxy):
    postureProxy.goToPosture("StandInit", 0.2)
    motionProxy.wbEnable(False)
    motionProxy.rest()
    return motionProxy.getAngles("Body", False)

def close_hand(hand, motionProxy):
    print("closing hand")
    motionProxy.setStiffnesses(hand, 1.0)
    # time.sleep(0.1)
    fractionMaxSpeed = 0.2
    # motionProxy.angleInterpolationWithSpeed(hand, 0.0, fractionMaxSpeed)
    time.sleep(0.5)
    motionProxy.angleInterpolation(hand, [0.0, 0.0], [0.5, 1], True)
    # motionProxy.setAngles(hand, 0.0, fractionMaxSpeed)
    # time.sleep(0.1)
    return motionProxy.getAngles("Body", False)

def open_hand(hand, motionProxy):
    print("opening hand")
    motionProxy.setStiffnesses(hand, 1.0)
    # time.sleep(0.1)
    fractionMaxSpeed = 0.2
    # motionProxy.angleInterpolationWithSpeed(hand, 1.0, fractionMaxSpeed)
    motionProxy.angleInterpolation(hand, [1.0, 1.0], [0.5, 1], True)
    # motionProxy.setAngles(hand, 1.0, fractionMaxSpeed)
    # time.sleep(0.1)
    return motionProxy.getAngles("Body", False)

def check_hand(hand, motionProxy, memoryProxy, speechProxy):
    hand_without_sensors = motionProxy.getAngles(hand, False)
    hand_with_sensors = motionProxy.getAngles(hand, True)
    hand_current = memoryProxy.getData("Device/SubDeviceList/"+hand+"/ElectricCurrent/Sensor/Value")
    if hand == "RHand":
        threshold = 0.02
    elif hand == "LHand":
        threshold = 0.05
    print(hand_current)
    # print(hand_without_sensors)
    # print(hand_with_sensors)
    check = hand_current > threshold
    if check:
        print("successfully grasped the object")
        speechProxy.say("successfully grasped the object")
    else:
        print("failed to grasp the object")
        speechProxy.say("failed to grasp the object")
    return check

def get_hand_index_config(motionProxy, hand, hand_action):
    Joints = motionProxy.getBodyNames("Body")
    index = Joints.index(hand)
    if hand_action == "Close":
        config = 0.0
    elif hand_action == "Open":
        config = 1.0
    return index, config

def get_other_arm(arm):
    if arm == "RArm":
        return "LArm"
    elif arm == "LArm":
        return "RArm"

def set_body_angles(initialAngles, motionProxy, override_hands=False, hand_left=0.0, hand_right=0.0):
    Joints = motionProxy.getBodyNames("Body")
    index_r_hand = Joints.index("RHand")
    index_l_hand = Joints.index("LHand")
    currentAngles = motionProxy.getAngles("Body", False)
    outputAngles = copy.deepcopy(initialAngles)
    if override_hands == True:
        outputAngles[index_r_hand] = hand_right
        outputAngles[index_l_hand] = hand_left
    else:
        outputAngles[index_r_hand] = currentAngles[index_r_hand]
        outputAngles[index_l_hand] = currentAngles[index_l_hand]
    # motionProxy.angleInterpolationWithSpeed("Body", outputAngles, 0.1)
    time.sleep(0.5)
    motionProxy.angleInterpolation("Body", outputAngles, [2]*len(outputAngles), True)

def set_arm_angles(initialAngles, motionProxy, arm, hand, override_hands=False, hand_angle=0.0):
    Joints = motionProxy.getBodyNames(arm)
    index_hand = Joints.index(hand)
    currentAngles = motionProxy.getAngles(arm, False)
    outputAngles = copy.deepcopy(initialAngles)
    if override_hands == True:
        outputAngles[index_hand] = hand_angle
    else:
        outputAngles[index_hand] = currentAngles[index_hand]
    # motionProxy.angleInterpolationWithSpeed(arm, outputAngles, 0.1)
    time.sleep(0.5)
    motionProxy.angleInterpolation(arm, outputAngles, [2]*len(outputAngles), True)


def rotate_wrist(hand, angle, motionProxy):
    if hand == "RHand":
        wrist = "RWristYaw"
    elif hand == "LHand":
        wrist = "LWristYaw"
    motionProxy.setStiffnesses(wrist, 1.0)
    fractionMaxSpeed = 0.2 # angle = -1 #-1.8238 to 1.8238
    # print("rotating wrist")
    # print(hand)
    # print(angle)
    motionProxy.angleInterpolationWithSpeed(wrist, angle, fractionMaxSpeed)
    time.sleep(0.5)
    return motionProxy.getAngles("Body", False)

def rotate_head(motionProxy, names, changes, fractionMaxSpeed, yaw_direction, pitch_direction):
    # print("markers not located yet, moving head")
    motionProxy.changeAngles(names, changes, fractionMaxSpeed)
    time.sleep(0.2)
    commandAngles = motionProxy.getAngles(names, False)
    if (commandAngles[0] > math.radians(40)) and yaw_direction == "left":
        changes[0] = changes[0] * (-1)
        yaw_direction = "right"
    elif (commandAngles[0] < math.radians(-40)) and yaw_direction == "right":
        changes[0] = changes[0] * (-1)
        yaw_direction = "left"
    if (commandAngles[1] > math.radians(20)) and pitch_direction == "down":
        changes[1] = changes[1] * (-1)
        pitch_direction = "up"
    elif (commandAngles[1] < math.radians(-20)) and pitch_direction == "up":
        changes[1] = changes[1] * (-1)
        pitch_direction = "down"
    return changes, yaw_direction, pitch_direction


def find_markers(marker_list, motionProxy, locate, locate_ids, visualize, visualize_ids, visualize_camera):
    motionProxy.setStiffnesses("Head", 1.0)
    names  = ["HeadYaw", "HeadPitch"]
    fractionMaxSpeed  = 0.05
    # angles  = [0.0, 0.0]
    # motionProxy.angleInterpolationWithSpeed(names, angles, fractionMaxSpeed)
    time.sleep(0.5)
    changes = [0.2, 0.1]
    yaw_direction = "left"
    pitch_direction = "down"
    while locate and (marker_list.are_markers_located(locate_ids) == False):
        # print('locating markers')
        changes, yaw_direction, pitch_direction = rotate_head(motionProxy, names, changes,
                                                  fractionMaxSpeed, yaw_direction, pitch_direction)
    while visualize and (marker_list.are_markers_visible(visualize_ids, visualize_camera) == False):
        # print('visualizing markers')
        changes, yaw_direction, pitch_direction = rotate_head(motionProxy, names, changes,
                                                  fractionMaxSpeed, yaw_direction, pitch_direction)


def track_marker(marker_list, motionProxy, marker_id, camera, continuous):
    motionProxy.setStiffnesses("Head", 1.0)
    global run_thread
    while not rospy.is_shutdown() and run_thread:
        marker_index = marker_list.marker_ids.index(marker_id)
        if camera == "front":
            error_xy = marker_list.viz_marker_xy_front[marker_index]
        elif camera == "bottom":
            error_xy = marker_list.viz_marker_xy_bottom[marker_index]
        names  = ["HeadYaw", "HeadPitch"]
        Kx, Ky = 0.001, 0.001
        changes  = [-Kx*error_xy[0], +Ky*error_xy[1]]
        fractionMaxSpeed  = 0.02
        motionProxy.changeAngles(names, changes, fractionMaxSpeed)
        time.sleep(0.1)
        # print(abs(error_xy))
        if (not continuous) and (abs(error_xy[0]) < 5) and (abs(error_xy[1]) < 5):
            print("marker stabilized")
            break




def move_arm(marker_list, loop_list, effector, hand, angles, motionProxy, position_marker, quaternion_marker,
             target_loop_id, delta_pos_marker, delta_pos_hand, abs_pos_hand, hand_action, following):

    initialAngles = motionProxy.getAngles("Body", False)

    space      = motion.FRAME_ROBOT
    axisMask   = almath.AXIS_MASK_VEL
    isAbsolute = True
    useSensorValues = True

    rotate_wrist("LHand", angles['wrist_yaw_left'],  motionProxy)
    rotate_wrist("RHand", angles['wrist_yaw_right'], motionProxy)

    posCurrent = motionProxy.getPosition(effector, space, useSensorValues)
    dwx, dwy, dwz = 0.0, 0.0, 0.0

    if target_loop_id != 0: # insertion through fixed target loop (1 for anchoring entity, 2 for rope loop)
        if target_loop_id == 1 or target_loop_id == 2:
            loop_list.calculate_position_fixed_loop(marker_list, target_loop_id)
            arm_trajectory = loop_list.calculate_magnetic_field(motionProxy, target_loop_id, effector, 100, False)
            times = [2 + 2*x for x in range(np.size(arm_trajectory,0))]
        elif target_loop_id == 3: # insertion through dynamic deformable target loop (3 for both)
            loop_list.calculate_position_dynamic_loop(marker_list, target_loop_id)
            arm_trajectory = loop_list.calculate_magnetic_field(motionProxy, target_loop_id, effector, 100, False)
            if np.size(arm_trajectory,0) < 5: # we are close enough to the center
                return "outside"
            times = [2.0, 4.0]
            arm_trajectory = arm_trajectory[1:3] # [0]: current position, [1]: 1 step of the magnetic field
            print "control loop magnetic field dynamic"
            # print("arm_trajectory")
            # print(arm_trajectory)
            # print(len(arm_trajectory))
    elif delta_pos_hand != [0,0,0]: # move wrt other hand
        posCurrent_other_hand = motionProxy.getPosition(get_other_arm(effector), space, useSensorValues)[0:3]
        position_target = map(op.add, posCurrent_other_hand, delta_pos_hand)
        position_intermediate = [np.mean([posCurrent[0],position_target[0]]), np.mean([posCurrent[1],position_target[1]]), 1.2*position_target[2]]
        posTarget = list(position_target) + [dwx, dwy, dwz]
        posIntermediate = list(position_intermediate) + [dwx, dwy, dwz]
        times = [4.0, 8.0]
        arm_trajectory = [posIntermediate, posTarget]
    elif delta_pos_marker != [0,0,0]: # move wrt marker
        position_target = map(op.add, position_marker, delta_pos_marker)
        position_intermediate = [np.mean([posCurrent[0],position_target[0]]), np.mean([posCurrent[1],position_target[1]]), 1.2*position_target[2]]
        posTarget = list(position_target) + [dwx, dwy, dwz]
        posIntermediate = list(position_intermediate) + [dwx, dwy, dwz]
        times = [4.0, 8.0]
        arm_trajectory = [posIntermediate, posTarget]
    elif abs_pos_hand != [0,0,0]: # synergestic move wrt body
        position_target = abs_pos_hand
        position_intermediate = [np.mean([posCurrent[0],position_target[0]]),
                                 np.mean([posCurrent[1],position_target[1]]),
                                 np.mean([posCurrent[2],position_target[2]])]
        posTarget = list(position_target) + [dwx, dwy, dwz]
        posIntermediate = list(position_intermediate) + [dwx, dwy, dwz]
        times = [4.0, 8.0]
        arm_trajectory = [posIntermediate, posTarget]

    motionProxy.positionInterpolation(effector, space, arm_trajectory,
                                      axisMask, times, isAbsolute)

    rotate_wrist("LHand", angles['wrist_yaw_left'],  motionProxy)
    rotate_wrist("RHand", angles['wrist_yaw_right'], motionProxy)

    if hand_action == "Close":
        close_hand(hand, motionProxy)
    elif hand_action == "Open":
        open_hand(hand, motionProxy)

    if following == "Withdraw":
        path       = [posIntermediate, posCurrent]
        motionProxy.positionInterpolation(effector, space, path,
                                  axisMask, times, isAbsolute)
        hand_index, hand_config = get_hand_index_config(motionProxy, hand, hand_action)
        initialAngles[hand_index] = hand_config
        motionProxy.angleInterpolationWithSpeed("Body", initialAngles, 0.02)
    elif following == "Stay":
        pass

    return "normal"

def move(marker_list, loop_list, target_marker, target_loop_id, delta_pos_marker, delta_pos_hand, abs_pos_hand,
         arm, hand, angles, hand_action, following, camera, check_grasp, motionProxy, memoryProxy, speechProxy,
         blind, visual_servoing, dynamic_mode=False):
    if not blind:
        print("visualizing marker with camera")
        find_markers(marker_list, motionProxy, False, marker_list.marker_ids, True, target_marker, camera)
        print("stabilizing marker")
        track_marker(marker_list, motionProxy, target_marker[0], camera, False)
    if visual_servoing:
        print("starting visual servoing tracking")
        global run_thread
        run_thread = True
        thread_track_marker = Thread(target = track_marker, args = (marker_list, motionProxy, target_marker[0], camera, True))
        thread_track_marker.start()
    print "dynamic_mode", dynamic_mode
    if dynamic_mode == True:
        print "entering dynamic mode"
        insert_result = "outside"
        while insert_result != "inside":
            print insert_result
            position_marker = marker_list.get_marker_position(target_marker[0])
            quaternion_marker = marker_list.get_marker_quaternion(target_marker[0])
            insert_result = move_arm(marker_list, loop_list, arm, hand, angles, motionProxy, position_marker, quaternion_marker,
                                     target_loop_id, delta_pos_marker, delta_pos_hand, abs_pos_hand, hand_action, following)
            print "next loop"
    elif not target_marker: # synergestic control, wrt body
        position_marker = []
        quaternion_marker = []
        move_arm(marker_list, loop_list, arm, hand, angles, motionProxy, position_marker, quaternion_marker,
                 target_loop_id, delta_pos_marker, delta_pos_hand, abs_pos_hand, hand_action, following)
    elif (marker_list.get_marker_located(target_marker[0])): # marker based, wrt other hand, loop, etc.
        position_marker = marker_list.get_marker_position(target_marker[0])
        quaternion_marker = marker_list.get_marker_quaternion(target_marker[0])
        move_arm(marker_list, loop_list, arm, hand, angles, motionProxy, position_marker, quaternion_marker,
                 target_loop_id, delta_pos_marker, delta_pos_hand, abs_pos_hand, hand_action, following)
    else:
        print("marker necessary has not been located or synergestic mode not requested.")
    if visual_servoing:
        print("terminating visual servoing")
        run_thread = False
        thread_track_marker.join()

    if check_grasp: # see if grasp was successful
        object_in_hand = check_hand(hand, motionProxy, memoryProxy, speechProxy)
        return object_in_hand
    else:
        return True # assumed success on insertion or simple motions

def make_crossing(motionProxy, postureProxy, memoryProxy, speechProxy, marker_list, loop_list, initialAngles):
    # 0: twist rope 1, 1: twist rope 2, 2: receive rope
    bodyAngles0 = [  -0.004643917083740234,
                     0.03523993492126465,
                     0.5690720081329346,
                     0.15949392318725586,
                     -0.39427995681762695,
                     -1.383626103401184,
                     0.7316761016845703,
                     0.7888000011444092,
                     -0.05824995040893555,
                     -0.019900083541870117,
                     -0.5123140811920166,
                     1.0338740348815918,
                     -0.5492138862609863,
                     0.018450021743774414,
                     -0.05824995040893555,
                     0.015382051467895508,
                     -0.5123980045318604,
                     1.0339579582214355,
                     -0.5475959777832031,
                     -0.01683211326599121,
                     1.2333779335021973,
                     -0.20713186264038086,
                     0.46169209480285645,
                     0.9127721786499023,
                     1.0430779457092285,
                     0.9731999635696411  ]
    bodyAngles1 = [  -0.004643917083740234,
                     0.024502038955688477,
                     0.4233419895172119,
                     -0.3141592741012573,
                     -0.15804386138916016,
                     -1.294654130935669,
                     -1.089181900024414,
                     0.97079998254776,
                     -0.19170808792114258,
                     0.09054803848266602,
                     -0.5184500217437744,
                     1.0354080200195312,
                     -0.524669885635376,
                     -0.09966802597045898,
                     -0.19170808792114258,
                     -0.09353208541870117,
                     -0.5200679302215576,
                     1.035491943359375,
                     -0.5245859622955322,
                     0.10281991958618164,
                     0.8529460430145264,
                     0.3141592741012573,
                     0.09813404083251953,
                     1.0170841217041016,
                     0.7255401611328125,
                     0.8659999966621399]
    bodyAngles2 = [  -0.01691603660583496,
                     0.04137611389160156,
                     1.3145960569381714,
                     -0.07827591896057129,
                     -1.5815958976745605,
                     -1.2394300699234009,
                     0.018366098403930664,
                     0.9703999757766724,
                     4.1961669921875e-05,
                     -0.0014920234680175781,
                     -0.4494199752807617,
                     0.6994619369506836,
                     -0.3497939109802246,
                     0.0015759468078613281,
                     4.1961669921875e-05,
                     4.1961669921875e-05,
                     -0.4510378837585449,
                     0.6995458602905273,
                     -0.34970998764038086,
                     -0.0014920234680175781,
                     1.4711480140686035,
                     -0.11509203910827637,
                     1.6582120656967163,
                     1.3100781440734863,
                     0.02603602409362793,
                     0.7799999713897705  ]
    close_angle, open_angle = 0.0, 1.0
    set_body_angles(bodyAngles0, motionProxy, True, close_angle, close_angle)
    set_body_angles(bodyAngles1, motionProxy, True, close_angle, close_angle)
    print("please, hold the crossing point")
    speechProxy.say("please, hold the crossing point.")
    time.sleep(2)
    open_hand("RHand", motionProxy)
    open_hand("LHand", motionProxy)

    set_body_angles(initialAngles, motionProxy, True, 1.0, 1.0) # return home position
    set_body_angles(bodyAngles2, motionProxy, True, open_angle, open_angle)
    print("please, place the rope back in my hands")
    speechProxy.say("please, place the rope back in my hands.")
    time.sleep(2)
    close_hand("RHand", motionProxy)
    time.sleep(2)
    close_hand("LHand", motionProxy)


def main_dynamic_insertion(robotIP, robotPORT, marker_list, loop_list):
    motionProxy, postureProxy, memoryProxy, speechProxy = nc.init_proxies(robotIP, robotPORT)
    initialAngles = robot_enable(motionProxy, postureProxy, speechProxy, True)

    print("initiating insertion task")
    speechProxy.say("initiating insertion task.")

    print("i will insert the rope through a dynamic target loop")
    speechProxy.say("i will insert the rope through a dynamic target loop.")

    print("locating all the required markers")
    speechProxy.say("locating all the required markers.")
    find_markers(marker_list, motionProxy, True, marker_list.marker_ids, False, [], "")

    print("going for the insertion without visual servoing")
    speechProxy.say("going for the insertion without visual servoing.")
    target_marker = [1]
    target_loop_id = 3 # 0: no insertion, 1: anchoring entity, 2: rope loop, 3: dynamic
    angles = {'wrist_yaw_left': +0.0, 'wrist_yaw_right': +0.0}
    delta_pos_marker = [0, 0, 0]
    delta_pos_hand = [0, 0, 0]
    abs_pos_hand = [0, 0, 0]
    object_in_hand = move(marker_list, loop_list, target_marker, target_loop_id,
                          delta_pos_marker, delta_pos_hand, abs_pos_hand, "RArm", "RHand", angles,
                          "Close", "Stay", "front", False, motionProxy, memoryProxy, speechProxy, True, False, True)

    print("insertion completed successfully")
    speechProxy.say("insertion completed successfully.")
    time.sleep(1)

    print("resting now")
    speechProxy.say("resting now.")
    finalAngles = robot_disable(motionProxy, postureProxy)



def main_zero_knot(robotIP, robotPORT, marker_list, loop_list):
    motionProxy, postureProxy, memoryProxy, speechProxy = nc.init_proxies(robotIP, robotPORT)
    initialAngles = robot_enable(motionProxy, postureProxy, speechProxy, False)

    time.sleep(2)
    print("initiating knotting task")
    speechProxy.say("initiating knotting task.")
    open_hand("RHand", motionProxy)
    open_hand("LHand", motionProxy)


    print("locating all the required markers")
    speechProxy.say("locating all the required markers.")
    find_markers(marker_list, motionProxy, True, marker_list.marker_ids, False, [], "")

    print("estimating position of the anchoring entity")
    speechProxy.say("estimating position of the anchoring entity.")
    loop_list.calculate_position_fixed_loop(marker_list, 1)

    print("going for the pick with visual servoing")
    speechProxy.say("going for the pick with visual servoing.")
    target_marker = [8]
    target_loop_id = 0 # 0: no insertion, 1: anchoring entity, 2: rope loop, 3: dynamic
    angles = {'wrist_yaw_left': +0.15, 'wrist_yaw_right': -1.2}
    delta_pos_marker = [-0.02, -0.06, -0.02]
    delta_pos_hand = [0, 0, 0]
    abs_pos_hand = [0, 0, 0]
    object_in_hand = move(marker_list, loop_list, target_marker, target_loop_id,
                          delta_pos_marker, delta_pos_hand, abs_pos_hand, "RArm", "RHand", angles,
                          "Close", "Withdraw", "bottom", True, motionProxy, memoryProxy, speechProxy, False, True)
    # close_hand("RHand", motionProxy)

    print("going for the insertion with visual servoing")
    speechProxy.say("going for the insertion with visual servoing.")
    target_marker = [7]
    target_loop_id = 1 # 0: no insertion, 1: anchoring entity, 2: rope loop, 3: dynamic
    angles = {'wrist_yaw_left': +0.15, 'wrist_yaw_right': -1.0}
    delta_pos_marker = [0.10, -0.14, 0.35]
    # delta_pos_marker = [-0.3, -0.08, 0.20]
    delta_pos_hand = [0, 0, 0]
    abs_pos_hand = [0, 0, 0]
    object_in_hand = move(marker_list, loop_list, target_marker, target_loop_id,
                          delta_pos_marker, delta_pos_hand, abs_pos_hand, "RArm", "RHand", angles,
                          "Close", "Stay", "bottom", False, motionProxy, memoryProxy, speechProxy, False, True)


    print("regrasping on the other side with my left arm")
    speechProxy.say("regrasping rope on the other side with my left arm.")
    # had to hardcode this because the NAO is unable to localize reliably his own hand.
    set_arm_angles(get_regrasp_pose_angles_zero_knot(motionProxy)[0], motionProxy, "LArm", "LHand", True, 1.0)
    set_arm_angles(get_regrasp_pose_angles_zero_knot(motionProxy)[1], motionProxy, "LArm", "LHand", True, 1.0)
    close_hand("LHand", motionProxy)

    # robot_balance(motionProxy, False) # to prevent the rest of the body from moving
    # target_marker = [7]
    # target_loop_id = 0 # 0: no insertion, 1: anchoring entity, 2: rope loop, 3: dynamic
    # angles = {'wrist_yaw_left': +0.10, 'wrist_yaw_right': -1.2}
    # delta_pos_marker = [-0.02, -0.02, 0.28]
    # # delta_pos_marker = [-0.3, -0.04, 0.20]
    # # delta_pos_hand = [-0.02, 0.05, 0]
    # delta_pos_hand = [0, 0, 0]
    # abs_pos_hand = [0, 0, 0]
    # object_in_hand = move(marker_list, loop_list, target_marker, target_loop_id,
    #                       delta_pos_marker, delta_pos_hand, abs_pos_hand, "LArm", "LHand", angles,
    #                       "Close", "Stay", "bottom", True, motionProxy, memoryProxy, speechProxy, True, False)

    print("releasing right hand grasp")
    speechProxy.say("releasing right hand grasp.")
    open_hand("RHand", motionProxy)
    robot_balance(motionProxy, True)

    print("returning to home position")
    speechProxy.say("returning to home position.")
    set_body_angles(initialAngles, motionProxy) # return home position


    print("going for the grasp on the other end of the rope to complete the unknot")
    speechProxy.say("going for the grasp on the other end of the rope to complete the zero knot.")
    target_marker = [8]
    target_loop_id = 0 # 0: no insertion, 1: anchoring entity, 2: rope loop, 3: dynamic
    angles = {'wrist_yaw_left': +0.15, 'wrist_yaw_right': -1.2}
    delta_pos_marker = [-0.04, -0.07, +0.11]
    delta_pos_hand = [0, 0, 0]
    abs_pos_hand = [0, 0, 0]
    object_in_hand = move(marker_list, loop_list, target_marker, target_loop_id,
                          delta_pos_marker, delta_pos_hand, abs_pos_hand, "RArm", "RHand", angles,
                          "Close", "Withdraw", "bottom", True, motionProxy, memoryProxy, speechProxy, False, True)

    print("zero knot completed successfully")
    speechProxy.say("zero knot completed successfully.")
    time.sleep(1)
    print("resting now")
    speechProxy.say("resting now.")
    finalAngles = robot_disable(motionProxy, postureProxy)


def main_trefoil_knot(robotIP, robotPORT, marker_list, loop_list):
    motionProxy, postureProxy, memoryProxy, speechProxy = nc.init_proxies(robotIP, robotPORT)
    initialAngles = robot_enable(motionProxy, postureProxy, speechProxy, True)

    print("initiating knotting task")
    speechProxy.say("initiating knotting task.")

    print("i will continue from the zero knot to make the trefoil")
    speechProxy.say("i will continue from the zero knot to make the trefoil.")

    print("twisting the rope to make a crossing")
    speechProxy.say("twisting the rope to make a crossing.")

    make_crossing(motionProxy, postureProxy, memoryProxy, speechProxy, marker_list, loop_list, initialAngles)


    print("locating all the required markers")
    speechProxy.say("locating all the required markers.")
    find_markers(marker_list, motionProxy, True, marker_list.marker_ids, False, [], "")

    print("estimating position of the anchoring entity")
    speechProxy.say("estimating position of the anchoring entity.")
    loop_list.calculate_position_fixed_loop(marker_list, 1)

    print("estimating position of the rope loop")
    speechProxy.say("estimating position of the rope loop.")
    loop_list.calculate_position_fixed_loop(marker_list, 2)

    # robot_balance(motionProxy, True)

    print("going for the insertion without visual servoing")
    speechProxy.say("going for the insertion without visual servoing.")
    target_marker = [0]
    target_loop_id = 2 # 0: no insertion, 1: anchoring entity, 2: rope loop, 3: dynamic
    angles = {'wrist_yaw_left': 0.00, 'wrist_yaw_right': 0.00}
    delta_pos_marker = [0.0, 0.0, 0.0]
    delta_pos_hand = [0, 0, 0]
    abs_pos_hand = [0, 0, 0]
    object_in_hand = move(marker_list, loop_list, target_marker, target_loop_id,
                          delta_pos_marker, delta_pos_hand, abs_pos_hand, "RArm", "RHand", angles,
                          "Close", "Stay", "bottom", False, motionProxy, memoryProxy, speechProxy, True, False)


    print("releasing left hand")
    speechProxy.say("releasing left hand.")
    open_hand("LHand", motionProxy)

    print("regrasping on the other side with my left arm")
    speechProxy.say("regrasping rope on the other side with my left arm.")
    # had to hardcode this because the NAO is unable to localize reliably his own hand.
    set_arm_angles(get_regrasp_pose_angles_trefoil_knot(motionProxy)[0], motionProxy, "LArm", "LHand", True, 1.0)
    set_arm_angles(get_regrasp_pose_angles_trefoil_knot(motionProxy)[1], motionProxy, "LArm", "LHand", True, 1.0)
    set_arm_angles(get_regrasp_pose_angles_trefoil_knot(motionProxy)[2], motionProxy, "RArm", "RHand", True, 0.0)
    close_hand("LHand", motionProxy)

    # robot_balance(motionProxy, False) # to prevent the rest of the body from moving
    # target_marker = [0]
    # target_loop_id = 0 # 0: no insertion, 1: anchoring entity, 2: rope loop, 3: dynamic
    # angles = {'wrist_yaw_left': 0.0, 'wrist_yaw_right': 0.0}
    # delta_pos_marker = [0.00, -0.12, 0.05]
    # # delta_pos_hand = [ -0.01, 0.0, 0.02]
    # abs_pos_hand = [0, 0, 0]
    # object_in_hand = move(marker_list, loop_list, target_marker, target_loop_id,
    #                       delta_pos_marker, delta_pos_hand, abs_pos_hand, "LArm", "LHand", angles,
    #                       "Close", "Stay", "bottom", True, motionProxy, memoryProxy, speechProxy, True, False)

    print("releasing right hand")
    speechProxy.say("releasing right hand.")
    open_hand("RHand", motionProxy)
    # robot_balance(motionProxy, True)

    print("returning to home position")
    speechProxy.say("returning to home position.")
    set_body_angles(initialAngles, motionProxy, True, 0.0, 1.0)

    print("please, place the other end of the rope back in my right hand")
    speechProxy.say("please, place the other end of the rope back in my right hand.")
    time.sleep(2)
    close_hand("RHand", motionProxy)

    print("trefoil knot completed successfully")
    speechProxy.say("trefoil knot completed successfully.")
    # set_body_angles(get_stretch_pose_angles(motionProxy), motionProxy, True, 0.0, 0.0)
    time.sleep(1)

    print("resting now")
    speechProxy.say("resting now.")
    finalAngles = robot_disable(motionProxy, postureProxy)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="luigi.local",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")
    args = parser.parse_args()

    rospy.init_node('nao_knotting')

    run_thread = True
    marker_list = Markers()

    # marker_list.add_marker_id(0)
    marker_list.add_marker_id(1)
    marker_list.add_marker_id(2)
    marker_list.add_marker_id(3)
    marker_list.add_marker_id(4)
    marker_list.add_marker_id(5)
    # marker_list.add_marker_id(6)
    # marker_list.add_marker_id(7)
    # marker_list.add_marker_id(8)

    loop_list = Loops()
    # loop_list.add_loop_id(1, np.zeros([4,3]), [6], 1) # anchoring entity (fixed)
    # loop_list.add_loop_id(2, np.zeros([4,3]), [0], 2) # rope loop (fixed)
    loop_list.add_loop_id(3, np.zeros([5,3]), [1,2,3,4,5], 3)
    # loop_list.reset_trajectories_rviz()

    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, AlvarMarkersCallBack)
    rospy.Subscriber("/visualization_marker", Marker, VisualizationMarkerCallBack)
    rospy.Subscriber("/nao_robot/camera/front/camera_info",  CameraInfo, CameraInfoFrontCallBack)
    rospy.Subscriber("/nao_robot/camera/bottom/camera_info", CameraInfo, CameraInfoBottomCallBack)

    # main_zero_knot(args.ip, args.port, marker_list, loop_list)
    # main_trefoil_knot(args.ip, args.port, marker_list, loop_list)
    main_dynamic_insertion(args.ip, args.port, marker_list, loop_list)

    #rospy.spin()
