#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
    # TODO: Convert ROS msg to PCL data
    pc = ros_to_pcl(pcl_msg)

    # TODO: Voxel Grid Downsampling
    vox = pc.make_voxel_grid_filter()
    LEAF_SIZE = 0.005
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    pc = vox.filter()
    # TODO: PassThrough Filter
    filter_axis = 'z'
    pass_through_z = pc.make_passthrough_filter()
    pass_through_z.set_filter_field_name(filter_axis)
    axis_min = 0.3
    axis_max = 5.0
    pass_through_z.set_filter_limits(axis_min, axis_max)  
    pc = pass_through_z.filter()
    filter_axis = 'x'
    pass_through_x = pc.make_passthrough_filter()
    pass_through_x.set_filter_field_name(filter_axis)
    axis_min = 0.34
    axis_max = 1.0
    pass_through_x.set_filter_limits(axis_min, axis_max)  
    pc = pass_through_x.filter()    
    outlier_filter = pc.make_statistical_outlier_filter()

    outlier_filter.set_mean_k(50)

    x = 0.5

    outlier_filter.set_std_dev_mul_thresh(x)

    pc = outlier_filter.filter()
    # TODO: RANSAC Plane Segmentation
    segmenter = pc.make_segmenter()
    segmenter.set_model_type(pcl.SACMODEL_PLANE)
    segmenter.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.015
    segmenter.set_distance_threshold(max_distance)
    # TODO: Extract inliers and outliers
    inliers, coefficients = segmenter.segment()
    ros_cloud_objects = pc.extract(inliers, negative=True)
    ros_cloud_table = pc.extract(inliers, negative=False)
    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(ros_cloud_objects)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(10000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    cluster_color = get_color_list(len(cluster_indices))
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                            rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    # TODO: Convert PCL data to ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(pcl_to_ros(ros_cloud_objects))
    pcl_table_pub.publish(pcl_to_ros(ros_cloud_table))
    pcl_segmentation_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs: 
    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = ros_cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)
        # Extract histogram features
        # TODO: complete this step just as is covered in capture_features.py
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .25
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()

    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass
    

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    TEST_SCENE_NUM = Int32()
    TEST_SCENE_NUM.data = 1
    OBJECT_NAME = String()
    WHICH_ARM = String()
    PICK_POSE = Pose()
    PLACE_POSE = Pose()
    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    # TODO: Parse parameters into individual variables
    labels = []
    centroids = [] # to be list of tuples (x, y, z)
    for object in object_list:
        labels.append(object.label)
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])
    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    dict_list = []
    for i in range(len(object_list_param)):
        target = object_list_param[i]
        OBJECT_NAME.data = target['name']
        #Get the PointCloud for a given object and obtain it's centroid
        for j in range(len(object_list)):
            curr_obj = object_list[j]
            if(curr_obj.label == OBJECT_NAME.data):
                centroid = centroids[j]
            # Create 'place_pose' for the object
                PICK_POSE.position.x = float(centroid[0])
                PICK_POSE.position.y = float(centroid[1])
                PICK_POSE.position.z = float(centroid[2])
            # Assign the arm to be used for pick_place
                PLACE_POSE.position.x = 0.0
                PLACE_POSE.position.z = 0.606

                if (object_list_param[i]['group'] == "red"):
                    WHICH_ARM.data = "left"            
                    PLACE_POSE.position.y = 0.71      
                else:
                    WHICH_ARM.data = "right"
                    PLACE_POSE.position.y = -0.71

                # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
                dic_yaml = make_yaml_dict(TEST_SCENE_NUM,WHICH_ARM,OBJECT_NAME,PICK_POSE,PLACE_POSE)
                dict_list.append(dic_yaml)
        # Wait for 'pick_place_routine' service to come up
        """
        rospy.wait_for_service('pick_place_routine')
        
        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        """

    # TODO: Output your request parameters into output yaml file
    print(dict_list)
    send_to_yaml("output_{}.yaml".format(TEST_SCENE_NUM.data), dict_list)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('pick_and_place', anonymous=True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)
    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_segmentation_pub = rospy.Publisher("/pcl_segmentation", PointCloud2, queue_size=1)
    detected_objects_pub = rospy.Publisher("/pcl_detected", DetectedObjectsArray, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
