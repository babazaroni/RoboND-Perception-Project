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

TEST_NUM = 2

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

def passthrough_filter(cloud, axis, min, max):
  filter = cloud.make_passthrough_filter()
  filter.set_filter_field_name(axis)
  filter.set_filter_limits(min, max)
  return filter.filter()

def do_euclidean_clustering(white_cloud,ClusterTolerance,MinClusterSize,MaxClusterSize):

    tree = white_cloud.make_kdtree()

    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(ClusterTolerance)
    ec.set_MinClusterSize(MinClusterSize)
    ec.set_MaxClusterSize(MaxClusterSize)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
    return cluster_indices



# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data

    pcl_cloud = ros_to_pcl(pcl_msg)
    
    # TODO: Statistical Outlier Filtering

    outlier_filter = pcl_cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(20)  # number of neighbor points.  try 20-50
    outlier_filter.set_std_dev_mul_thresh(0.5) #set average distance to be considered out of range  try .1-.5
    cloud_filtered = outlier_filter.filter()


    # TODO: Voxel Grid Downsampling

    vox = cloud_filtered.make_voxel_grid_filter()

    LEAF_SIZE = .005  # try .005 .01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter

    cloud_filtered = passthrough_filter(cloud_filtered,'z',.6,1.3)
    cloud_filtered = passthrough_filter(cloud_filtered,'x',.3,1.0)
    cloud_filtered = passthrough_filter(cloud_filtered,'y',-.5,.5)


    # TODO: RANSAC Plane Segmentation

    seg = cloud_filtered.make_segmenter()

    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    max_distance = .01  # try .01 .03 .034
    seg.set_distance_threshold(max_distance)

    inliers, coefficients = seg.segment()


    # TODO: Extract inliers and outliers

    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)


    # TODO: Euclidean Clustering

    white_cloud = XYZRGB_to_XYZ(cloud_objects) # Apply function to convert XYZRGB to XYZ


#    cluster_indices = do_euclidean_clustering(white_cloud,.01,200,15000)  # only gets 7 objects
    cluster_indices = do_euclidean_clustering(white_cloud,.01,20,15000)  # gets 8 objects

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

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

    ros_cloud_object_cluster = pcl_to_ros(cluster_cloud)

    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)


    # TODO: Publish ROS messages

    pcl_objects_cloud_pub.publish(ros_cloud_object_cluster)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)

    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster

        pcl_cluster = cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector

        color_hists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        normal_hists = compute_normal_histograms(normals)
        feature = np.concatenate((color_hists, normal_hists))


        # Make the prediction

        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)


        # Publish a label into RViz

        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.

        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects

    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(detected_object_list):

    # TODO: Initialize variables

    test_scene_num = Int32()
    test_scene_num.data = TEST_NUM

    object_name = String()
    object_group = String()

    centroids = []
    yaml_list = []


    # TODO: Get/Read parameters

    object_list_param = rospy.get_param('/object_list')
    dropbox_list = rospy.get_param('/dropbox')

    # TODO: Parse parameters into individual variables


    # TODO: Rotate PR2 in place to capture side tables for the collision map


    # TODO: Loop through the pick list

    for params in object_list_param:

        object_name.data = params['name']
        object_group = params['group']

#        print("object_name.data:",object_name.data)

        for detected_object in detected_object_list:
#            print("trying: ",str(object_name.data),str(detected_object.label))

            if str(object_name.data) != str(detected_object.label): #skip if not detected.  Glue is never detected
                 continue

#            print("found:",object_name.data)


            # TODO: Get the PointCloud for a given object and obtain it's centroid

#            labels.append(detected_object.label)
            points_arr = ros_to_pcl(detected_object.cloud).to_array()
            centroid = np.mean(points_arr, axis=0)[:3]
            centroids.append(centroid)

            # TODO: Create 'place_pose' for the object

            pose_place = Pose()
            pose_pick = Pose()

            pose_pick.position.x = np.asscalar(centroid[0]) 
            pose_pick.position.y = np.asscalar(centroid[1])
            pose_pick.position.z = np.asscalar(centroid[2])


#            print("centroid: ",centroid)

            place_pos = [0, 0, 0] # default placement  
            arm_name = String('left')  # defaul arm
            # go thru the drop boxes, looking for correct group/arm and destination position
            for box_params in dropbox_list:
                if box_params['group'] == object_group:
                    x, y, z = box_params['position']
                    pose_place.position.x = np.float(x) 
                    pose_place.position.y = np.float(y)
                    pose_place.position.z = np.float(z)
                    arm_name.data = box_params['name']
                    break

            # TODO: Assign the arm to be used for pick_plac


            # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format


            yaml = make_yaml_dict(test_scene_num, arm_name, object_name, pose_pick, pose_place)
            yaml_list.append(yaml)

            break  # break here is just making yaml files

            # Wait for 'pick_place_routine' service to come up

            rospy.wait_for_service('pick_place_routine')

            try:
                pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                # TODO: Insert your message variables to be sent as a service request

                resp = pick_place_routine(test_scene_num, object_name, arm_name, pose_pick, pose_place)
                print ("Response: ",resp.success)

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        # TODO: Output your request parameters into output yaml file

        send_to_yaml('output_%d.yaml' % TEST_NUM, yaml_list)




if __name__ == '__main__':

    # TODO: ROS node initialization

    rospy.init_node('clustering', anonymous=True)


    # TODO: Create Subscribers

    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers

    pcl_objects_cloud_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    object_markers_pub=rospy.Publisher("/object_markers",Marker,queue_size=1)
    detected_objects_pub=rospy.Publisher("/detected_objects",DetectedObjectsArray,queue_size=1)


    # TODO: Load Model From disk

    model = pickle.load(open('model_w' + str(TEST_NUM) + '.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown

    while not rospy.is_shutdown():
        rospy.spin()
