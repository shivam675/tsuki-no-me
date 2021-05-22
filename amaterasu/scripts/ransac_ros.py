#!/usr/bin/env python

# Import PCL module
import pcl
from pcl_helper import *


def pcl_callback(pcl_msg):
	#print(pcl_msg)
	cloud = ros_to_pcl(pcl_msg)
	vox = cloud.make_voxel_grid_filter()
	LEAF_SIZE = 0.006
	vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
	cloud_filtered = vox.filter()
	# passthrough = cloud_filtered.make_passthrough_filter()
	# #Assign axis and range to the passthrough filter object.
	# filter_axis = 'z'
	# passthrough.set_filter_field_name(filter_axis)
	# axis_min = 0.5
	# axis_max = 1.6
	# passthrough.set_filter_limits(axis_min, axis_max)


	# cloud_filtered = passthrough.filter()

	# seg = cloud_filtered.make_segmenter()

	# # Set the model you wish to fit 
	# seg.set_model_type(pcl.SACMODEL_PLANE)
	# seg.set_method_type(pcl.SAC_RANSAC)


	# max_distance = 0.002

	# seg.set_distance_threshold(max_distance)

	# # Call the segment function to obtain set of inlier indices and model coefficients
	# inliers, coefficients = seg.segment()
	# ################################################################
	# # Extract inliers
	# # Implement inlier extraction here
	# # Extract inliers
	# extracted_inliers = cloud_filtered.extract(inliers, negative=False)
	
	# # # ros_table = pcl_to_ros(extracted_inliers)
	# # # pcl_table_pub.publish(ros_table)
	# # # ################################################################

	# # # # Extract outliers

	# extracted_outliers = cloud_filtered.extract(inliers, negative=True)
	# extracted_outliers = pcl.PointCloud_PointXYZRGB()
	# # ros_objecs = pcl_to_ros(cloud_filtered)
	# # pcl_objects_pub.publish(ros_objecs)
	# rospy.loginfo('Publishing rn')
	# ##################################################################
	# ########################clustering################################
	white_cloud = XYZRGB_to_XYZ(cloud_filtered)
	#print(white_cloud)
	tree = white_cloud.make_kdtree()
	#print(tree)
	# Create a cluster extraction object
	ec = white_cloud.make_EuclideanClusterExtraction()
	# Set tolerances for distance threshold 
	# as well as minimum and maximum cluster size (in points)
	# NOTE: These are poor choices of clustering parameters
	# Your task is to experiment and find values that work for segmenting objects.
	ec.set_ClusterTolerance(0.04) # [m]
	ec.set_MinClusterSize(50)
	ec.set_MaxClusterSize(5000)
	# Search the k-d tree for clusters
	ec.set_SearchMethod(tree)
	# Extract indices for each of the discovered clusters
	cluster_indices = ec.Extract()

	# Create Cluster-Mask Point Cloud to visualize each cluster separately
	# Assign a color corresponding to each segmented object in scene
	cluster_color = get_color_list(len(cluster_indices))

	color_cluster_point_list = []
	#rospy.loginfo(cluster_indices)
	for j, indices in enumerate(cluster_indices):
		for i, indice in enumerate(indices):
			color_cluster_point_list.append([white_cloud[indice][0], white_cloud[indice][1], white_cloud[indice][2], rgb_to_float(cluster_color[j])])

	#Create new cloud containing all clusters, each with unique color
	cluster_cloud = pcl.PointCloud_PointXYZRGB()
	cluster_cloud.from_list(color_cluster_point_list)
	ros_cluster_cloud = pcl_to_ros(cluster_cloud)
	pcl_cluster_pub.publish(ros_cluster_cloud)

if __name__ == '__main__':

	# TODO: ROS node initialization
	rospy.init_node('clustering', anonymous=True)

	# TODO: Create Subscribers
	pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=5)

	# TODO: Create Publishers
	#pcl_objects_pub = rospy.Publisher("/cluster_in", PointCloud2, queue_size=1)
	#pcl_table_pub = rospy.Publisher("/pcl_table", pc2.PointCloud2, queue_size=1)
	
	pcl_cluster_pub = rospy.Publisher("/cluster_in", pc2.PointCloud2, queue_size=5)

	# Initialize color_list
	get_color_list.color_list = []

	# TODO: Spin while node is not shutdown
	while not rospy.is_shutdown():
		rospy.spin()