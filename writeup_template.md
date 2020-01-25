## Project: Perception Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

    def pcl_callback(pcl_msg):
	    #Convert ROS msg to PCL data
	    pc = ros_to_pcl(pcl_msg)

	    #Voxel Grid Downsampling
	    vox = pc.make_voxel_grid_filter()
	    LEAF_SIZE = 0.005
	    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
	    pc = vox.filter()
	    #PassThrough Filter
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
	    #RANSAC Plane Segmentation
	    segmenter = pc.make_segmenter()
	    segmenter.set_model_type(pcl.SACMODEL_PLANE)
	    segmenter.set_method_type(pcl.SAC_RANSAC)
	    max_distance = 0.015
	    segmenter.set_distance_threshold(max_distance)
	    #Extract inliers and outliers
	    inliers, coefficients = segmenter.segment()
	    ros_cloud_objects = pc.extract(inliers, negative=True)
	    ros_cloud_table = pc.extract(inliers, negative=False)

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  
    white_cloud = XYZRGB_to_XYZ(ros_cloud_objects)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(10000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    cluster_color = get_color_list(len(cluster_indices))
    #Create Cluster-Mask Point Cloud to visualize each cluster separately
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
    #Convert PCL data to ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    #Publish ROS messages
    pcl_objects_pub.publish(pcl_to_ros(ros_cloud_objects))
    pcl_table_pub.publish(pcl_to_ros(ros_cloud_table))
    pcl_segmentation_pub.publish(ros_cluster_cloud)
#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.


    def compute_color_histograms(cloud, using_hsv=False):

	    # Compute histograms for the clusters
	    point_colors_list = []

	    # Step through each point in the point cloud
	    for point in pc2.read_points(cloud, skip_nans=True):
		rgb_list = float_to_rgb(point[3])
		if using_hsv:
		    point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
		else:
		    point_colors_list.append(rgb_list)

	    # Populate lists with color values
	    channel_1_vals = []
	    channel_2_vals = []
	    channel_3_vals = []

	    for color in point_colors_list:
		channel_1_vals.append(color[0])
		channel_2_vals.append(color[1])
		channel_3_vals.append(color[2])
	    
	    # TODO: Compute histograms
	    nbins = 32
	    rng = (0,256)
	    channel1_hist = np.histogram(channel_1_vals,bins=nbins,range=rng)
	    channel2_hist = np.histogram(channel_2_vals,bins=nbins,range=rng)
	    channel3_hist = np.histogram(channel_3_vals,bins=nbins,range=rng)

	    # TODO: Concatenate and normalize the histograms
	    hist_features = np.concatenate((channel1_hist[0], channel2_hist[0], channel3_hist[0])).astype(np.float64)
	    # Generate random features for demo mode.  
	    # Replace normed_features with your feature vector
	    normed_features = hist_features/np.sum(hist_features) 
	    return normed_features 


    def compute_normal_histograms(normal_cloud):
	    norm_x_vals = []
	    norm_y_vals = []
	    norm_z_vals = []

	    for norm_component in pc2.read_points(normal_cloud,
		                                  field_names = ('normal_x', 'normal_y', 'normal_z'),
		                                  skip_nans=True):
		norm_x_vals.append(norm_component[0])
		norm_y_vals.append(norm_component[1])
		norm_z_vals.append(norm_component[2])

	    # TODO: Compute histograms of normal values (just like with color)
	    nbins=32
	    normx_hist = np.histogram(norm_x_vals, bins=nbins)
	    normy_hist = np.histogram(norm_y_vals, bins=nbins)
	    normz_hist = np.histogram(norm_z_vals, bins=nbins)
	    # TODO: Concatenate and normalize the histograms
	    hist_features = np.concatenate((normx_hist[0], normy_hist[0], normz_hist[0])).astype(np.float64)
	    # Generate random features for demo mode.  
	    # Replace normed_features with your feature vector
	    normed_features = hist_features / np.sum(hist_features)

	    return normed_features
	   
	   
**Object Recognition:**

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



### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

![](/home/mohamed/imgs/test1.png) 
![](/home/mohamed/imgs/test2.png) 
![](/home/mohamed/imgs/test3.png) 




