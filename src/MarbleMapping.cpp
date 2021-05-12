/*
 * Original work Copyright (c) 2010-2013, A. Hornung, University of Freiburg
 * Modified work Copyright 2020 Dan Riley
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <MarbleMapping.h>

using namespace octomap;
using octomap_msgs::Octomap;

bool is_equal (double a, double b, double epsilon = 1.0e-7)
{
    return std::abs(a - b) < epsilon;
}

namespace marble_mapping{

MarbleMapping::MarbleMapping(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_)
: m_nh(nh_),
  m_nh_private(private_nh_),
  m_pointCloudSub(NULL),
  m_tfPointCloudSub(NULL),
  m_reconfigureServer(m_config_mutex, private_nh_),
  m_octree(NULL),
  m_maxRange(-1.0),
  m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
  m_colorFactor(0.8),
  m_latchedTopics(true),
  m_res(0.05),
  m_treeDepth(0),
  m_maxTreeDepth(0),
  m_pointcloudMinX(-std::numeric_limits<double>::max()),
  m_pointcloudMaxX(std::numeric_limits<double>::max()),
  m_pointcloudMinY(-std::numeric_limits<double>::max()),
  m_pointcloudMaxY(std::numeric_limits<double>::max()),
  m_pointcloudMinZ(-std::numeric_limits<double>::max()),
  m_pointcloudMaxZ(std::numeric_limits<double>::max()),
  m_occupancyMinZ(-std::numeric_limits<double>::max()),
  m_occupancyMaxZ(std::numeric_limits<double>::max()),
  m_filterSpeckles(false), m_filterGroundPlane(false),
  m_groundFilterDistance(0.04), m_groundFilterAngle(0.15), m_groundFilterPlaneDistance(0.07),
  m_initConfig(true)
{
  double probHit, probMiss, thresMin, thresMax;

  m_nh_private.param("input", m_input, 1);
  m_nh_private.param("frame_id", m_worldFrameId, m_worldFrameId);
  m_nh_private.param("base_frame_id", m_baseFrameId, m_baseFrameId);
  m_nh_private.param("color_factor", m_colorFactor, m_colorFactor);

  m_nh_private.param("pointcloud_min_x", m_pointcloudMinX,m_pointcloudMinX);
  m_nh_private.param("pointcloud_max_x", m_pointcloudMaxX,m_pointcloudMaxX);
  m_nh_private.param("pointcloud_min_y", m_pointcloudMinY,m_pointcloudMinY);
  m_nh_private.param("pointcloud_max_y", m_pointcloudMaxY,m_pointcloudMaxY);
  m_nh_private.param("pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
  m_nh_private.param("pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);
  m_nh_private.param("occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
  m_nh_private.param("occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);

  m_nh_private.param("compress_maps", m_compressMaps, true);
  m_nh_private.param("num_threads", m_numThreads, 1);
  m_nh_private.param("downsample_size", m_downsampleSize, 0.0);
  m_nh_private.param("pcl_time_limit", m_pclTimeLimit, 100.0);

  m_nh_private.param("filter_speckles", m_filterSpeckles, m_filterSpeckles);
  m_nh_private.param("filter_ground", m_filterGroundPlane, m_filterGroundPlane);
  // distance of points from plane for RANSAC
  m_nh_private.param("ground_filter/distance", m_groundFilterDistance, m_groundFilterDistance);
  // angular derivation of found plane:
  m_nh_private.param("ground_filter/angle", m_groundFilterAngle, m_groundFilterAngle);
  // distance of found plane from z=0 to be detected as ground (e.g. to exclude tables)
  m_nh_private.param("ground_filter/plane_distance", m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);

  m_nh_private.param("sensor_model/max_range", m_maxRange, m_maxRange);

  m_nh_private.param("resolution", m_res, m_res);
  m_nh_private.param("merged_resolution", m_mres, 0.2);
  m_nh_private.param("sensor_model/hit", probHit, 0.7);
  m_nh_private.param("sensor_model/miss", probMiss, 0.4);
  m_nh_private.param("sensor_model/min", thresMin, 0.12);
  m_nh_private.param("sensor_model/max", thresMax, 0.97);
  m_nh_private.param("publish_duration", pub_duration, 0.1);
  m_nh_private.param("publish_merged_binary", m_publishMergedBinaryMap, false);
  m_nh_private.param("publish_merged_full", m_publishMergedFullMap, false);

  // Enable multiagent functions such as diff creation and merging
  m_nh_private.param("multiagent", m_multiagent, false);
  // Enable diff publishing (diffs are still created)
  m_nh_private.param("publish_diffs", m_publishDiffs, false);
  // Threshold for when to add map diffs
  m_nh_private.param("diff_threshold", diff_threshold, 1000);
  // How many seconds to check/publish diffs
  m_nh_private.param("diff_duration", diff_duration, 10.0);
  m_nh_private.param("diff_merged", m_diffMerged, false);

  if (m_filterGroundPlane && (m_pointcloudMinZ > 0.0 || m_pointcloudMaxZ < 0.0)){
    ROS_WARN_STREAM("You enabled ground filtering but incoming pointclouds will be pre-filtered in ["
              <<m_pointcloudMinZ <<", "<< m_pointcloudMaxZ << "], excluding the ground level z=0. "
              << "This will not work.");
  }

  // initialize octomap object & params
  m_octree = new OcTreeT(m_res);
  m_octree->setProbHit(probHit);
  m_octree->setProbMiss(probMiss);
  m_octree->setClampingThresMin(thresMin);
  m_octree->setClampingThresMax(thresMax);

  #pragma omp parallel
  #pragma omp master
  {
    // Set the number of threads, limited to maximum threads allowed by OMP_NUM_THREADS
    if ((m_numThreads == 0) || (m_numThreads > omp_get_max_threads()))
      m_numThreads = omp_get_num_threads();
    ROS_INFO("Using %d threads of %d available", m_numThreads, omp_get_max_threads());
    keyrays.resize(m_numThreads);
  }
  omp_set_num_threads(m_numThreads);

  // Merged OcTree
  m_merged_tree = new OcTreeStamped(m_mres);
  m_merged_tree->setProbHit(probHit);
  m_merged_tree->setProbMiss(probMiss);
  m_merged_tree->setClampingThresMin(thresMin);
  m_merged_tree->setClampingThresMax(thresMax);
  m_treeDepth = m_merged_tree->getTreeDepth();
  m_maxTreeDepth = m_treeDepth;

  // Use change detection to build diff tree
  if (m_diffMerged)
    m_merged_tree->enableChangeDetection(true);
  else
    m_octree->enableChangeDetection(true);

  // Differences OcTree, for sharing between agents
  m_diff_tree = new OcTreeT(m_mres);
  m_diff_tree->setProbHit(probHit);
  m_diff_tree->setProbMiss(probMiss);
  m_diff_tree->setClampingThresMin(thresMin);
  m_diff_tree->setClampingThresMax(thresMax);

  // Camera View OcTree, to show what has been seen by secondary sensor(s)
  m_camera_tree = new OcTreeT(m_mres);
  m_camera_tree->setProbHit(probHit);
  m_camera_tree->setProbMiss(probMiss);
  m_camera_tree->setClampingThresMin(thresMin);
  m_camera_tree->setClampingThresMax(thresMax);

  num_diffs = 0;
  next_idx = 2;

  double r, g, b, a;
  m_nh_private.param("color/r", r, 0.0);
  m_nh_private.param("color/g", g, 0.0);
  m_nh_private.param("color/b", b, 1.0);
  m_nh_private.param("color/a", a, 1.0);
  m_color.r = r;
  m_color.g = g;
  m_color.b = b;
  m_color.a = a;

  m_nh_private.param("color_free/r", r, 0.0);
  m_nh_private.param("color_free/g", g, 1.0);
  m_nh_private.param("color_free/b", b, 0.0);
  m_nh_private.param("color_free/a", a, 1.0);
  m_colorFree.r = r;
  m_colorFree.g = g;
  m_colorFree.b = b;
  m_colorFree.a = a;

  // Get secondary camera view's height, width and range and whether to publish
  double vfov_angle, hfov_angle;
  m_nh_private.param("build_camera_map", m_buildCameraMap, false);
  m_nh_private.param("publish_camera_map", m_publishCameraMap, false);
  m_nh_private.param("publish_camera_view", m_publishCameraView, false);
  m_nh_private.param("camera_range", camera_range, 5.0);
  m_nh_private.param("camera_horizontal_fov", hfov_angle, 60.0);
  m_nh_private.param("camera_vertical_fov", vfov_angle, 20.0);
  double hfov_rad = hfov_angle * M_PI / 180.0;
  double vfov_rad = vfov_angle * M_PI / 180.0;
  camera_h = 2.0 * tan(vfov_rad / 2.0) * camera_range;
  camera_w = 2.0 * tan(hfov_rad / 2.0) * camera_range;

  m_nh_private.param("publish_optional_duration", pub_opt_duration, 0.2);
  m_nh_private.param("publish_marker_array", m_publishMarkerArray, false);
  m_nh_private.param("publish_free_space", m_publishFreeSpace, false);
  m_nh_private.param("publish_point_cloud", m_publishPointCloud, false);
  m_nh_private.param("publish_point_cloud_diff", m_publishPointCloudDiff, false);
  m_nh_private.param("publish_neighbor_maps", m_publishNeighborMaps, false);

  m_nh_private.param("latch", m_latchedTopics, m_latchedTopics);

  m_nh_private.param("remove_ceiling", m_removeCeiling, false);
  m_nh_private.param("remove_ceiling_depth", m_removeCeilingDepth, 4);

  // Diff merging parameters
  m_nh_private.param("agents", m_agents, std::string("X1,X2"));
  m_nh_private.param("diff_pre", m_diff_pre, std::string("/robot_data/"));
  m_nh_private.param("diff_post", m_diff_post, std::string("/map_diff"));

  bool pclInput = false;
  bool octomapInput = false;
  bool diffInput = false;

  if (m_input == 1) pclInput = true;
  else if (m_input == 2) octomapInput = true;
  else if (m_input == 3) diffInput = true;

  // Normal octomap format map publishers
  if (pclInput) {
    m_binaryMapPub = m_nh.advertise<Octomap>("octomap_binary", 1, m_latchedTopics);
    m_fullMapPub = m_nh.advertise<Octomap>("octomap_full", 1, m_latchedTopics);
  }

  // Merged map publishers
  if (m_publishMergedBinaryMap)
    m_mergedBinaryMapPub = m_nh.advertise<Octomap>("merged_map", 1, m_latchedTopics);
  if (m_publishMergedFullMap)
    m_mergedFullMapPub = m_nh.advertise<Octomap>("merged_map_full", 1, m_latchedTopics);

  // Diff map publishers
  if (m_publishDiffs) {
    m_diffMapPub = m_nh.advertise<Octomap>("map_diff", 1, m_latchedTopics);
    m_diffsMapPub = m_nh.advertise<marble_mapping::OctomapArray>("map_diffs", 1, m_latchedTopics);
  }

  if (m_publishCameraMap)
    m_cameraMapPub = m_nh.advertise<Octomap>("camera_map", 1, m_latchedTopics);

  // Optional marker array and point cloud publishers
  if (m_publishMarkerArray)
    m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>("merged_map_markers", 1, m_latchedTopics);
  if (m_publishFreeSpace)
    m_fmarkerPub = m_nh.advertise<visualization_msgs::MarkerArray>("merged_map_free", 1, m_latchedTopics);
  if (m_publishPointCloud)
    m_pointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("merged_map_pc", 1, m_latchedTopics);
  if (m_publishPointCloudDiff)
    m_pointCloudDiffPub = m_nh.advertise<sensor_msgs::PointCloud2>("pc_diff", 1, m_latchedTopics);

  if (m_publishCameraView) {
    m_cameraViewPub = m_nh.advertise<visualization_msgs::Marker>("camera_view", 1, m_latchedTopics);
    m_cameraView.header.frame_id = "world";
    m_cameraView.id = 0;
    m_cameraView.scale.x = 1;
    m_cameraView.scale.y = 1;
    m_cameraView.scale.z = 1;
    m_cameraView.pose.position.x = 0.0;
    m_cameraView.pose.position.y = 0.0;
    m_cameraView.pose.position.z = 0.0;
    m_cameraView.pose.orientation.x = 0.0;
    m_cameraView.pose.orientation.y = 0.0;
    m_cameraView.pose.orientation.z = 0.0;
    m_cameraView.pose.orientation.w = 1.0;
    m_cameraView.color.a = 0.85;
    m_cameraView.color.r = 0.0;
    m_cameraView.color.g = 191.0/255.0;
    m_cameraView.color.b = 1.0;
    m_cameraView.type = 11;
    m_cameraView.action = visualization_msgs::Marker::ADD;
  }

  // Subscribers
  if (pclInput) {
    m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 5);
    m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 5);
    m_tfPointCloudSub->registerCallback(boost::bind(&MarbleMapping::insertCloudCallback, this, _1));
    // Services
    m_octomapBinaryService = m_nh.advertiseService("octomap_binary", &MarbleMapping::octomapBinarySrv, this);
    m_octomapFullService = m_nh.advertiseService("octomap_full", &MarbleMapping::octomapFullSrv, this);
    m_resetService = m_nh_private.advertiseService("reset", &MarbleMapping::resetSrv, this);
  } else if (octomapInput) {
    m_octomapSub = m_nh.subscribe("cloud_in", 100, &MarbleMapping::octomapCallback, this);
  } else if (diffInput) {
    // Get the agents from the launch parameters
    std::vector<std::string> agents_vec;
    std::replace(m_agents.begin(), m_agents.end(), ',', ' ');
    std::stringstream ss(m_agents);
    std::string temp;
    while (ss >> temp)
      agents_vec.push_back(temp);

    // Create a subscriber for each agent
    for (auto agent : agents_vec) {
      std::string topic = m_diff_pre + agent + m_diff_post;
      m_diffSubs[agent] = m_nh.subscribe<octomap_msgs::Octomap>(topic, 100, boost::bind(&MarbleMapping::octomapDiffsCallback, this, _1, agent));
    }

    // Populate the neighbors message so mergeNeighbors will work
    neighbors.num_neighbors = 1;
    neighbors.neighbors.push_back(marble_mapping::OctomapArray());
    neighbors.neighbors[0].num_octomaps = 1;
    neighbors.neighbors[0].octomaps.push_back(octomap_msgs::Octomap());
  }

  if (m_multiagent) {
    m_neighborsSub = m_nh.subscribe("neighbor_maps", 100, &MarbleMapping::neighborMapsCallback, this);
    // Timer for updating the local map diff that will be broadcast
    diff_timer = m_nh.createTimer(ros::Duration(diff_duration), &MarbleMapping::updateDiff, this);
  }

  // Timer for publishing the OctoMaps
  ros::TimerOptions ops_pub;
  ops_pub.period = ros::Duration(pub_duration);
  ops_pub.callback = boost::bind(&MarbleMapping::publishOctoMaps, this, _1);
  if (!octomapInput) ops_pub.callback_queue = &pub_queue;
  pub_timer = m_nh.createTimer(ops_pub);

  // Timer to publish the optional maps, on a separate thread
  if (m_publishMarkerArray || m_publishFreeSpace || m_publishPointCloud) {
    ros::TimerOptions ops;
    ops.period = ros::Duration(pub_opt_duration);
    ops.callback = boost::bind(&MarbleMapping::publishOptionalMaps, this, _1);
    // Assign to the publishing thread, unless input is an octomap
    // If input is octomap the map is deleted so need to run single threaded
    if (!octomapInput) ops.callback_queue = &pub_queue;
    pub_opt_timer = m_nh.createTimer(ops);
  }

  pclTimeLimit = ros::Duration(0.0);
  longTimeDiff = ros::Duration(0.0);
  pclCount = 0;
  pclCountProcessed = 0;
  pclDropped = 0;

  dynamic_reconfigure::Server<MarbleMappingConfig>::CallbackType f;
  f = boost::bind(&MarbleMapping::reconfigureCallback, this, _1, _2);
  m_reconfigureServer.setCallback(f);
}

MarbleMapping::~MarbleMapping(){
  if (m_tfPointCloudSub){
    delete m_tfPointCloudSub;
    m_tfPointCloudSub = NULL;
  }

  if (m_pointCloudSub){
    delete m_pointCloudSub;
    m_pointCloudSub = NULL;
  }

  if (m_octree){
    delete m_octree;
    m_octree = NULL;
  }

  if (m_merged_tree){
    delete m_merged_tree;
    m_merged_tree = NULL;
  }

  if (m_diff_tree){
    delete m_diff_tree;
    m_diff_tree = NULL;
  }
}

bool MarbleMapping::openFile(const std::string& filename){
  if (filename.length() <= 3)
    return false;

  std::string suffix = filename.substr(filename.length()-3, 3);
  if (suffix== ".bt"){
    if (!m_octree->readBinary(filename)){
      return false;
    }
  } else if (suffix == ".ot"){
    AbstractOcTree* tree = AbstractOcTree::read(filename);
    if (!tree){
      return false;
    }
    if (m_octree){
      delete m_octree;
      m_octree = NULL;
    }
    m_octree = dynamic_cast<OcTreeT*>(tree);
    if (!m_octree){
      ROS_ERROR("Could not read OcTree in file, currently there are no other types supported in .ot");
      return false;
    }

  } else{
    return false;
  }

  ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(),m_octree->size());

  m_treeDepth = m_octree->getTreeDepth();
  m_maxTreeDepth = m_treeDepth;
  m_res = m_octree->getResolution();

  return true;

}

void MarbleMapping::neighborMapsCallback(const marble_mapping::OctomapNeighborsConstPtr& msg) {
  neighbors = *msg;

  // If hard reset passed, start the self and merged map over!
  if (neighbors.hardReset) {
    boost::mutex::scoped_lock lock(m_mtx);
    m_octree->clear();
    if (m_diffMerged)
      m_merged_tree->resetChangeDetection();
    else
      m_octree->resetChangeDetection();
    mapdiffs = marble_mapping::OctomapArray();
    num_diffs = 0;
  }

  // Better to put this somewhere else, but the callback should be infrequent enough this is ok
  try {
    mergeNeighbors();
  } catch (const std::exception& e) {
    ROS_ERROR("merging error! %s", e.what());
  }

  if (m_compressMaps) {
    boost::mutex::scoped_lock lock(m_mtx);
    m_merged_tree->prune();
  }
}

void MarbleMapping::octomapCallback(const octomap_msgs::OctomapConstPtr& msg) {
  boost::mutex::scoped_lock lock(m_mtx);
  delete m_merged_tree;
  m_merged_tree = (octomap::OcTreeStamped*)octomap_msgs::binaryMsgToMap(*msg);
}

void MarbleMapping::octomapDiffsCallback(const octomap_msgs::OctomapConstPtr& msg, const std::string owner) {
  // Update the neighbors message with current info
  neighbors.neighbors[0].owner = owner;
  neighbors.neighbors[0].octomaps[0] = *msg;

  // Merge the diff
  try {
    mergeNeighbors();
  } catch (const std::exception& e) {
    ROS_ERROR("merging error! %s", e.what());
  }

  if (m_compressMaps) {
    boost::mutex::scoped_lock lock(m_mtx);
    m_merged_tree->prune();
  }

  // Publish the binary maps
  // ros::Time rostime = ros::Time::now();
  // if (m_publishMergedBinaryMap)
  //   publishMergedBinaryOctoMap(rostime);
  //
  // if (m_publishNeighborMaps)
  //   publishNeighborMaps(rostime);
}

void MarbleMapping::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
  // Get the initial time offset for the sensor
  // This might not work well if multiple sensors publish to the same topic, instead of using
  // a topic_tools to merge them, where the timestamps get rewritten!
  if (pclTimeLimit == ros::Duration(0.0)) {
    pclTimeLimit = ros::Time().now() - cloud->header.stamp + ros::Duration(m_pclTimeLimit);
  }

  pclCount++;
  // Drop any "old" pointclouds
  // For multiple sensors, might want a way to ensure we get a good mix of each!
  if (cloud->header.stamp + pclTimeLimit < ros::Time().now()) {
    pclDropped++;
    ros::Duration timediff = ros::Time::now() - cloud->header.stamp - pclTimeLimit + ros::Duration(m_pclTimeLimit);
    if (timediff > longTimeDiff) longTimeDiff = timediff;

    return;
  }

  ros::WallTime startTime = ros::WallTime::now();

  //
  // ground filtering in base frame
  //
  PCLPointCloud pc; // input cloud for filtering and ground-detection
  pcl::fromROSMsg(*cloud, pc);

  if (m_downsampleSize > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud (pc.makeShared());
    voxel_filter.setFilterFieldName("x");
    voxel_filter.setLeafSize (m_downsampleSize, m_downsampleSize, m_downsampleSize);
    voxel_filter.filter (pc);
  }

  tf::StampedTransform sensorToWorldTf;
  try {
    m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
  } catch(tf::TransformException& ex){
    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }

  Eigen::Matrix4f sensorToWorld;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);


  // set up filter for height range, also removes NANs:
  pcl::PassThrough<PCLPoint> pass_x;
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
  pcl::PassThrough<PCLPoint> pass_y;
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
  pcl::PassThrough<PCLPoint> pass_z;
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

  PCLPointCloud pc_ground; // segmented ground plane
  PCLPointCloud pc_nonground; // everything else

  if (m_filterGroundPlane){
    tf::StampedTransform sensorToBaseTf, baseToWorldTf;
    try{
      m_tfListener.waitForTransform(m_baseFrameId, cloud->header.frame_id, cloud->header.stamp, ros::Duration(0.2));
      m_tfListener.lookupTransform(m_baseFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToBaseTf);
      m_tfListener.lookupTransform(m_worldFrameId, m_baseFrameId, cloud->header.stamp, baseToWorldTf);


    }catch(tf::TransformException& ex){
      ROS_ERROR_STREAM( "Transform error for ground plane filter: " << ex.what() << ", quitting callback.\n"
                        "You need to set the base_frame_id or disable filter_ground.");
    }


    Eigen::Matrix4f sensorToBase, baseToWorld;
    pcl_ros::transformAsMatrix(sensorToBaseTf, sensorToBase);
    pcl_ros::transformAsMatrix(baseToWorldTf, baseToWorld);

    // transform pointcloud from sensor frame to fixed robot frame
    pcl::transformPointCloud(pc, pc, sensorToBase);
    pass_x.setInputCloud(pc.makeShared());
    pass_x.filter(pc);
    pass_y.setInputCloud(pc.makeShared());
    pass_y.filter(pc);
    pass_z.setInputCloud(pc.makeShared());
    pass_z.filter(pc);
    filterGroundPlane(pc, pc_ground, pc_nonground);

    // transform clouds to world frame for insertion
    pcl::transformPointCloud(pc_ground, pc_ground, baseToWorld);
    pcl::transformPointCloud(pc_nonground, pc_nonground, baseToWorld);
  } else {
    // directly transform to map frame:
    pcl::transformPointCloud(pc, pc, sensorToWorld);

    // just filter height range:
    pass_x.setInputCloud(pc.makeShared());
    pass_x.filter(pc);
    pass_y.setInputCloud(pc.makeShared());
    pass_y.filter(pc);
    pass_z.setInputCloud(pc.makeShared());
    pass_z.filter(pc);

    pc_nonground = pc;
    // pc_nonground is empty without ground segmentation
    pc_ground.header = pc.header;
    pc_nonground.header = pc.header;
  }


  insertScan(sensorToWorldTf, pc_ground, pc_nonground);

  if (m_compressMaps) {
    boost::mutex::scoped_lock lock(m_mtx);
    m_octree->prune();
    m_merged_tree->prune();
  }

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Pointcloud insertion in MarbleMapping done (%zu+%zu pts (ground/nonground), %f sec)", pc_ground.size(), pc_nonground.size(), total_elapsed);

  pclCountProcessed++;
}

void MarbleMapping::insertScan(const tf::StampedTransform& sensorToWorldTf, const PCLPointCloud& ground, const PCLPointCloud& nonground){
  // Get the rotation matrix and the position, and build camera view if enabled
  tf::Matrix3x3 rotation = sensorToWorldTf.getBasis();
  point3d sensorOrigin = pointTfToOctomap(sensorToWorldTf.getOrigin());
  if (m_buildCameraMap)
    buildView(rotation, sensorOrigin);

  // instead of direct scan insertion, compute update to filter ground:
  KeySet free_cells, occupied_cells;

  // Profile our processing time
  ros::WallTime startTime = ros::WallTime::now();

  // insert ground points only as free:
  #pragma omp parallel for schedule(guided)
  for (PCLPointCloud::const_iterator it = ground.begin(); it < ground.end(); ++it) {
    point3d point(it->x, it->y, it->z);
    unsigned threadIdx = omp_get_thread_num();
    KeyRay* keyRay = &(keyrays.at(threadIdx));

    // maxrange check
    if ((m_maxRange > 0.0) && ((point - sensorOrigin).norm() > m_maxRange) ) {
      point = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
    }

    // only clear space (ground points)
    if (m_octree->computeRayKeys(sensorOrigin, point, *keyRay)) {
      #pragma omp critical(free_insert)
      {
        free_cells.insert(keyRay->begin(), keyRay->end());
      }
    }
  }

  // all other points: free on ray, occupied on endpoint:
  #pragma omp parallel for schedule(guided)
  for (PCLPointCloud::const_iterator it = nonground.begin(); it < nonground.end(); ++it) {
    point3d point(it->x, it->y, it->z);
    unsigned threadIdx = omp_get_thread_num();
    KeyRay* keyRay = &(keyrays.at(threadIdx));

    // maxrange check
    if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {

      // free cells
      if (m_octree->computeRayKeys(sensorOrigin, point, *keyRay)) {
        #pragma omp critical(free_insert)
        {
          free_cells.insert(keyRay->begin(), keyRay->end());
        }
      }
      // occupied endpoint
      OcTreeKey key;
      if (m_octree->coordToKeyChecked(point, key)) {
        #pragma omp critical(occupied_insert)
        {
          occupied_cells.insert(key);
        }
      }
    } else {// ray longer than maxrange:;
      point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
      if (m_octree->computeRayKeys(sensorOrigin, new_end, *keyRay)) {
        #pragma omp critical(free_insert)
        {
          free_cells.insert(keyRay->begin(), keyRay->end());
        }

        octomap::OcTreeKey endKey;
        if (m_octree->coordToKeyChecked(new_end, endKey)) {
          #pragma omp critical(free_insert)
          {
            // This clears out previously occupied cells that were at the edge of the range
            free_cells.insert(endKey);
          }
        } else {
          ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
        }
      }
    }
  }

  // Find the total time for ray casting
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("The %zu points took %f seconds)", nonground.size(), total_elapsed);

  boost::mutex::scoped_lock lock(m_mtx);
  // mark free cells only if not seen occupied in this cloud
  for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
    if (occupied_cells.find(*it) == occupied_cells.end()){
      // Voxels are based on the main octree, so need the actual coordinate for other res trees
      point3d point = m_octree->keyToCoord(*it);
      m_octree->updateNode(*it, false);
      // Update the merged map.  Using the coordinate allows for different resolutions
      OcTreeNodeStamped *newNode = m_merged_tree->updateNode(point, false);
      newNode->setTimestamp(1);
      // Build the map of what the secondary sensor can see
      if (m_buildCameraMap && pointInsideView(point)) {
        m_camera_tree->updateNode(point, false);
      }
    }
  }

  // now mark all occupied cells:
  for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
    point3d point = m_octree->keyToCoord(*it);
    m_octree->updateNode(*it, true);
    OcTreeNodeStamped *newNode = m_merged_tree->updateNode(point, true);
    newNode->setTimestamp(1);
    if (m_buildCameraMap && pointInsideView(point)) {
      m_camera_tree->updateNode(point, true);
    }

  }
}

template <class OcTreeMT>
int MarbleMapping::updateDiffTree(OcTreeMT* tree, PCLPointCloud& pclDiffCloud) {
  // Find all changed nodes since last update and create a new octomap
  KeyBoolMap::const_iterator startPnt = tree->changedKeysBegin();
  KeyBoolMap::const_iterator endPnt = tree->changedKeysEnd();

  int num_nodes = 0;
  for (KeyBoolMap::const_iterator iter = startPnt; iter != endPnt; ++iter) {
    // Copy the value for each node
    OcTreeNode* node = tree->search(iter->first);

    // We should never have a NULL node, but just in case...
    if (node != NULL) {
      m_diff_tree->setNodeValue(iter->first, node->getLogOdds());
      num_nodes++;

      // Create a point cloud diff if enabled
      if ((m_publishPointCloudDiff) && (node->getLogOdds() >= 0)) {
        point3d point = tree->keyToCoord(iter->first);
        pclDiffCloud.push_back(PCLPoint(point.x(), point.y(), point.z()));
      }
    }
  }

  return num_nodes;
}

void MarbleMapping::updateDiff(const ros::TimerEvent& event) {
  // TODO Add a check to see if we're in comm with anyone, and if not, don't update the diff
  PCLPointCloud pclDiffCloud;
  int num_nodes;

  // Update the diff tree from either the merged map or self map
  boost::mutex::scoped_lock lock(m_mtx);
  m_diff_tree->clear();
  if (m_diffMerged)
    num_nodes = updateDiffTree(m_merged_tree, pclDiffCloud);
  else
    num_nodes = updateDiffTree(m_octree, pclDiffCloud);
  lock.unlock();

  // Only update the diff map if enough has changed
  if (num_nodes > diff_threshold) {
    if (m_publishDiffs) {
      num_diffs++;
      m_diff_tree->prune();

      // Create and publish message for this diff
      octomap_msgs::Octomap msg;
      octomap_msgs::binaryMapToMsg(*m_diff_tree, msg);
      msg.header.frame_id = m_worldFrameId;
      msg.header.stamp = ros::Time().now();
      msg.header.seq = num_diffs - 1;
      m_diffMapPub.publish(msg);

      // Add the diff to the map diffs array
      mapdiffs.octomaps.push_back(msg);
      mapdiffs.num_octomaps = num_diffs;
      m_diffsMapPub.publish(mapdiffs);
    }

    // Publish the diff point cloud if enabled
    if (m_publishPointCloudDiff) {
      sensor_msgs::PointCloud2 cloud;
      pcl::toROSMsg (pclDiffCloud, cloud);
      cloud.header.frame_id = m_worldFrameId;
      cloud.header.stamp = ros::Time().now();
      m_pointCloudDiffPub.publish(cloud);
    }

    boost::mutex::scoped_lock lock(m_mtx);
    if (m_diffMerged)
      m_merged_tree->resetChangeDetection();
    else
      m_octree->resetChangeDetection();
  }

  // Stats on dropped clouds
  if (pclDropped > 0) {
    double dropped = (double)pclDropped / (double)pclCount * 100.0;
    ROS_INFO("Dropped %.1f%% of clouds.  Largest delay was %.2f seconds. Processed %d clouds.", dropped, longTimeDiff.toSec(), pclCountProcessed);
    pclCount = 0;
    pclCountProcessed = 0;
    pclDropped = 0;
    longTimeDiff = ros::Duration(0.0);
  } else {
    ROS_INFO("Processed %d clouds", pclCountProcessed);
    pclCount = 0;
    pclCountProcessed = 0;
  }
}

void MarbleMapping::mergeNeighbors() {
  // Merge neighbor maps with local map
  std::shared_ptr<octomap::OcTree> ntree(nullptr);
  bool overwrite_node;
  unsigned ts;

  for (int i=0; i < neighbors.num_neighbors; i++) {
    std::string nid = neighbors.neighbors[i].owner;

    // If clear passed then re-merge everything
    if (neighbors.clear) {
      if (i == 0) {
        boost::mutex::scoped_lock lock(m_mtx);
        // First time through clear the merged tree, and reset change detection if necessary
        m_merged_tree->clear();

        // Copy the self map to merged map
        for (OcTree::leaf_iterator it = m_octree->begin_leafs();
            it != m_octree->end_leafs(); ++it) {
          point3d point = it.getCoordinate();
          OcTreeNodeStamped *newNode = m_merged_tree->setNodeValue(point, it->getLogOdds());
          newNode->setTimestamp(1);
        }

        if (m_diffMerged)
          m_merged_tree->resetChangeDetection();
      }
      // For each neighbor, clear the diffs
      seqs[nid.data()].clear();
      neighbor_maps[nid.data()] = NULL;
    }

    // Initialize neighbor map data if enabled
    if (m_publishNeighborMaps && !neighbor_maps[nid.data()]) {
      neighbor_maps[nid.data()] = new OcTreeT(m_mres);
      neighbor_updated[nid.data()] = false;
      neighbor_pubs[nid.data()] = m_nh.advertise<Octomap>("neighbors/"+nid+"/map", 1, m_latchedTopics);
    }

    // Check each diff for new ones to merge
    for (int j=0; j < neighbors.neighbors[i].octomaps.size(); j++) {
      uint32_t cur_seq = neighbors.neighbors[i].octomaps[j].header.seq;
      bool exists = std::count(seqs[nid.data()].cbegin(), seqs[nid.data()].cend(), cur_seq);

      // Only merge if we haven't already merged this sequence
      if (!exists) {
        // Add to our list of sequences, and get a unique timestamp for this neighbor
        seqs[nid.data()].push_back(cur_seq);
        if (idx[nid.data()])
          ts = idx[nid.data()];
        else {
          idx[nid.data()] = next_idx;
          ts = next_idx;
          next_idx++;
        }

        // If we missed an earlier sequence and get it later, don't replace, but potentially merge
        if (cur_seq >= *std::max_element(seqs[nid.data()].cbegin(), seqs[nid.data()].cend()))
          overwrite_node = true;
        else
          overwrite_node = false;

        if (neighbors.neighbors[i].octomaps[j].binary)
          ntree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(neighbors.neighbors[i].octomaps[j])));
        else
          ntree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(neighbors.neighbors[i].octomaps[j])));

        // Iterate through the diff tree and merge
        ntree->expand();
        boost::mutex::scoped_lock lock(m_mtx);
        for (OcTree::leaf_iterator it = ntree->begin_leafs(), end = ntree->end_leafs();
            it != end; ++it) {
          OcTreeKey nodeKey = it.getKey();
          OcTreeNodeStamped *nodeM = m_merged_tree->search(nodeKey);
          if (nodeM != NULL) {
            if (overwrite_node && (nodeM->getTimestamp() == ts)) {
              // If the diff is newer, and the merge node came from this neighbor
              // replace the value and maintain the timestamp id
              m_merged_tree->setNodeValue(nodeKey, it->getLogOdds());
              nodeM->setTimestamp(ts);
            } else if ((nodeM->getTimestamp() == 0) || (nodeM->getTimestamp() != 1)) {
              // If there's already a node in the merged map, but it's from another neighbor,
              // or it's been previously merged, merge the value, and set timestamp to merged
              // Update based on occupancy seems to work better than getLogOdds
              if (it->getLogOdds() >= 0)
                m_merged_tree->updateNode(nodeKey, true);
              else
                m_merged_tree->updateNode(nodeKey, false);
              nodeM->setTimestamp(0);
            }
          } else {
            // If the node doesn't exist in the merged map, add it with the value
            OcTreeNodeStamped *newNode = m_merged_tree->setNodeValue(nodeKey, it->getLogOdds());
            newNode->setTimestamp(ts);
          }

          // If neighbor maps enabled, add the node if it's new
          if (m_publishNeighborMaps) {
            neighbor_updated[nid.data()] = true;
            if (neighbor_maps[nid.data()]->search(nodeKey) != NULL) {
                neighbor_maps[nid.data()]->setNodeValue(nodeKey, it->getLogOdds());
            } else {
              neighbor_maps[nid.data()]->setNodeValue(nodeKey, it->getLogOdds());
            }
          }
        }
      }
    }
  }
}

void MarbleMapping::publishOptionalMaps(const ros::TimerEvent& event) {
  // Publish optional maps such as marker arrays.  Performed in separate thread.
  ros::WallTime startTime = ros::WallTime::now();
  ros::Time rostime = ros::Time::now();
  if ((m_octree->size() <= 1) && (m_merged_tree->size() <= 1)) {
    return;
  }

  bool publishFreeMarkerArray = m_publishFreeSpace && (m_fmarkerPub.getNumSubscribers() > 0);
  bool publishMarkerArray = m_publishMarkerArray && (m_markerPub.getNumSubscribers() > 0);
  bool publishPointCloud = m_publishPointCloud && (m_pointCloudPub.getNumSubscribers() > 0);

  // init markers:
  visualization_msgs::MarkerArray freeNodesVis, occupiedNodesVis;

  // init pointcloud:
  pcl::PointCloud<PCLPoint> pclCloud;

  boost::mutex::scoped_lock lock(m_mtx);
  // Only traverse the tree if we're publishing a marker array or point cloud
  if (publishMarkerArray || publishFreeMarkerArray || publishPointCloud) {
    // each array stores all cubes of a different size, one for each depth level:
    freeNodesVis.markers.resize(m_treeDepth+1);
    occupiedNodesVis.markers.resize(m_treeDepth+1);

    // now, traverse all leafs in the tree:
    for (OcTreeStamped::iterator it = m_merged_tree->begin(m_maxTreeDepth),
        end = m_merged_tree->end(); it != end; ++it) {
      if (m_merged_tree->isNodeOccupied(*it)) {
        // Skip this node if remove ceiling is enabled
        if (m_removeCeiling && isCeiling(it.getKey())) {
          continue;
        }

        double z = it.getZ();
        double half_size = it.getSize() / 2.0;
        if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ) {
          double size = it.getSize();
          double x = it.getX();
          double y = it.getY();

          // Ignore speckles in the map:
          if (m_filterSpeckles && (it.getDepth() == m_treeDepth +1) && isSpeckleNode(it.getKey())) {
            ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
            continue;
          } // else: current octree node is no speckle, send it out

          //create marker:
          if (publishMarkerArray) {
            unsigned idx = it.getDepth();
            assert(idx < occupiedNodesVis.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
            double minX, minY, minZ, maxX, maxY, maxZ;
            m_merged_tree->getMetricMin(minX, minY, minZ);
            m_merged_tree->getMetricMax(maxX, maxY, maxZ);

            double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
            occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
          }

          // insert into pointcloud:
          if (publishPointCloud) {
            pclCloud.push_back(PCLPoint(x, y, z));
          }

        }
      } else { // node not occupied => mark as free in 2D map if unknown so far
        if (publishFreeMarkerArray) {
          double z = it.getZ();
          double half_size = it.getSize() / 2.0;
          if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ) {
            double x = it.getX();
            double y = it.getY();

            //create marker for free space:
            unsigned idx = it.getDepth();
            assert(idx < freeNodesVis.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            freeNodesVis.markers[idx].points.push_back(cubeCenter);
          }
        }
      }
    }
  }

  // finish MarkerArray:
  if (publishMarkerArray){
    for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
      double size = m_merged_tree->getNodeSize(i);

      occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
      occupiedNodesVis.markers[i].header.stamp = rostime;
      occupiedNodesVis.markers[i].ns = "map";
      occupiedNodesVis.markers[i].id = i;
      occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      occupiedNodesVis.markers[i].scale.x = size;
      occupiedNodesVis.markers[i].scale.y = size;
      occupiedNodesVis.markers[i].scale.z = size;
      occupiedNodesVis.markers[i].color = m_color;


      if (occupiedNodesVis.markers[i].points.size() > 0)
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    m_markerPub.publish(occupiedNodesVis);
  }

  // finish FreeMarkerArray:
  if (publishFreeMarkerArray){
    for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){
      double size = m_merged_tree->getNodeSize(i);

      freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
      freeNodesVis.markers[i].header.stamp = rostime;
      freeNodesVis.markers[i].ns = "map";
      freeNodesVis.markers[i].id = i;
      freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      freeNodesVis.markers[i].scale.x = size;
      freeNodesVis.markers[i].scale.y = size;
      freeNodesVis.markers[i].scale.z = size;
      freeNodesVis.markers[i].color = m_colorFree;


      if (freeNodesVis.markers[i].points.size() > 0)
        freeNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    m_fmarkerPub.publish(freeNodesVis);
  }

  // finish pointcloud:
  if (publishPointCloud){
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg (pclCloud, cloud);
    cloud.header.frame_id = m_worldFrameId;
    cloud.header.stamp = rostime;
    m_pointCloudPub.publish(cloud);
  }

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Optional map publishing in MarbleMapping took %f sec", total_elapsed);
}

void MarbleMapping::publishOctoMaps(const ros::TimerEvent& event) {
  ros::WallTime startTime = ros::WallTime::now();
  ros::Time rostime = ros::Time::now();
  if ((m_octree->size() <= 1) && (m_merged_tree->size() <= 1)) {
    return;
  }

  bool publishMergedBinaryMap = m_publishMergedBinaryMap && (m_mergedBinaryMapPub.getNumSubscribers() > 0);
  bool publishMergedFullMap = m_publishMergedFullMap && (m_mergedFullMapPub.getNumSubscribers() > 0);
  bool publishCameraMap = m_publishCameraMap && (m_cameraMapPub.getNumSubscribers() > 0);
  bool publishCameraView = m_publishCameraView && (m_cameraViewPub.getNumSubscribers() > 0);
  bool publishBinaryMap = (m_binaryMapPub.getNumSubscribers() > 0);
  bool publishFullMap = (m_fullMapPub.getNumSubscribers() > 0);

  if (publishBinaryMap)
    publishBinaryOctoMap(rostime);

  if (publishFullMap)
    publishFullOctoMap(rostime);

  if (publishMergedBinaryMap)
    publishMergedBinaryOctoMap(rostime);

  if (publishMergedFullMap)
    publishMergedFullOctoMap(rostime);

  if (publishCameraMap)
    publishCameraOctoMap(rostime);

  if (publishCameraView)
    m_cameraViewPub.publish(m_cameraView);

  if (m_publishNeighborMaps)
    publishNeighborMaps(rostime);

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Map publishing in MarbleMapping took %f sec", total_elapsed);
}

bool MarbleMapping::octomapBinarySrv(OctomapSrv::Request  &req,
                                    OctomapSrv::Response &res)
{
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("Sending binary map data on service request");
  res.map.header.frame_id = m_worldFrameId;
  res.map.header.stamp = ros::Time::now();
  if (!octomap_msgs::binaryMapToMsg(*m_octree, res.map))
    return false;

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("Binary octomap sent in %f sec", total_elapsed);
  return true;
}

bool MarbleMapping::octomapFullSrv(OctomapSrv::Request  &req,
                                    OctomapSrv::Response &res)
{
  ROS_INFO("Sending full map data on service request");
  res.map.header.frame_id = m_worldFrameId;
  res.map.header.stamp = ros::Time::now();


  if (!octomap_msgs::fullMapToMsg(*m_octree, res.map))
    return false;

  return true;
}

bool MarbleMapping::resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
  visualization_msgs::MarkerArray occupiedNodesVis;
  occupiedNodesVis.markers.resize(m_treeDepth +1);
  ros::Time rostime = ros::Time::now();
  m_octree->clear();
  ROS_INFO("Cleared octomap");

  publishBinaryOctoMap(rostime);
  for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){

    occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
    occupiedNodesVis.markers[i].header.stamp = rostime;
    occupiedNodesVis.markers[i].ns = "map";
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }

  m_markerPub.publish(occupiedNodesVis);

  visualization_msgs::MarkerArray freeNodesVis;
  freeNodesVis.markers.resize(m_treeDepth +1);

  for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){

    freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
    freeNodesVis.markers[i].header.stamp = rostime;
    freeNodesVis.markers[i].ns = "map";
    freeNodesVis.markers[i].id = i;
    freeNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    freeNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  m_fmarkerPub.publish(freeNodesVis);

  return true;
}

void MarbleMapping::publishBinaryOctoMap(const ros::Time& rostime) const{

  Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = rostime;

  boost::mutex::scoped_lock lock(m_mtx);
  if (octomap_msgs::binaryMapToMsg(*m_octree, map))
    m_binaryMapPub.publish(map);
  else
    ROS_ERROR("Error serializing OctoMap");
}

void MarbleMapping::publishFullOctoMap(const ros::Time& rostime) const{

  Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = rostime;

  boost::mutex::scoped_lock lock(m_mtx);
  if (octomap_msgs::fullMapToMsg(*m_octree, map))
    m_fullMapPub.publish(map);
  else
    ROS_ERROR("Error serializing OctoMap");
}

void MarbleMapping::publishMergedBinaryOctoMap(const ros::Time& rostime) const{

  Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = rostime;

  boost::mutex::scoped_lock lock(m_mtx);
  if (octomap_msgs::binaryMapToMsg(*m_merged_tree, map)) {
    map.id = "OcTree";
    m_mergedBinaryMapPub.publish(map);
  } else
    ROS_ERROR("Error serializing OctoMap");
}

void MarbleMapping::publishMergedFullOctoMap(const ros::Time& rostime) const{

  Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = rostime;

  boost::mutex::scoped_lock lock(m_mtx);
  if (octomap_msgs::fullMapToMsg(*m_merged_tree, map)) {
    map.id = "OcTree";
    m_mergedFullMapPub.publish(map);
  } else
    ROS_ERROR("Error serializing OctoMap");
}

void MarbleMapping::publishCameraOctoMap(const ros::Time& rostime) const{

  Octomap map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = rostime;

  boost::mutex::scoped_lock lock(m_mtx);
  if (octomap_msgs::binaryMapToMsg(*m_camera_tree, map)) {
    map.id = "OcTree";
    m_cameraMapPub.publish(map);
  } else
    ROS_ERROR("Error serializing OctoMap");
}

void MarbleMapping::publishNeighborMaps(const ros::Time& rostime) {
  boost::mutex::scoped_lock lock(m_mtx);
  for (auto& n : neighbor_maps) {
    // Find any maps that have been updated, and have subscribers
    if (neighbor_updated.at(n.first) && (neighbor_pubs.at(n.first).getNumSubscribers() > 0)) {
      neighbor_updated.at(n.first) = false;
      Octomap map;
      map.header.frame_id = m_worldFrameId;
      map.header.stamp = rostime;

      if (octomap_msgs::binaryMapToMsg(*n.second, map))
        neighbor_pubs.at(n.first).publish(map);
      else
        ROS_ERROR("Error serializing OctoMap");
    }
  }
}

void MarbleMapping::filterGroundPlane(const PCLPointCloud& pc, PCLPointCloud& ground, PCLPointCloud& nonground) const{
  ground.header = pc.header;
  nonground.header = pc.header;

  if (pc.size() < 50){
    ROS_WARN("Pointcloud in MarbleMapping too small, skipping ground plane extraction");
    nonground = pc;
  } else {
    // plane detection for ground plane removal:
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object and set up:
    pcl::SACSegmentation<PCLPoint> seg;
    seg.setOptimizeCoefficients (true);
    // TODO: maybe a filtering based on the surface normals might be more robust / accurate?
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold (m_groundFilterDistance);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle(m_groundFilterAngle);


    PCLPointCloud cloud_filtered(pc);
    // Create the filtering object
    pcl::ExtractIndices<PCLPoint> extract;
    bool groundPlaneFound = false;

    while(cloud_filtered.size() > 10 && !groundPlaneFound){
      seg.setInputCloud(cloud_filtered.makeShared());
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0){
        ROS_INFO("PCL segmentation did not find any plane.");

        break;
      }

      extract.setInputCloud(cloud_filtered.makeShared());
      extract.setIndices(inliers);

      if (std::abs(coefficients->values.at(3)) < m_groundFilterPlaneDistance){
        ROS_DEBUG("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(),
                  coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
        extract.setNegative (false);
        extract.filter (ground);

        // remove ground points from full pointcloud:
        // workaround for PCL bug:
        if(inliers->indices.size() != cloud_filtered.size()){
          extract.setNegative(true);
          PCLPointCloud cloud_out;
          extract.filter(cloud_out);
          nonground += cloud_out;
          cloud_filtered = cloud_out;
        }

        groundPlaneFound = true;
      } else{
        ROS_DEBUG("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(),
                  coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
        pcl::PointCloud<PCLPoint> cloud_out;
        extract.setNegative (false);
        extract.filter(cloud_out);
        nonground +=cloud_out;
        // debug
        //            pcl::PCDWriter writer;
        //            writer.write<PCLPoint>("nonground_plane.pcd",cloud_out, false);

        // remove current plane from scan for next iteration:
        // workaround for PCL bug:
        if(inliers->indices.size() != cloud_filtered.size()){
          extract.setNegative(true);
          cloud_out.points.clear();
          extract.filter(cloud_out);
          cloud_filtered = cloud_out;
        } else{
          cloud_filtered.points.clear();
        }
      }

    }
    // TODO: also do this if overall starting pointcloud too small?
    if (!groundPlaneFound){ // no plane found or remaining points too small
      ROS_WARN("No ground plane found in scan");

      // do a rough fitlering on height to prevent spurious obstacles
      pcl::PassThrough<PCLPoint> second_pass;
      second_pass.setFilterFieldName("z");
      second_pass.setFilterLimits(-m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);
      second_pass.setInputCloud(pc.makeShared());
      second_pass.filter(ground);

      second_pass.setFilterLimitsNegative (true);
      second_pass.filter(nonground);
    }

    // debug:
    //        pcl::PCDWriter writer;
    //        if (pc_ground.size() > 0)
    //          writer.write<PCLPoint>("ground.pcd",pc_ground, false);
    //        if (pc_nonground.size() > 0)
    //          writer.write<PCLPoint>("nonground.pcd",pc_nonground, false);

  }


}

bool MarbleMapping::isSpeckleNode(const OcTreeKey&nKey) const {
  OcTreeKey key;
  bool neighborFound = false;
  boost::mutex::scoped_lock lock(m_mtx);
  for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]){
    for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]){
      for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]){
        if (key != nKey){
          OcTreeNode* node = m_merged_tree->search(key);
          if (node && m_merged_tree->isNodeOccupied(node)){
            // we have a neighbor => break!
            neighborFound = true;
          }
        }
      }
    }
  }

  return neighborFound;
}

void MarbleMapping::reconfigureCallback(marble_mapping::MarbleMappingConfig& config, uint32_t level){
  if (m_maxTreeDepth != unsigned(config.max_depth))
    m_maxTreeDepth = unsigned(config.max_depth);
  else{
    m_pointcloudMinZ            = config.pointcloud_min_z;
    m_pointcloudMaxZ            = config.pointcloud_max_z;
    m_occupancyMinZ             = config.occupancy_min_z;
    m_occupancyMaxZ             = config.occupancy_max_z;
    m_filterSpeckles            = config.filter_speckles;
    m_filterGroundPlane         = config.filter_ground;

    // Parameters with a namespace require an special treatment at the beginning, as dynamic reconfigure
    // will overwrite them because the server is not able to match parameters' names.
    if (m_initConfig){
		// If parameters do not have the default value, dynamic reconfigure server should be updated.
		if(!is_equal(m_groundFilterDistance, 0.04))
          config.ground_filter_distance = m_groundFilterDistance;
		if(!is_equal(m_groundFilterAngle, 0.15))
          config.ground_filter_angle = m_groundFilterAngle;
	    if(!is_equal( m_groundFilterPlaneDistance, 0.07))
          config.ground_filter_plane_distance = m_groundFilterPlaneDistance;
        if(!is_equal(m_maxRange, -1.0))
          config.sensor_model_max_range = m_maxRange;
        if(!is_equal(m_octree->getProbHit(), 0.7))
          config.sensor_model_hit = m_octree->getProbHit();
	    if(!is_equal(m_octree->getProbMiss(), 0.4))
          config.sensor_model_miss = m_octree->getProbMiss();
		if(!is_equal(m_octree->getClampingThresMin(), 0.12))
          config.sensor_model_min = m_octree->getClampingThresMin();
		if(!is_equal(m_octree->getClampingThresMax(), 0.97))
          config.sensor_model_max = m_octree->getClampingThresMax();
        m_initConfig = false;

	    boost::recursive_mutex::scoped_lock reconf_lock(m_config_mutex);
        m_reconfigureServer.updateConfig(config);
    }
    else{
	  m_groundFilterDistance      = config.ground_filter_distance;
      m_groundFilterAngle         = config.ground_filter_angle;
      m_groundFilterPlaneDistance = config.ground_filter_plane_distance;
      m_maxRange                  = config.sensor_model_max_range;
      m_octree->setClampingThresMin(config.sensor_model_min);
      m_octree->setClampingThresMax(config.sensor_model_max);

     // Checking values that might create unexpected behaviors.
      if (is_equal(config.sensor_model_hit, 1.0))
		config.sensor_model_hit -= 1.0e-6;
      m_octree->setProbHit(config.sensor_model_hit);
	  if (is_equal(config.sensor_model_miss, 0.0))
		config.sensor_model_miss += 1.0e-6;
      m_octree->setProbMiss(config.sensor_model_miss);
	}
  }
}

std_msgs::ColorRGBA MarbleMapping::heightMapColor(double h) {

  std_msgs::ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v; color.g = n; color.b = m;
      break;
    case 1:
      color.r = n; color.g = v; color.b = m;
      break;
    case 2:
      color.r = m; color.g = v; color.b = n;
      break;
    case 3:
      color.r = m; color.g = n; color.b = v;
      break;
    case 4:
      color.r = n; color.g = m; color.b = v;
      break;
    case 5:
      color.r = v; color.g = m; color.b = n;
      break;
    default:
      color.r = 1; color.g = 0.5; color.b = 0.5;
      break;
  }

  return color;
}

void MarbleMapping::buildView(const tf::Matrix3x3& rotation, const octomap::point3d& position) {
  // Build the planes that make up the camera field of view

  // Vectors to define where the camera is looking
  Eigen::Vector3f view(rotation[0][0], rotation[1][0], rotation[2][0]);
  Eigen::Vector3f right(rotation[0][1], rotation[1][1], rotation[2][1]);
  Eigen::Vector3f up(rotation[0][2], rotation[1][2], rotation[2][2]);
  Eigen::Vector3f pos(position.x(), position.y(), position.z());

  // Points defining corners of the view
  Eigen::Vector3f fp_c(pos + view * camera_range);
  Eigen::Vector3f fp_tl(fp_c + (up * camera_h / 2) + (right * camera_w / 2));
  Eigen::Vector3f fp_tr(fp_c + (up * camera_h / 2) - (right * camera_w / 2));
  Eigen::Vector3f fp_bl(fp_c - (up * camera_h / 2) + (right * camera_w / 2));
  Eigen::Vector3f fp_br(fp_c - (up * camera_h / 2) - (right * camera_w / 2));

  // Build the planes
  pl_f.head<3>() = (fp_bl - fp_br).cross(fp_tr - fp_br);
  pl_f(3) = -fp_c.dot(pl_f.head<3>());

  Eigen::Vector3f a(fp_bl - pos);
  Eigen::Vector3f b(fp_br - pos);
  Eigen::Vector3f c(fp_tr - pos);
  Eigen::Vector3f d(fp_tl - pos);

  pl_r.head<3>() = b.cross(c);
  pl_l.head<3>() = d.cross(a);
  pl_t.head<3>() = c.cross(d);
  pl_b.head<3>() = a.cross(b);

  pl_r(3) = -pos.dot(pl_r.head<3>());
  pl_l(3) = -pos.dot(pl_l.head<3>());
  pl_t(3) = -pos.dot(pl_t.head<3>());
  pl_b(3) = -pos.dot(pl_b.head<3>());

  // Build the camera view polygon for visualization, if enabled
  if (m_publishCameraView) {
    geometry_msgs::Point camera;
    camera.x = pos(0);
    camera.y = pos(1);
    camera.z = pos(2);
    geometry_msgs::Point TL;
    TL.x = fp_tl(0);
    TL.y = fp_tl(1);
    TL.z = fp_tl(2);
    geometry_msgs::Point TR;
    TR.x = fp_tr(0);
    TR.y = fp_tr(1);
    TR.z = fp_tr(2);
    geometry_msgs::Point BL;
    BL.x = fp_bl(0);
    BL.y = fp_bl(1);
    BL.z = fp_bl(2);
    geometry_msgs::Point BR;
    BR.x = fp_br(0);
    BR.y = fp_br(1);
    BR.z = fp_br(2);

    m_cameraView.points.clear();
    m_cameraView.points.push_back(camera);
    m_cameraView.points.push_back(TL);
    m_cameraView.points.push_back(BL);
    m_cameraView.points.push_back(camera);
    m_cameraView.points.push_back(TL);
    m_cameraView.points.push_back(TR);
    m_cameraView.points.push_back(camera);
    m_cameraView.points.push_back(TR);
    m_cameraView.points.push_back(BR);
    m_cameraView.points.push_back(camera);
    m_cameraView.points.push_back(BR);
    m_cameraView.points.push_back(BL);
  }
}

bool MarbleMapping::pointInsideView(octomap::point3d& point) {
  // Find whether a given point lies inside the field of view of the camera
  Eigen::Vector4f pt(point.x(), point.y(), point.z(), 1.0f);
  return (pt.dot(pl_l) <= 0) &&
         (pt.dot(pl_r) <= 0) &&
         (pt.dot(pl_t) <= 0) &&
         (pt.dot(pl_b) <= 0) &&
         (pt.dot(pl_f) <= 0);
}

bool MarbleMapping::isCeiling(const octomap::OcTreeKey& curKey) {
  // Determine whether a node is part of the ceiling and should be removed
  bool skipNode = false;
  for (int i = 1; i <= m_removeCeilingDepth; i++) {
    // Look at i nodes above the current node to see if does NOT exist
    OcTreeKey checkKey = curKey;
    checkKey.k[2] = curKey.k[2] + i;
    OcTreeNodeStamped* aboveNode = m_merged_tree->search(checkKey);
    if (!aboveNode) {
      for (int j = 1; j <= m_removeCeilingDepth; j++) {
        // If not, look at j nodes below the current node to see if it's free
        checkKey.k[2] = curKey.k[2] - j;
        OcTreeNodeStamped* belowNode = m_merged_tree->search(checkKey);
        if (belowNode && belowNode->getLogOdds() < 0) {
          // Found a free node below, now check if it's surrounded by free nodes as well
          checkKey.k[0] = curKey.k[0] + 1;
          belowNode = m_merged_tree->search(checkKey);
          if (!(belowNode && belowNode->getLogOdds() < 0))
            continue;

          checkKey.k[0] = curKey.k[0] - 1;
          belowNode = m_merged_tree->search(checkKey);
          if (!(belowNode && belowNode->getLogOdds() < 0))
            continue;

          checkKey.k[1] = curKey.k[1] + 1;
          belowNode = m_merged_tree->search(checkKey);
          if (!(belowNode && belowNode->getLogOdds() < 0))
            continue;

          checkKey.k[1] = curKey.k[1] - 1;
          belowNode = m_merged_tree->search(checkKey);
          if (!(belowNode && belowNode->getLogOdds() < 0))
            continue;

          // Made it this far, then all the nodes below are free, so mark and break the loop
          skipNode = true;
          break;
        }
      }

      // If we found a node below surrounded by free nodes, we can stop searching
      if (skipNode)
        break;
    }
  }

  return skipNode;
}
}
