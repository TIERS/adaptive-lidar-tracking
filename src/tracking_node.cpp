#include <math.h>

#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/feature_histogram.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  

#include <string.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <tuple>
#include <thread>
#include <mutex>

#include <math.h>

typedef pcl::PointXYZI  PointType;

class AdapIntegraDetector{

private:
    
    ros::NodeHandle nh;
    
    ros::Subscriber subLaserCloud;
    ros::Subscriber subL515Cloud;
    ros::Subscriber subUWBPose;
    ros::Subscriber subIMU;
    ros::Subscriber subOdom;

    ros::Publisher pubLaserCloud;
    ros::Publisher pubSelectCloud;

    ros::Publisher pubObjPose;

    ros::Publisher ig_pub;
    ros::Publisher ig_2_pub;
    ros::Publisher ig_5_pub;
    ros::Publisher ig_10_pub;
    ros::Publisher ig_20_pub;
    ros::Publisher ig_50_pub;
    ros::Publisher ig_100pub;
    ros::Publisher ig_200pub;
    ros::Publisher ig_500pub;

    tf::TransformListener tf_listener;

    int tmpMsgRcvCount = 0;
    int tmpMapSavedCount = 0;
    int tmpMapThreshold = 100;

    int ig_pts_cnt = 0;
    int valid_pts_th = 0;
    bool update_pose = 0;

    int ig_2_cnt = 0;
    int ig_5_cnt = 0;
    int ig_10_cnt = 0;
    int ig_20_cnt = 0;
    int ig_50_cnt = 0;
    int ig_100cnt = 0;
    int ig_200cnt = 0;
    int ig_500cnt = 0;

    int cnt = 0;

    PointType StartPose;
    geometry_msgs::Pose objPose;
 
    pcl::PointCloud<PointType>::Ptr intgratingCloud ; // Integrating cloud
    pcl::PointCloud<PointType>::Ptr interatedCloud ; // Integrating cloud
    pcl::PointCloud<PointType>::Ptr tmpCloud ;
    pcl::PointCloud<PointType>::Ptr tmpMap;
    pcl::PointCloud<PointType>::Ptr tmp_pcd;

    pcl::PointCloud<PointType>::Ptr interated_Cloud ; // Integrating cloud
    pcl::PointCloud<PointType>::Ptr interated_2_Cloud ; // Integrating cloud
    pcl::PointCloud<PointType>::Ptr interated_5_Cloud ; // Integrating cloud
    pcl::PointCloud<PointType>::Ptr interated_10_Cloud ; // Integrating cloud
    pcl::PointCloud<PointType>::Ptr interated_20_Cloud ; // Integrating cloud
    pcl::PointCloud<PointType>::Ptr interated_50_Cloud ; // Integrating cloud
    pcl::PointCloud<PointType>::Ptr interated_100Cloud ; // Integrating cloud
    pcl::PointCloud<PointType>::Ptr interated_200Cloud ; // Integrating cloud
    pcl::PointCloud<PointType>::Ptr interated_500Cloud ; // Integrating cloud 

    std::vector<std::string> result_1_vec;
    std::vector<std::string> result_2_vec;
    std::vector<std::string> result_5_vec;
    std::vector<std::string> result_10_vec;
    std::vector<std::string> result_20_vec;
    std::vector<std::string> result_50_vec;
    std::vector<std::string> result_100_vec;
    std::vector<std::string> result_200_vec;
    std::vector<std::string> result_500_vec;
 
    std::queue<pcl::PointCloud<PointType>> ig_cloud_queue;
    std::mutex ig_queue_mutex;

    std::queue<pcl::PointCloud<PointType>> itr_pts_queue; // Interasting 
    std::mutex itr_queue_mutex;

    std::mutex update_pose_mutex;                       // Locker of the pose 

    pcl::PointCloud<PointType>::Ptr accumulatePts;

    std::vector< pcl::PointCloud<PointType> > tmpMap_vec;

    geometry_msgs::PoseStamped  uwb_pose;
    std::vector< geometry_msgs::PoseStamped > uwbPose_vec;
    std::vector< geometry_msgs::PoseStamped > objPose_vec;
    std::vector< double > orient_vec;
    // pcl::PointCloud<PointType>::Ptr outlierCloud;

    // pcl::VoxelGrid<PointType> downSizeFilter;
    
    float search_radius = 0;

    geometry_msgs::Pose  odom_pose;
    geometry_msgs::Twist odom_twist;
    float uwb_distance = 0.0;

    int msg_cnt = 0;
    double timestamp = 0.0;
    long frame_seq = 0;
 
    PointType last_pose_1;
    bool pose_1_initialed;
    double last_pose_1_ts;
    PointType last_pose_2;
    bool pose_2_initialed;
    double last_pose_2_ts; 
    PointType last_pose_5;
    bool pose_5_initialed;
    double last_pose_5_ts; 
    PointType last_pose_10;
    bool pose_10_initialed; 
    double last_pose_10_ts;
    PointType last_pose_20;
    bool pose_20_initialed;
    double last_pose_20_ts; 
    PointType last_pose_50;
    bool pose_50_initialed;
    double last_pose_50_ts; 
    PointType last_pose_100;
    bool pose_100_initialed; 
    double last_pose_100_ts;
    PointType last_pose_200;
    bool pose_200_initialed; 
    double last_pose_200_ts;
    PointType last_pose_500;
    bool pose_500_initialed; 
    double last_pose_500_ts;

    geometry_msgs::Vector3 avg_speed;
    double avg_speed_10;
    ros::Time last_time;

public:
    AdapIntegraDetector():nh("~") {
          
        if(!nh.getParam("start_pose_x", StartPose.x)
            || !nh.getParam("start_pose_y", StartPose.y) 
            || !nh.getParam("start_pose_z", StartPose.z)){
            StartPose.x = 0.0;
            StartPose.y = 0.0;
            StartPose.z = 0.0;
        } 
        
        if(!nh.getParam("search_radius", search_radius)){
            search_radius = 0.2;
        }

        pose_1_initialed = false;
        pose_2_initialed = false;
        pose_5_initialed = false;
        pose_10_initialed = false;
        pose_20_initialed = false;
        pose_50_initialed = false;
        pose_100_initialed = false;
        pose_200_initialed = false;
        pose_500_initialed = false;
 
        ig_pts_cnt = 0;          // Real time valid integrated points of interest object
        valid_pts_th = 5; 
        update_pose = false;

        intgratingCloud.reset(new pcl::PointCloud<PointType>());
        interatedCloud.reset(new pcl::PointCloud<PointType>());
        tmpCloud.reset(new pcl::PointCloud<PointType>());
        tmpMap.reset(new pcl::PointCloud<PointType>());
        tmp_pcd.reset(new pcl::PointCloud<PointType>());
        accumulatePts.reset(new pcl::PointCloud<PointType>());

        interated_Cloud.reset(new pcl::PointCloud<PointType>());
        interated_2_Cloud.reset(new pcl::PointCloud<PointType>());
        interated_5_Cloud.reset(new pcl::PointCloud<PointType>());
        interated_10_Cloud.reset(new pcl::PointCloud<PointType>());
        interated_20_Cloud.reset(new pcl::PointCloud<PointType>());
        interated_50_Cloud.reset(new pcl::PointCloud<PointType>());
        interated_100Cloud.reset(new pcl::PointCloud<PointType>());
        interated_200Cloud.reset(new pcl::PointCloud<PointType>());
        interated_500Cloud.reset(new pcl::PointCloud<PointType>());

        interated_Cloud->header.frame_id = "/livox_frame";
        interated_2_Cloud->header.frame_id = "/livox_frame";
        interated_5_Cloud->header.frame_id = "/livox_frame";
        interated_10_Cloud->header.frame_id = "/livox_frame";
        interated_20_Cloud->header.frame_id = "/livox_frame";
        interated_50_Cloud->header.frame_id = "/livox_frame";
        interated_100Cloud->header.frame_id = "/livox_frame";
        interated_200Cloud->header.frame_id = "/livox_frame";
        interated_500Cloud->header.frame_id = "/livox_frame";

        intgratingCloud->header.frame_id = "/livox_frame";

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1, &AdapIntegraDetector::laserCloudHandler, this);
 
        pubSelectCloud = nh.advertise<pcl::PointCloud<PointType>>("/livox/selected_points", 2);
        pubObjPose = nh.advertise<geometry_msgs::PoseStamped>("/livox/drone_pose", 2);
        
        ig_pub = nh.advertise<pcl::PointCloud<PointType>>("/livox/ig_cloud", 2);
        ig_2_pub = nh.advertise<pcl::PointCloud<PointType>>("/livox/ig_2_cloud", 2);
        ig_5_pub = nh.advertise<pcl::PointCloud<PointType>>("/livox/ig_5_cloud", 2);
        ig_10_pub = nh.advertise<pcl::PointCloud<PointType>>("/livox/ig_10cloud", 2);
        ig_20_pub = nh.advertise<pcl::PointCloud<PointType>>("/livox/ig_20_cloud", 2);
        ig_50_pub = nh.advertise<pcl::PointCloud<PointType>>("/livox/ig_50_cloud", 2);
        ig_100pub = nh.advertise<pcl::PointCloud<PointType>>("/livox/ig_100cloud", 2);
        ig_200pub = nh.advertise<pcl::PointCloud<PointType>>("/livox/ig_200cloud", 2);
        ig_500pub = nh.advertise<pcl::PointCloud<PointType>>("/livox/ig_500cloud", 2);
        
    }

    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){ 

        timestamp = laserCloudMsg->header.stamp.toSec();
        last_time = laserCloudMsg->header.stamp;
        // std::cout << timestamp << std::endl;
        frame_seq = laserCloudMsg->header.seq;
 
        tmpCloud->clear();
        pcl::fromROSMsg(*laserCloudMsg, *tmpCloud); 

        bool is_debuging = false;

        if(!is_debuging){
            PointType current_pose; 

            obj_pts_extraction(tmpCloud, search_radius, current_pose, last_time);   

            if(ig_2_cnt < 2){
                *interated_2_Cloud = *interated_2_Cloud + *tmpCloud;
                ig_2_cnt++;
            }else{
                ig_2_pub.publish(*interated_2_Cloud);
                obj_pts_extraction(interated_2_Cloud, search_radius, current_pose, last_time); 
                interated_2_Cloud->clear();
                ig_2_cnt = 0;
            } 

            if(ig_5_cnt < 5){
                *interated_5_Cloud = *interated_5_Cloud + *tmpCloud;
                ig_5_cnt++;
            }else{
                ig_5_pub.publish(*interated_5_Cloud);
                obj_pts_extraction(interated_5_Cloud, search_radius, current_pose, last_time); 
                interated_5_Cloud->clear();
                ig_5_cnt = 0;
            }  

            if(ig_10_cnt < 10){
                *interated_10_Cloud = *interated_10_Cloud + *tmpCloud;
                ig_10_cnt++;
            }else{
                ig_10_pub.publish(*interated_10_Cloud);
                obj_pts_extraction(interated_10_Cloud, search_radius, current_pose, last_time); 
                interated_10_Cloud->clear();
                ig_10_cnt = 0;
            } 

            if(ig_20_cnt < 20){
                *interated_20_Cloud = *interated_20_Cloud + *tmpCloud;
                ig_20_cnt++;
            }else{
                ig_20_pub.publish(*interated_20_Cloud);
                obj_pts_extraction(interated_20_Cloud, search_radius, current_pose, last_time); 
                interated_20_Cloud->clear();
                ig_20_cnt = 0;
            }
 
            if(ig_50_cnt < 50){
                *interated_50_Cloud = *interated_50_Cloud + *tmpCloud;
                ig_50_cnt++;
            }else{
                ig_50_pub.publish(*interated_50_Cloud);
                obj_pts_extraction(interated_50_Cloud, 0.8, current_pose, last_time); 
                interated_50_Cloud->clear();
                ig_50_cnt = 0;
            }

            if(ig_100cnt < 100){
                *interated_100Cloud = *interated_100Cloud + *tmpCloud;
                ig_100cnt++;
            }else{
                ig_100pub.publish(*interated_100Cloud);
                obj_pts_extraction(interated_100Cloud,  search_radius, current_pose, last_time);
                interated_100Cloud->clear();
                ig_100cnt = 0;
            }

            if(ig_200cnt < 200){
                *interated_200Cloud = *interated_200Cloud + *tmpCloud;
                ig_200cnt++;
            }else{
                ig_200pub.publish(*interated_200Cloud);
                obj_pts_extraction(interated_200Cloud,  search_radius, current_pose, last_time);
                interated_200Cloud->clear();
                ig_200cnt = 0;
            }
 
        }else{
            
            obj_points_PF_extraction(tmpCloud, 1, timestamp, frame_seq);

            if(ig_2_cnt < 2){
                *interated_2_Cloud = *interated_2_Cloud + *tmpCloud;
                ig_2_cnt++;
            }else{
                ig_2_pub.publish(*interated_2_Cloud); 
                obj_points_PF_extraction(interated_2_Cloud, 2, timestamp, frame_seq);
                interated_2_Cloud->clear();
                ig_2_cnt = 0;
            }

            if(ig_5_cnt < 5){
                *interated_5_Cloud = *interated_5_Cloud + *tmpCloud;
                ig_5_cnt++;
            }else{
                ig_5_pub.publish(*interated_5_Cloud);
                obj_points_PF_extraction(interated_5_Cloud, 5, timestamp, frame_seq);
                interated_5_Cloud->clear();
                ig_5_cnt = 0;
            }

            if(ig_10_cnt < 10){
                *interated_10_Cloud = *interated_10_Cloud + *tmpCloud;
                ig_10_cnt++;
            }else{
                ig_10_pub.publish(*interated_10_Cloud);
                obj_points_PF_extraction(interated_10_Cloud, 10, timestamp, frame_seq);
                interated_10_Cloud->clear();
                ig_10_cnt =  0;
            }

            if(ig_20_cnt < 20){
                *interated_20_Cloud = *interated_20_Cloud + *tmpCloud;
                ig_20_cnt++;
            }else{
                ig_20_pub.publish(*interated_20_Cloud);
                obj_points_PF_extraction(interated_20_Cloud, 20, timestamp, frame_seq);
                interated_20_Cloud->clear();
                ig_20_cnt = 0;
            }


            if(ig_50_cnt < 50){
                *interated_50_Cloud = *interated_50_Cloud + *tmpCloud;
                ig_50_cnt++;
            }else{
                ig_50_pub.publish(*interated_50_Cloud);
                obj_points_PF_extraction(interated_50_Cloud, 50, timestamp, frame_seq);
                interated_50_Cloud->clear();
                ig_50_cnt = 0;
            }

            if(ig_100cnt < 100){
                *interated_100Cloud = *interated_100Cloud + *tmpCloud;
                ig_100cnt++;
            }else{
                ig_100pub.publish(*interated_100Cloud);
                obj_points_PF_extraction(interated_100Cloud, 100, timestamp, frame_seq);
                interated_100Cloud->clear();
                ig_100cnt = 0;
            }

            if(ig_200cnt < 200){
                *interated_200Cloud = *interated_200Cloud + *tmpCloud;
                ig_200cnt++;
            }else{
                ig_200pub.publish(*interated_200Cloud);
                obj_points_PF_extraction(interated_200Cloud, 200, timestamp, frame_seq);
                interated_200Cloud->clear();
                ig_200cnt = 0;
            }
 
        } 
    }

    // # Extract Points and updating pose
    void obj_pts_extraction(pcl::PointCloud<PointType>::Ptr interatedCloud, float radius, PointType LastPose, ros::Time timestamp){
        
        pcl::KdTreeFLANN<PointType>::Ptr kdtreePts(new pcl::KdTreeFLANN<PointType>());
        // kdtreePts.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreePts->setInputCloud(interatedCloud); 
        long currt_stamp = ros::Time::now().toNSec();

        pcl::PointCloud<PointType>::Ptr selectedCloud ; 
        selectedCloud.reset(new pcl::PointCloud<PointType>());
        selectedCloud->header.frame_id = "/livox_frame"; 
        pcl_conversions::toPCL(timestamp, selectedCloud->header.stamp);

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        PointType serach_pose = StartPose;
        std::cout << serach_pose.x << ","<< serach_pose.y << "," << serach_pose.z << std::endl;
        
        serach_pose.x = serach_pose.x +  1.0 * avg_speed.x ; // 10hz 

        if ( kdtreePts->radiusSearch(StartPose, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
            float sum_x, sum_y, sum_z= 0; 
            int points_num = 0;

            for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
                PointType pt = (*interatedCloud)[ pointIdxRadiusSearch[i] ];
                // Remove ground points
                if(pt.z > -0.165 && pt.y > -2.1 &&  pt.y < 1.75 &&  pt.x < 6.2){ // Circle 
                    selectedCloud->points.push_back(pt); 
                    // sum_x += pt.x; sum_y += pt.y; sum_z += pt.z;
                    points_num ++ ;
                }
            } 
            points_num = pointIdxRadiusSearch.size ();
            std::cout<< currt_stamp << "==> Thread ID: " << std::this_thread::get_id() << " | "<< points_num   << std::endl;

        }

        // TODO Clustering Object
        bool is_clustering = false;
        if(is_clustering){
            pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);
            pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
            tree->setInputCloud (selectedCloud);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<PointType> ec;
            ec.setClusterTolerance (0.1); // 2cm
            ec.setMinClusterSize (2);
            ec.setMaxClusterSize (100);
            ec.setSearchMethod (tree);
            ec.setInputCloud (selectedCloud);
            ec.extract (cluster_indices); 

            std::vector<pcl::PointCloud<PointType>> clusters_vec;
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {   
                pcl::PointCloud<PointType>::Ptr cloud_cluster (new pcl::PointCloud<PointType>);
                cloud_cluster->header.frame_id = "livox_frame"; 
                bool wrong_target = false;
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){ 
                    if( ((*selectedCloud)[*pit].z < serach_pose.z - 0.2) || ((*selectedCloud)[*pit].z > serach_pose.z + 0.2)){
                        wrong_target = true;
                        continue;
                    }
                    cloud_cluster->push_back((*selectedCloud)[*pit]);
                }
                clusters_vec.push_back(*cloud_cluster);

                cloud_cluster->width = cloud_cluster->size ();
                cloud_cluster->height = 1; 
                cloud_cluster->is_dense = true; 

                ig_pub.publish(*cloud_cluster);
            } 
        }

        if(!selectedCloud->points.empty()){ 
            float sum_x = 0, sum_y = 0, sum_z= 0; 
            int pt_cnt = 0;
            for (std::size_t i = 0; i < selectedCloud->size (); ++i){
                PointType pt = selectedCloud->points[i];
                if(pt.z > -0.4){
                    pt_cnt++;
                    sum_x += pt.x; sum_y += pt.y; sum_z += pt.z; 
                }
            }  
            if(pt_cnt > 0){
                PointType new_pose;
                new_pose.x= sum_x / pt_cnt; new_pose.y = sum_y / pt_cnt; new_pose.z = sum_z / pt_cnt; 
                std::unique_lock<std::mutex> lck1(update_pose_mutex); 
                if(lck1.owns_lock()){
                    if(std::abs(new_pose.x - StartPose.x < 0.5 )) 
                        { 
                        StartPose = new_pose;
                        lck1.unlock(); 

                        geometry_msgs::PoseStamped pose; 
                        pcl_conversions::fromPCL(selectedCloud->header.stamp, pose.header.stamp);
                        pose.header.frame_id = "/livox_frame";
                        pose.pose.position.x = new_pose.x;
                        pose.pose.position.y = new_pose.y;
                        pose.pose.position.z = new_pose.z; 

                        if( objPose_vec.size() > 1){
                            geometry_msgs::PoseStamped last_pose = objPose_vec.back();
                            std::pair<geometry_msgs::Vector3, geometry_msgs::Quaternion> info_pair = get_speed(last_pose, pose);

                            geometry_msgs::Vector3 speed = info_pair.first;
                            avg_speed = speed;

                            geometry_msgs::Quaternion orientation = info_pair.second;
                            pose.pose.orientation = orientation; 
                        }
                        pubObjPose.publish(pose);
                        objPose_vec.push_back(pose);

                        std::string result= std::to_string(timestamp.toSec())  + ',' + 
                        std::to_string( selectedCloud->points.size() ) + ',' + 
                        std::to_string( new_pose.x) + "," + std::to_string( new_pose.y) +  ","  + 
                        std::to_string( new_pose.z) + "," + std::to_string(  avg_speed.x) + "," + 
                        std::to_string( avg_speed.y)   + "," + std::to_string(  avg_speed.z);

                        // Write to file
                        std::ofstream outfile; 
                        std::string file_name = "/home/qing/Desktop/IROS2021/result/loop2_result_1.csv";
                        outfile.open(file_name, std::ios::app);

                        outfile <<  result << std::endl;
                        outfile.close();

                    }
                    
                }else{
                    ROS_WARN("Can't get the locker of Pose");
                }

            }else{
                ROS_INFO("Current PointCLoud is empty!");
            } 

            pubSelectCloud.publish(*selectedCloud);
         }

    }
    
    //TODO
    void clusters_seperate( std::vector<pcl::PointCloud<PointType>> clusters_vec){
        for (std::vector<pcl::PointCloud<PointType>>::const_iterator it = clusters_vec.begin (); it != clusters_vec.end (); ++it){
            for(int i=0; i < it->size(); i++){ 
            } 
        } 

    }

    std::pair<geometry_msgs::Vector3, geometry_msgs::Quaternion> get_speed(  geometry_msgs::PoseStamped last, geometry_msgs::PoseStamped current){ 
        double delta_time = current.header.stamp.toSec() - last.header.stamp.toSec();

        geometry_msgs::Vector3 speed;
        speed.x = ((current.pose.position.x - last.pose.position.x) /delta_time );
        speed.y = ((current.pose.position.y - last.pose.position.y) /delta_time );
        speed.z = ((current.pose.position.z - last.pose.position.z) /delta_time );

        // # Oritention
        double theta = atan2( (current.pose.position.y - last.pose.position.y) , (current.pose.position.x - last.pose.position.x) );
   
        double angle = theta * 180 / (M_PI);
 
        // std::cout << delta_time << " | " << delta_time <<  "/ Speed : " << speed.x << " " << speed.y << " " << speed.z << " | Yaw : " << angle << std::endl;

        tf2::Quaternion quat_tf;
        quat_tf.setRPY( 0, 0, theta );  // Create this quaternion from roll/pitch/yaw (in radians)
        geometry_msgs::Quaternion quat_msg; 
        quat_msg = tf2::toMsg(quat_tf); 
 
        std::pair<geometry_msgs::Vector3, geometry_msgs::Quaternion> info = std::make_pair( speed, quat_msg);
        return info;
    }
    
     // # Extract Points with pathFilter without updating pose 
    void obj_points_PF_extraction(pcl::PointCloud<PointType>::Ptr interatedCloud, int range, double timestamp, long frameseq ){
        
        // PassFilter x (3.0, 25.0); y (0.0, 1.0); z (-0.3, 0.0);  
        //pasttFilter tello flying 123 (4.1, 8.0)  (-4, 4.0) (0, 1.6)
        //pasttFilter tello flying 4 (2, 27)   (-0.7, 1.0) (-0.5 , 1.20)

        //Circle 
        // Create the filtering object
        pcl::PassThrough<PointType> pass;
        pcl::PointCloud<PointType>::Ptr selectedCloud ; 
        selectedCloud.reset(new pcl::PointCloud<PointType>());
        selectedCloud->header.frame_id = "/livox_frame"; 
 
        // /Circle 
        pcl::PointCloud<PointType>::Ptr x_filtered_cloud( new pcl::PointCloud<PointType>() ); 
        pass.setInputCloud(interatedCloud); 
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0, 6.46); 
        pass.filter(*x_filtered_cloud);

        pcl::PointCloud<PointType>::Ptr y_filtered_cloud( new pcl::PointCloud<PointType>() ); 
        pass.setInputCloud(x_filtered_cloud); 
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-2.1, 1.7); 
        pass.filter(*y_filtered_cloud);

        pass.setInputCloud(y_filtered_cloud); 
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.171, 0.875); 
        pass.filter(*selectedCloud); 

        pubSelectCloud.publish(*selectedCloud);
        return;

        //Caculate the position
        PointType current_pose;
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_z = 0.0;
        for( int i = 0; i < selectedCloud->points.size(); i++){
            sum_x = sum_x + selectedCloud->points[i].x;
            sum_y = sum_y +  selectedCloud->points[i].y;
            sum_z = sum_z +  selectedCloud->points[i].z;
        } 
        current_pose.x = sum_x / selectedCloud->points.size();
        current_pose.y = sum_y / selectedCloud->points.size();
        current_pose.z = sum_z / selectedCloud->points.size();

        // Caculate the speed
        std::pair<double, double> dis_speed_pair;

        if(range == 1){
            //result_1_vec.push_back(result);
            if(!pose_1_initialed){
                pose_1_initialed = true;
                last_pose_1_ts = timestamp;
                last_pose_1 = current_pose;
                }
            else{
                dis_speed_pair= caculate_speed(last_pose_1, last_pose_1_ts, current_pose, timestamp);
                last_pose_1_ts = timestamp;
                last_pose_1 = current_pose;
            }
        }else if(range == 2){
            //result_2_vec.push_back(result);
            if(!pose_2_initialed){
                pose_2_initialed = true;
                last_pose_2_ts = timestamp;
                last_pose_2 = current_pose;}
            else{
                dis_speed_pair= caculate_speed(last_pose_2, last_pose_2_ts, current_pose, timestamp);
                last_pose_2_ts = timestamp;
                last_pose_2 = current_pose;}
        }else if(range == 5){
            //result_5_vec.push_back(result);
            if(!pose_5_initialed){
                pose_5_initialed = true;
                last_pose_5_ts = timestamp;
                last_pose_5 = current_pose;}
            else{
                dis_speed_pair= caculate_speed(last_pose_5, last_pose_5_ts, current_pose, timestamp);
                last_pose_5_ts = timestamp;
                last_pose_5 = current_pose;}
        }else if(range == 10){
            //result_10_vec.push_back(result);
            if(!pose_10_initialed){
                pose_10_initialed = true;
                last_pose_10_ts = timestamp;
                last_pose_10 = current_pose;}
            else{
                dis_speed_pair= caculate_speed(last_pose_10, last_pose_10_ts, current_pose, timestamp);
                last_pose_10_ts = timestamp;
                last_pose_10 = current_pose;
                avg_speed_10 = dis_speed_pair.second;
                std::cout << "P/V: (" <<current_pose.x <<','<< current_pose.y <<',' << current_pose.z << " )  " << dis_speed_pair.second<< std::endl;
                }
        }else if(range == 20){
            //result_20_vec.push_back(result);
            if(!pose_20_initialed){
                pose_20_initialed = true;
                last_pose_20_ts = timestamp;
                last_pose_20 = current_pose;}
            else{
                dis_speed_pair= caculate_speed(last_pose_20, last_pose_20_ts, current_pose, timestamp);
                last_pose_20_ts = timestamp;
                last_pose_20 = current_pose;}
        }else if(range == 50){
            //result_50_vec.push_back(result);
            if(!pose_50_initialed){
                pose_50_initialed = true;
                last_pose_50_ts = timestamp;
                last_pose_50 = current_pose;}
            else{
                dis_speed_pair= caculate_speed(last_pose_50, last_pose_50_ts, current_pose, timestamp);
                last_pose_50_ts = timestamp;
                last_pose_50 = current_pose;}
        }else if(range == 100){
            //result_100_vec.push_back(result);
            if(!pose_100_initialed){
                pose_100_initialed = true;
                last_pose_100_ts = timestamp;
                last_pose_100 = current_pose;}
            else{
                dis_speed_pair= caculate_speed(last_pose_100, last_pose_100_ts, current_pose, timestamp);
                last_pose_100_ts = timestamp;
                last_pose_100 = current_pose;}
        }else if(range == 200){
            //result_200_vec.push_back(result);
            if(!pose_200_initialed){
                pose_200_initialed = true;
                last_pose_200_ts = timestamp;
                last_pose_200 = current_pose;}
            else{
                dis_speed_pair= caculate_speed(last_pose_200, last_pose_200_ts, current_pose, timestamp);
                last_pose_200_ts = timestamp;
                last_pose_200 = current_pose;
                
                // Write to file
                std::ofstream outfile; 
                std::string file_name = "/home/qing/Desktop/IROS2021/result/convex_hull_adp_200/pts_" + std::to_string(timestamp) + ".csv";
                outfile.open(file_name, std::ios::app);
                for(int i = 0; i < selectedCloud->points.size(); i++){
                    outfile <<  selectedCloud->points[i].x << ',' << selectedCloud->points[i].y << ',' << selectedCloud->points[i].z << std::endl;
                }
                outfile.close(); 
                }
        }else if(range == 500){
            //result_200_vec.push_back(result);
            if(!pose_500_initialed){
                pose_500_initialed = true;
                last_pose_500_ts = timestamp;
                last_pose_500 = current_pose;}
            else{
                dis_speed_pair= caculate_speed(last_pose_500, last_pose_500_ts, current_pose, timestamp);
                last_pose_500_ts = timestamp;
                last_pose_500 = current_pose;
                // Write to file
                std::ofstream outfile; 
                std::string file_name = "/home/qing/Desktop/IROS2021/result/convex_hull_adp_500/pts_" + std::to_string(timestamp) + ".csv";
                outfile.open(file_name, std::ios::app); 
                for(int i = 0; i < selectedCloud->points.size(); i++){
                    outfile <<  selectedCloud->points[i].x << ',' << selectedCloud->points[i].y << ',' << selectedCloud->points[i].z << std::endl;
                }
                outfile.close(); 
                }
        }else{
            ROS_WARN_STREAM("WARN INPUT " << range);
        }
        
        // std::string result= std::to_string(timestamp) + ',' + std::to_string(frameseq) + ',' + 
        //         std::to_string( selectedCloud->points.size() ) + "," + std::to_string( odom_twist.linear.x ) + "," +
        //         std::to_string( odom_twist.linear.y) + ","  +  std::to_string( odom_twist.linear.z) +  ","  + 
        //         std::to_string( odom_pose.position.x) + ","  +  std::to_string( odom_pose.position.y) +  ","  + 
        //         std::to_string( uwb_distance );
        std::string result= std::to_string(timestamp) + ',' + std::to_string(frameseq) + ',' + 
                            std::to_string( selectedCloud->points.size() ) + "," + std::to_string( avg_speed_10) + "," +
                            std::to_string( 0) + ","  +  std::to_string( 0 ) +  ","  + 
                            std::to_string( current_pose.x) + ","  +  std::to_string( current_pose.y) +  ","  + 
                            std::to_string( dis_speed_pair.first );

        // Write to file
        std::ofstream outfile; 
        std::string file_name = "./result/adp_" + std::to_string(range) + "_result.csv";
        outfile.open(file_name, std::ios::app);

        outfile <<  result << std::endl;
        outfile.close();
  
        if(selectedCloud->points.size() > 0){  
            pubSelectCloud.publish(*selectedCloud);
            selectedCloud->clear(); 
        }
    }
    
    std::pair<double, double> caculate_speed( PointType last_pose, double last_pose_timestamp, PointType current_pose, double current_timestamp){
        double dx = current_pose.x - last_pose.x;
        double dy = current_pose.y - last_pose.y;
        double dz = current_pose.z - last_pose.z;

        double dis = sqrt(current_pose.x * current_pose.x + current_pose.y * current_pose.y +  current_pose.z * current_pose.z);
        double ds = sqrt(dx * dx + dy * dy + dz * dz); 

        std::pair<double, double> info = std::make_pair( dis, ds / (current_timestamp - last_pose_timestamp));
        return info;
    }

    void uwbPoseCloudHandler(const std_msgs::String uwbPoseMsg){
        std::string data(uwbPoseMsg.data);
        uwb_distance = std::stof(data);
        // std::cout << "uwb_distance : " << uwb_distance << std::endl;
    }
 

    void uwbPoseCloudHandler(const geometry_msgs::PoseConstPtr& uwbPoseMsg){
        geometry_msgs::PoseStamped pose_stamped ;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "uwb";
        pose_stamped.pose.position.x = uwbPoseMsg->position.x;
        pose_stamped.pose.position.y = uwbPoseMsg->position.y;
        pose_stamped.pose.position.z = uwbPoseMsg->position.z;

        // Update object's uwb pose
        uwb_pose = pose_stamped;
        //uwbPose_vec.push_back(pose_stamped);
    }
}; 


int main(int argc, char** argv)
{
    ros::init(argc, argv, "livox_drone_detector"); 

    class AdapIntegraDetector DD;

    ros::MultiThreadedSpinner spinner(15);
    spinner.spin();
 
    return 0;
}

