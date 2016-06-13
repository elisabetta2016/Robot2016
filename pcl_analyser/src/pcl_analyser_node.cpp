#include <ros/ros.h>
#include <ros/timer.h>
#include <math.h>
#include <iostream>
#include <string>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/filters/filter.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

// Messages
#include <sensor_msgs/PointCloud2.h>
#include "donkey_rover/Rover_Scanner.h"

#include <sensor_msgs/point_cloud_conversion.h>

// PCL keypoints headers
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/range_image/range_image.h>

#include <pcl/keypoints/iss_3d.h>

typedef pcl::PointXYZ PointType;
// Range image coordinate frame 
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;


class ObstacleDetectorClass
{
	public:
		
	ObstacleDetectorClass(ros::NodeHandle& node)
	{
			n_=node;

			//subscribers
			//SubFromCloud_		 = n_.subscribe("cloud_in", 1, &ObstacleDetectorClass::cloud_call_back,this);
			SubFromCloud_		 = n_.subscribe("/camera/depth/points", 1, &ObstacleDetectorClass::cloud_call_back,this);
			
			
			// publishers
			obstcle_pub_		  = n_.advertise<sensor_msgs::PointCloud2> ("obstacle_cloud", 1);
			obstcle_proj_pub_	  = n_.advertise<sensor_msgs::PointCloud2> ("obstacle_proj_cloud", 1);
			keypoint_pub_		  = n_.advertise<sensor_msgs::PointCloud2> ("keypoints", 1);
			ISS_keypoint_pub_	  = n_.advertise<sensor_msgs::PointCloud2> ("ISS_keypoints", 1);
			
			


    			
    			// Range image params
    			
			support_size = 0.4f;
			setUnseenToMaxRange = false;
			

    			
    			
	}
	
	

	
	void ketpoint_extract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
	{
		ROS_WARN("NARF KEYPOINT DETECTION STARTS");
		Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
		
                
		float noise_level = 0.0;
  		float min_range = 0.0f;
  		int border_size = 1;
  		boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
  		pcl::RangeImage& range_image = *range_image_ptr;
  		angular_resolution = pcl::deg2rad (1.0f);
  		//angular_resolution = 0.5f; 
  		range_image.createFromPointCloud (*cloud_in, angular_resolution, pcl::deg2rad (180.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  		//pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
  		
  		//range_image.integrateFarRanges (far_ranges);
  		if (setUnseenToMaxRange)
    			range_image.setUnseenToMaxRange ();
    			
    		pcl::RangeImageBorderExtractor range_image_border_extractor;
  		pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
  		narf_keypoint_detector.setRangeImage (&range_image);
  		narf_keypoint_detector.getParameters ().support_size = support_size;
  		
  		//std::cout << range_image << "\n";
  		
  		
  		pcl::PointCloud<int> keypoint_indices;
  		narf_keypoint_detector.compute (keypoint_indices);
  		std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";
  		
  		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  		pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
  		keypoints.points.resize (keypoint_indices.points.size ());
  		for (size_t i=0; i<keypoint_indices.points.size (); ++i)
  		{
    			keypoints.points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();
    			//if (i==1) std::cout << range_image.points[keypoint_indices.points[i]].getVector3fMap () << "\n";
    		}
    		

    		pcl::toROSMsg(keypoints,key_cloud);
    		key_cloud.header.frame_id = "base_link";
    		key_cloud.header.stamp = ros::Time::now();
    		keypoint_pub_.publish(key_cloud);
    		ROS_INFO("Keypoints Published");

    		
	}
	
	void ISS_keypoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_cloud)
	{
		ROS_WARN("ISS KEYPOINT DETECTION STARTS");
		float support_radius = 0.08f;
		float nms_radius = 0.04f;
		
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
		pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;
		iss_detector.setSearchMethod (tree);
		iss_detector.setSalientRadius (support_radius);
		iss_detector.setNonMaxRadius (nms_radius);
		iss_detector.setInputCloud (cloud_in);
		
		iss_detector.compute (*keypoints_cloud);
		ROS_INFO(" Number of Keypoints found:");
		std::cout << keypoints_cloud->points.size() << "\n";
		pcl::toROSMsg(*keypoints_cloud,key_cloud);
		key_cloud.header.frame_id = "base_link";
    		key_cloud.header.stamp = ros::Time::now();
		ISS_keypoint_pub_.publish(key_cloud);
	}
	
	
	void cloud_voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,float cube_size)
	{
		
  		pcl::VoxelGrid<pcl::PointXYZ> VG;
  		VG.setInputCloud (cloud_in);
  		VG.setLeafSize (cube_size, cube_size, cube_size);
 		VG.filter (*cloud_out);	
	
	}	

	void cloud_outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
	{
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  		sor.setInputCloud (cloud_in);
  		sor.setMeanK (50);
  		sor.setStddevMulThresh (1.0);
  		sor.filter (*cloud_filtered);
	}
  	

	
	void obstacle_find_Publish(const sensor_msgs::PointCloud2ConstPtr& cloud)
	{
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_pcl (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::fromROSMsg (*cloud, *cloud_pcl);
	
 	
 	cloud_voxel_filter(cloud_pcl,cloud_filtered_pcl,0.1);
  
  
  
  
  	// Create the normal estimation class, and pass the input dataset to it
 	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  	ne.setInputCloud (cloud_pcl);
  	ne.setViewPoint(0,0,10);

  	// Create an empty kdtree representation, and pass it to the normal estimation object.
  	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  	ne.setSearchMethod (tree);

  	// normal datasets
  	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  	// Use all neighbors in a sphere of radius 10cm
  	ne.setRadiusSearch (0.10);

  	// Compute the features
  	ne.compute (*cloud_normals);
  	//std::cout << cloud_normals->points.size() << "\n";
  
  
  	//Defining the Kdtree for finding the indicies
  	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  	kdtree.setInputCloud (cloud_pcl);
  	// Defining  the search point
  	pcl::PointXYZ searchPoint;

	// Starting to search and insert
	pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_pcl (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr obs_projected (new pcl::PointCloud<pcl::PointXYZ>);
	int K = 1;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	for (size_t i = 0; i < cloud_filtered_pcl->points.size (); ++i)
	{
		searchPoint.x = cloud_filtered_pcl->points[i].x;
		searchPoint.y = cloud_filtered_pcl->points[i].y;
		searchPoint.z = cloud_filtered_pcl->points[i].z;
     		if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  		{
  	
  	
			for (size_t j = 0; j < pointIdxNKNSearch.size (); ++j)
			{

				
				if(cloud_normals->points[pointIdxNKNSearch[j]].normal_z < normal_threshold &&
			       		cloud_pcl->points[pointIdxNKNSearch[j]].z        > height_threshold &&
			       		cloud_pcl->points[pointIdxNKNSearch[j]].z        < height_max)
			       	{
			
					obstacle_pcl->points.push_back (cloud_pcl->points[ pointIdxNKNSearch[j] ]);
				}
			}		

		}
	
	}
	obstacle_pcl->width = int (obstacle_pcl->points.size ());
	obstacle_pcl->height = 1;
	obstacle_pcl->is_dense = true;
	

	pcl::toROSMsg(*obstacle_pcl,output_cloud);
	
    	output_cloud.header.frame_id = "base_link";
    	output_cloud.header.stamp = ros::Time::now();	
    	
	ROS_INFO("Nummber of points in obstacle_cloud is:");
	std::cout << obstacle_pcl->points.size() << "\n";
	
	obstcle_pub_.publish(output_cloud);
	
	cloud_2D_projection(obstacle_pcl,obs_projected);
	
	pcl::toROSMsg(*obs_projected,cloud_obstacle_projected);
    	cloud_obstacle_projected.header.frame_id = "base_link";
    	cloud_obstacle_projected.header.stamp = ros::Time::now();		
	
	
	}
	
	void cloud_2D_projection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
	{
	
	// Create a set of planar coefficients with X=Y=0,Z=1
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	coefficients->values.resize (4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	// Create the filtering object
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (cloud_in);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud_out);
		
	}
	
	void cloud_call_back(const sensor_msgs::PointCloud2ConstPtr& cloud)
	{
		//obstacle_find_Publish(cloud);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>);
		//pcl::PointCloud<pcl::PointXYZ>::Ptr key_pcl (new pcl::PointCloud<pcl::PointXYZ>);
		std::vector<int> mapping;
		pcl::removeNaNFromPointCloud(*cloud_pcl, *cloud_pcl, mapping);
		pcl::fromROSMsg (*cloud, *cloud_pcl);
		cloud_voxel_filter(cloud_pcl,cloud_pcl,0.05);
		ketpoint_extract(cloud_pcl);
		//ISS_keypoint(cloud_pcl,key_pcl);
	
	}
	
	void run()
	{
	
		double normal_threshold_default = 0.7;
		double height_threshold_default = 0.0;
		double height_max_default = 2.0;
		
		ros::NodeHandle n("~");
		
		n.param("normal_threshold", normal_threshold, normal_threshold_default);
		n.param("height_threshold", height_threshold, height_threshold_default);
		n.param("height_max", height_max,height_max_default);
		
  
		if (normal_threshold != normal_threshold_default) ROS_INFO_ONCE("normal threshold is changed to %f", normal_threshold);
		if (height_threshold != height_threshold_default) ROS_INFO_ONCE("height threshold is changed to %f", height_threshold);
		if (height_max       != height_max_default)       ROS_INFO_ONCE("height threshold is changed to %f", height_max);
		
		ros::spin ();
	
	}
	
	
	protected:
	
		// Node Handler
		ros::NodeHandle n_;
		
		// Subscribers
		ros::Subscriber SubFromCloud_;
		
		
		// Publishers
		ros::Publisher obstcle_pub_;
		ros::Publisher obstcle_proj_pub_;
		ros::Publisher Scanner_commander_pub_;
		ros::Publisher keypoint_pub_;
		ros::Publisher ISS_keypoint_pub_;

		
		//Class Global Variables

		double normal_threshold;
		double height_threshold;
		double height_max;
		sensor_msgs::PointCloud2 output_cloud; 
		sensor_msgs::PointCloud2 cloud_obstacle_projected; 
		sensor_msgs::PointCloud2 key_cloud;

		
		float angular_resolution;
		float support_size;
		bool setUnseenToMaxRange;
		
		
		
		
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcl_analyser");
	ros::NodeHandle node;

	ObstacleDetectorClass Obstacle_rec(node);
	
	Obstacle_rec.run();
	
	return 0;
}
