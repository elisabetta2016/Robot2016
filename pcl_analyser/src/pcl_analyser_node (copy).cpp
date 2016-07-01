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
#include <geometry_msgs/Vector3.h>

#include <sensor_msgs/point_cloud_conversion.h>

// PCL keypoints headers
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/range_image/range_image.h>

#include <pcl/keypoints/iss_3d.h>
#include <pcl/common/transforms.h>
//Costmap

#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>

// TF
#include <tf/transform_listener.h>
#include <Eigen/Dense> 


typedef pcl::PointXYZ PointType;
// Range image coordinate frame 
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
using costmap_2d::LETHAL_OBSTACLE;





class ObstacleDetectorClass
{
	public:
		
	ObstacleDetectorClass(ros::NodeHandle& node)
	{
			n_=node;

			//subscribers
			//SubFromCloud_		 = n_.subscribe("cloud_in", 1, &ObstacleDetectorClass::cloud_call_back,this);
			SubFromCloud_		 = n_.subscribe("/RL_cloud", 1, &ObstacleDetectorClass::cloud_call_back,this);
			
			
			// publishers
			obstcle_pub_		  = n_.advertise<sensor_msgs::PointCloud2> ("obstacle_cloud", 1);
			obstcle_proj_pub_	  = n_.advertise<sensor_msgs::PointCloud2> ("obstacle_proj_cloud", 1);
			cost_map_cl_pub_	  = n_.advertise<sensor_msgs::PointCloud2> ("costmap_cloud", 1);
			
			repuslive_force_pub_	  = n_.advertise<geometry_msgs::Vector3> ("force", 1);
			
			//keypoint_pub_		  = n_.advertise<sensor_msgs::PointCloud2> ("keypoints", 1);
			//ISS_keypoint_pub_	  = n_.advertise<sensor_msgs::PointCloud2> ("ISS_keypoints", 1);
			
			


    			
    			// Range image params
    			
			support_size = 0.4f;
			setUnseenToMaxRange = false;
			
			//Initializer
			repulsive_force.x = 0.0;
			repulsive_force.y = 0.0;
			repulsive_force.z = 0.0;
			
			//costmap params
			costmap_x_size = 6.0; // meters
			costmap_y_size = 6.0; // meters
			costmap_res = 0.2;    // meters/cell
			
			cell_x = (unsigned int) floor(abs(costmap_x_size/costmap_res));
			cell_y = (unsigned int) floor(abs(costmap_y_size/costmap_res));
			
			master_grid_ = new costmap_2d::Costmap2D(cell_x,cell_y,costmap_res,-1.0,-3.0,0);
			n = &n_;
    			
    			global_frame = "laser";
			topic_name = "/global_costmap";
			
			master_grid_ros = new costmap_2d::Costmap2DPublisher(n,master_grid_,global_frame,topic_name,false);
			
    			
	}
	
	

	/*
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

    		
	}*/
	/*
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
	*/
	
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
  	
  	void compute_repulsive_force(pcl::PointCloud<pcl::PointXYZ>::Ptr obs_proj)
  	{
  	
  	for (size_t i = 0; i < obs_proj->points.size (); ++i)
		{
			repulsive_force.x +=  1/obs_proj->points[i].x;
			repulsive_force.y +=  1/obs_proj->points[i].y;
			
		}
		//float norm = sqrtf(powf(repulsive_force.x,2)+powf(repulsive_force.y,2));
		//repulsive_force.x = repulsive_force.x/norm;
  		//repulsive_force.y = repulsive_force.y/norm;
  	}

	
	void obstacle_find_Publish(const sensor_msgs::PointCloud2ConstPtr& cloud)
	{
	ROS_INFO("Obstacle detection Starts!");
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_pcl (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::fromROSMsg(*cloud, *cloud_pcl);
	
 	//cloud_voxel_filter(cloud_pcl,cloud_pcl,0.03);
 	//cloud_outlier_removal(cloud_pcl,cloud_pcl);
 	//cloud_voxel_filter(cloud_pcl,cloud_filtered_pcl,0.03);
 	*cloud_filtered_pcl = *cloud_pcl;
  	//cloud_outlier_removal(cloud_filtered_pcl,cloud_filtered_pcl);
  
  
  
  	// Create the normal estimation class, and pass the input dataset to it
 	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  	ne.setInputCloud (cloud_pcl);
  	ne.setViewPoint(0,0,0.3);

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
	pcl::PointCloud<pcl::PointXYZ> obstacle_nomal_pcl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr obs_projected (new pcl::PointCloud<pcl::PointXYZ>);
	int K = 1;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	
	obstacle_nomal_pcl = *cloud_filtered_pcl;

	for (size_t i = 0; i < cloud_filtered_pcl->points.size (); ++i)
	{
		searchPoint.x = cloud_filtered_pcl->points[i].x;
		searchPoint.y = cloud_filtered_pcl->points[i].y;
		searchPoint.z = cloud_filtered_pcl->points[i].z;
		
     		if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  		{
  	
  	
			for (size_t j = 0; j < pointIdxNKNSearch.size (); ++j)
			{

				//obstacle_nomal_pcl.points[pointIdxNKNSearch[j]].z = cloud_normals->points[pointIdxNKNSearch[j]].normal_z;
				
				
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
	
	//cloud_outlier_removal(obstacle_pcl,obstacle_pcl);
	
	pcl::toROSMsg(*obstacle_pcl,output_cloud);
	
    	output_cloud.header.frame_id = "laser";
    	output_cloud.header.stamp = ros::Time::now();	
    	
	ROS_INFO("Nummber of points in obstacle_cloud is:");
	std::cout << obstacle_pcl->points.size() << "\n";
	
	obstcle_pub_.publish(output_cloud);
	ROS_WARN("Obstacle cloud published");
	cloud_2D_projection(obstacle_pcl,obs_projected);
	
	
	pcl::toROSMsg(*obs_projected,cloud_obstacle_projected);
    	cloud_obstacle_projected.header.frame_id = "laser";
    	cloud_obstacle_projected.header.stamp = ros::Time::now();		
	obstcle_proj_pub_.publish(cloud_obstacle_projected);
	
	compute_repulsive_force(obs_projected);
	
	cloud_to_costmap(obs_projected);
	cost_map_2_cloud();
	
	repuslive_force_pub_.publish(repulsive_force);
	repulsive_force.x = 0.0;
	repulsive_force.y = 0.0;
	repulsive_force.z = 0.0;
	//debug

	
	
	//debug end
	
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
		obstacle_find_Publish(cloud);
		
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>);
		//pcl::PointCloud<pcl::PointXYZ>::Ptr key_pcl (new pcl::PointCloud<pcl::PointXYZ>);
		//std::vector<int> mapping;
		//pcl::removeNaNFromPointCloud(*cloud_pcl, *cloud_pcl, mapping);
		//pcl::fromROSMsg (*cloud, *cloud_pcl);
		//cloud_voxel_filter(cloud_pcl,cloud_pcl,0.05);
		//ketpoint_extract(cloud_pcl);
		//ISS_keypoint(cloud_pcl,key_pcl);
	
	}
	
	void cloud_to_costmap(pcl::PointCloud<pcl::PointXYZ>::Ptr obs_2d)
	{
	
	int mx;
	int my;

    	master_grid_->resetMap(0,0,cell_x-1,cell_y-1);
	for (size_t i = 0; i < obs_2d->points.size (); ++i)
	{
	
		master_grid_->worldToMapEnforceBounds((double) obs_2d->points[i].x,(double) obs_2d->points[i].y,mx,my);
		master_grid_ros->updateBounds(0,cell_x-1,0,cell_x-1);
		master_grid_->setCost(mx,my, LETHAL_OBSTACLE);
		
	
	
	}  	
  	// Costmap2DPublisher
  	master_grid_ros->publishCostmap();
  	
	}
	
	void cloud_to_costmap(pcl::PointCloud<pcl::PointXYZ> obs_2d)
	{
	
	int mx;
	int my;

    	master_grid_->resetMap(0,0,cell_x-1,cell_y-1);
	for (size_t i = 0; i < obs_2d.points.size (); ++i)
	{
	
		master_grid_->worldToMapEnforceBounds((double) obs_2d.points[i].x,(double) obs_2d.points[i].y,mx,my);
		master_grid_ros->updateBounds(0,cell_x-1,0,cell_x-1);
		master_grid_->setCost(mx,my, LETHAL_OBSTACLE);
		
	
	
	}  	
  	// Costmap2DPublisher
  	master_grid_ros->publishCostmap();
  	
	}	
	
	void cost_map_2_cloud()
	{
	double temp_x;
	double temp_y;
	cost_map_cloud.clear();
	for (size_t i=0; i < cell_x + 1; i++)
	{
		for (size_t j=0; j < cell_y + 1; j++)
		{
			if(master_grid_->getCost(i,j) == LETHAL_OBSTACLE) 
			{
				master_grid_->mapToWorld(i,j,temp_x,temp_y);
				//ROS_INFO("sono qua");
				for (int k = -1; k < 2; k++)
				{
					for (int l = -1; l < 2; l++)
					{
						pcl::PointXYZ point;
						point.z = 0.0;
						point.x = temp_x + (float) k*costmap_res/2*0.9;
						point.y = temp_y + (float) l*costmap_res/2*0.9;	
						/*
						std::cout << "point.x" << point.x << "\n";
						std::cout << "point.x" << point.x << "\n";
						std::cout << "point.x" << point.x << "\n";*/			
						cost_map_cloud.points.push_back(point);
					}
				}
			}
		
		}
	}
	
	
	}
	
	void run()
	{
	
		double normal_threshold_default = 0.7;
		double height_threshold_default = -0.1;
		double height_max_default = 2.0;
		
		ros::NodeHandle n("~");
		
		n.param("normal_threshold", normal_threshold, normal_threshold_default);
		n.param("height_threshold", height_threshold, height_threshold_default);
		n.param("height_max", height_max,height_max_default);
		
  
		if (normal_threshold != normal_threshold_default) ROS_INFO_ONCE("normal threshold is changed to %f", normal_threshold);
		if (height_threshold != height_threshold_default) ROS_INFO_ONCE("height threshold is changed to %f", height_threshold);
		if (height_max       != height_max_default)       ROS_INFO_ONCE("height threshold is changed to %f", height_max);
		
		ros::Rate rate(10.0);
		tf::TransformListener listener;
		
		bool first_loop = true;
		bool transform_present = true; 
		float curr_x;
		float curr_y;
		float curr_yaw;
		float last_x;
		float last_y;
		float last_yaw;		
		
		

		Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
		
		while(ros::ok())
		{
			
			
		    	tf::StampedTransform transform_odom_laser;
    		    	try{
      				listener.lookupTransform("/odom", "/laser", ros::Time(0), transform_odom_laser);
      				transform_present = true;
    		    	}
    			catch (tf::TransformException ex)
    			{
      				ROS_ERROR("%s",ex.what());
      				ros::Duration(0.05).sleep();
      				transform_present = false;
    			}
			//Reading x and y
			curr_x =transform_odom_laser.getOrigin().x();
			curr_y =transform_odom_laser.getOrigin().y();
			
			tfScalar roll,pitch,yaw;
			
			tf::Matrix3x3 M(transform_odom_laser.getRotation());
			M.getRPY(roll,pitch,yaw,(unsigned int) 1);
			curr_yaw = (float) yaw;
			
			//ROS_INFO("CIAOOOOOO");
			if (!first_loop)
			{	
				
				float delta_x   = (curr_x - last_x);
				float delta_y   = (curr_y - last_y);
				float delta_yaw = (curr_yaw - last_yaw);
				
				//ROS_ERROR("delta_x = %f, delta_y = %f, delta_yaw = %f",delta_x,delta_y,delta_yaw);
				/*
				| cos(yaw)  sin(yaw) 0  x|
				|-sin(yaw)  cos(yaw) 0  y|
				|     0        0     1  0|
				|     0        0     0  1|
				*/
				
				transform_1 (0,0) =  cos (delta_yaw);
  				transform_1 (0,1) =  sin (delta_yaw);
  				transform_1 (1,0) = -sin (delta_yaw);
  				transform_1 (1,1) =  cos (delta_yaw);
  				transform_1 (0,3) = -delta_y; //odom orientation is 90 degree rotated with respect to laser
  				transform_1 (1,3) = -delta_x;
  				transform_1 (2,3) = 0.0;
  				transform_1 = transform_1;
  				pcl::transformPointCloud (cost_map_cloud, cost_map_cloud, transform_1);
  				cloud_to_costmap(cost_map_cloud);
			}
						
			last_x = curr_x;
			last_y = curr_y;
			last_yaw = curr_yaw;
			if (first_loop && transform_present) 
			{
				first_loop = false;
				ROS_INFO("start moving pc!....");	
			}
		
			// Publishing cost_map pc
			pcl::toROSMsg(cost_map_cloud,costmap_cl);
		//std::cout << "   --   >   " <<cost_map_cloud.points.size() << "\n";
    			costmap_cl.header.frame_id = "laser";
    			costmap_cl.header.stamp = ros::Time::now();		
			cost_map_cl_pub_.publish(costmap_cl);			
			
			rate.sleep();
			ros::spinOnce ();
			
			
		}
	}
	
	
	protected:
	
		// Node Handler
		ros::NodeHandle n_;
		ros::NodeHandle* n; 
		// Subscribers
		ros::Subscriber SubFromCloud_;
		
		
		// Publishers
		ros::Publisher obstcle_pub_;
		ros::Publisher obstcle_proj_pub_;
		ros::Publisher cost_map_cl_pub_;
		ros::Publisher repuslive_force_pub_;
		
		//ros::Publisher keypoint_pub_;
		//ros::Publisher ISS_keypoint_pub_;
		geometry_msgs::Vector3 repulsive_force;
		
		//Class Global Variables

		double normal_threshold;
		double height_threshold;
		double height_max;
		sensor_msgs::PointCloud2 output_cloud; 
		sensor_msgs::PointCloud2 cloud_obstacle_projected; 
		sensor_msgs::PointCloud2 costmap_cl;
		sensor_msgs::PointCloud2 key_cloud;

		//costmap variables
		costmap_2d::Costmap2D* master_grid_;
		double costmap_x_size;
		double costmap_y_size;
		double costmap_res;
		pcl::PointCloud<pcl::PointXYZ> cost_map_cloud;
		unsigned int cell_x;
		unsigned int cell_y;
				
		std::string global_frame, topic_name;
		costmap_2d::Costmap2DPublisher* master_grid_ros;
		
		
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
