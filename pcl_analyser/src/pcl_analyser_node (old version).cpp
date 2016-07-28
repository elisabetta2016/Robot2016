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
#include "donkey_rover/Rover_Track_Speed.h"
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Joy.h>
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
#include <list>
// TF
#include <tf/transform_listener.h>
#include <Eigen/Dense> 

#define INFLATED_OBSTACLE 200

using namespace Eigen;
typedef pcl::PointXYZ PointType;
// Range image coordinate frame 
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
using costmap_2d::LETHAL_OBSTACLE;


struct PATH_COST{
	float Lethal_cost;
	float Travel_cost;
	float Inf_cost;
	bool collision;	
};
struct CELL{
	unsigned int x;
	unsigned int y;
	unsigned char c;
};

float Travel_cost_inc = 0.1;
float Lethal_cost_inc = 10.0;
float Inf_cost_inc = 1.0;
double b = 0.8;
int sample = 15;

bool pso_analyse = false;

class ObstacleDetectorClass
{
	public:
		
	ObstacleDetectorClass(ros::NodeHandle& node)
	{
			n_=node;

			//subscribers
			SubFromCloud_		 = n_.subscribe("/RL_cloud", 1, &ObstacleDetectorClass::cloud_call_back,this);
			subFromJoystick_ 	 = n_.subscribe("joy", 1, &ObstacleDetectorClass::joyCallback,this);
			subFromTrackSpeed_	 = n_.subscribe("/RoverTrackSpeedZZZZzzz", 1, &ObstacleDetectorClass::TrackCallback,this);
			subFromGoal_		 = n_.subscribe("/goal", 1, &ObstacleDetectorClass::GoalCallback,this);
			
			// publishers
			obstcle_pub_		  = n_.advertise<sensor_msgs::PointCloud2> ("obstacle_cloud", 1);
			obstcle_proj_pub_	  = n_.advertise<sensor_msgs::PointCloud2> ("obstacle_proj_cloud", 1);
			cost_map_cl_pub_	  = n_.advertise<sensor_msgs::PointCloud2> ("costmap_cloud", 1);
			path_trace_pub_        = n_.advertise<sensor_msgs::PointCloud2> ("path_trace", 1);
			
			repuslive_force_pub_	  = n_.advertise<geometry_msgs::Vector3> ("force", 1);
			path_pub_	  	  = n_.advertise<nav_msgs::Path> ("Path_sim", 1);
			path_solution_pub_        = n_.advertise<nav_msgs::Path> ("Path_pso", 1);
			

    			
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
			
			cell_x = (unsigned int) floor(abs(costmap_x_size/costmap_res)); //cell
			cell_y = (unsigned int) floor(abs(costmap_y_size/costmap_res)); //cell
			
			master_grid_ = new costmap_2d::Costmap2D(cell_x,cell_y,costmap_res,-1.0,-3.0,0);
			n = &n_;
    			
    			global_frame = "laser";
			topic_name = "/global_costmap";
			
			master_grid_ros = new costmap_2d::Costmap2DPublisher(n,master_grid_,global_frame,topic_name,false);
			
			//obstacle avoidance params
			goal_present = false;
			
			
    			
	}

	void fill_costmap_test()
	{
		
		pcl::PointCloud<pcl::PointXYZ> fake_obs;

		float X_obs = 1.6;
		float Y_obs = -0.5;
		
		pcl::PointXYZ point;
		point.x = X_obs;
		point.y = Y_obs;
		point.z = 0.0;
		for(int i=0;i<3; i++)
		{
			point.x = point.x + costmap_res;
			point.y = Y_obs;
			for(int j=0;j < 3;j++)
			{	
				//ROS_INFO("j = %d", j);
				
				
				point.y = point.y - costmap_res;
				point.z = 0.0;
				fake_obs.points.push_back(point);
				//master_grid_->setCost(i,j, LETHAL_OBSTACLE);
				//master_grid_ros->updateBounds(0,cell_x-1,0,cell_x-1);
			}
		}
		cloud_to_costmap(fake_obs);
		cost_map_2_cloud();
	
	
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
  	
  	void compute_repulsive_force(pcl::PointCloud<pcl::PointXYZ>::Ptr obs_proj)
  	{
  	
  	for (size_t i = 0; i < obs_proj->points.size (); ++i)
		{
			repulsive_force.x +=  1/obs_proj->points[i].x;
			repulsive_force.y +=  1/obs_proj->points[i].y;
			
		}

  	}

	
	void obstacle_find_Publish(const sensor_msgs::PointCloud2ConstPtr& cloud)
	{
	//ROS_INFO("Obstacle detection Starts!");
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_pcl (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::fromROSMsg(*cloud, *cloud_pcl);

 	*cloud_filtered_pcl = *cloud_pcl;

  
  
  
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
    	
	//ROS_INFO("Nummber of points in obstacle_cloud is:");
	//std::cout << obstacle_pcl->points.size() << "\n";
	
	obstcle_pub_.publish(output_cloud);
	//ROS_WARN("Obstacle cloud published");
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
	
	lethal_inflation();
  	// Costmap2DPublisher
  	
  	master_grid_ros->publishCostmap();
  	
	}
	
	
	void lethal_inflation() //fill the cell around the lethal obstacle
	{
	    
		std::list<int> list_x_lethal; //create two list
		std::list<int> list_y_lethal;
		float lethal_rad = 0.1;
		
	  	
		for (unsigned int i=0; i < cell_x ; i++) //loop in x
		{
			for (unsigned int j=0; j < cell_y ; j++) //loop in y
			{
				if(master_grid_->getCost(i,j) == LETHAL_OBSTACLE) //find the lethal obstacle
				{
					list_x_lethal.push_back(i); //fill the list with lethal obstacles
					list_y_lethal.push_back(j);
				}
			}
		}
		
		//loop in the list of lethal obstacles
		for (int k=0; k < list_x_lethal.size(); k++) //or list_y_lethal.size(), it's the same
		{
			int cell_around = (int) floor(fabs(lethal_rad/costmap_res))+1;
			int i = list_x_lethal.front(); //take the first element of the list
			int j = list_y_lethal.front();
			
			for (int ii=-cell_around; ii<cell_around+1; ii++) //loop around the obstacle
			{
				for (int jj=-cell_around; jj<cell_around+1; jj++) 
				{
				
					if ((i+ii) < 0) ii = std::min(-i,ii);
			        	if ((j+jj) < 0) jj = std::min(-j,jj);
					try
					
					{ 
			
					master_grid_->setCost(i+ii, j+jj, LETHAL_OBSTACLE); //fill the cell around the obstacle
					inf_inflation(i+ii,j+jj, master_grid_);
					
					}
			
					//continue even if exit from the grid
			
					catch(int e)	
		
					{
					//do nothing
					}
				}
			}
			
			list_x_lethal.pop_front(); //eliminate the first element of the list 
			list_y_lethal.pop_front();
		
		}			
	}
	
	void inf_inflation(int i,int j, costmap_2d::Costmap2D* grid)
	{
		
		float inf_rad = 0.3;
		int cell_around = (int) floor(fabs(inf_rad/costmap_res))+1;
		unsigned char INFLATION_OBSTACLE = 200;
		for (int k=-cell_around; k<cell_around+1; k++) //loop around the lethal obstacle
			{
			
				for (int l=-cell_around; l<cell_around+1; l++) 
				{
				
				if ((i+k) < 0) k = std::min(-i,k);
				if ((i+l) < 0) l = std::min(-j,l);
				unsigned char cost = grid->getCost((unsigned int)k+i,(unsigned int)l+j);
				//std::cout << "the cost is " << cost << std::endl;
				
					if(cost != LETHAL_OBSTACLE) //if is a lethal do nothing
					{
						try {
						     grid->setCost((unsigned int)i+k, (unsigned int)j+l, INFLATION_OBSTACLE); //fill the cell around lethal_obs
						}
						catch(int e)
						{
						//do nothing
						}
					}
				}
			}
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

				for (int k = 0; k < 1; k++) // -1;2
				{
					for (int l = 0; l < 1; l++) // -1;2
					{
						pcl::PointXYZ point;
						point.z = 0.0;
						point.x = temp_x + (float) k*costmap_res/2*0.9;
						point.y = temp_y + (float) l*costmap_res/2*0.9;			
						cost_map_cloud.points.push_back(point);
					}
				}
			}
		
		}
	   }
	
	}
	
	void joyCallback(const sensor_msgs::JoyConstPtr& joy)
	{
		// inputs
  		float V_in = joy->axes[1];
  		float Omega_in = joy->axes[2];
  		VectorXf V_input;
  		VectorXf Omega_input;
  		
  		V_input.setOnes(sample);
  		Omega_input.setOnes(sample);
  		V_input = V_in * V_input;
  		Omega_input = Omega_in * Omega_input;
  		double Ts = 3.0;
  		Vector3f x_0;
  		x_0 << 0.0, 0.0, 0.0;
  		Vector3f x_dot_0;
  		x_dot_0 << 0.0, 0.0, 0.0;
  		
  		//outputs
  		Vector3f x_dot_f;
  		MatrixXf x;
  		x.setZero(3,sample);
  		
  		//invoke
  		x = Rover_vw(V_input, Omega_input, b, Ts,x_0,x_dot_0 , sample, x_dot_f);
  	//ROS_INFO("trajectory length:%d   x_mid:%f", x.cols(), x(0,sample-8));
  	
  	//std::cout << x.cols() << "\n";
  		PATH_COST cost = Cost_of_path(x, master_grid_);
  		//ROS_WARN("lethal cost:%f, collision:%d", cost.Lethal_cost,cost.collision);
  		
  		if (cost.collision)
  		{
  		// some logic to be implemented and the best path to be find by the method
  			if (!goal_present)
  			{
  				nav_goal(0) = x(0,sample-2);
  				nav_goal(1) = x(1,sample-2);
  				nav_goal(2) = 0.0;
  			}
  			float D = 2.0;
  			size_t particle_no = 5;
  			size_t iteration = 3;
  			Matrix3f output;
  			MatrixXf output_tra;
  			bool solution_found;
  			if (!pso_analyse)
  			{
  				//PSO_path_finder(nav_goal, D, V_in, particle_no, iteration, output, output_tra, solution_found);
				pso_analyse = true;  			
  			}
  			
  		}		
	  	
  	}
  	
  	

  	
  	  	
	void GoalCallback(const geometry_msgs::Vector3::ConstPtr& msg)
	{
		nav_goal(0,0) = msg->x;
		nav_goal(0,1) = msg->y;
		nav_goal(0,2) = 0.0; 
		goal_present = true; 
	}
	void TrackCallback(const donkey_rover::Rover_Track_Speed::ConstPtr& msg)
	{
		float V_in = (msg->Front_Right_Track_Speed + msg->Front_Right_Track_Speed)/2;
		float Omega_in = (msg->Front_Right_Track_Speed - msg->Front_Right_Track_Speed)/0.8; //to be checked
  		VectorXf V_input;
  		VectorXf Omega_input;
  		//int sample = 15;
  		V_input.setOnes(sample);
  		Omega_input.setOnes(sample);
  		V_input = V_in * V_input;
  		Omega_input = Omega_in * Omega_input;
  		double Ts = 3.0;
  		Vector3f x_0;
  		x_0 << 0.0, 0.0, 0.0;
  		Vector3f x_dot_0;
  		x_dot_0 << 0.0, 0.0, 0.0;
  		
  		//outputs
  		Vector3f x_dot_f;
  		MatrixXf x;
  		//invoke
  		//Rover_vw(V_input, Omega_input, b, Ts,x_0,x_dot_0 , sample, x, x_dot_f);
  		PATH_COST cost = Cost_of_path(x, master_grid_);
  		if (cost.collision)
  		{
  		// some logic to be implemented and the best path to be find by the method
  		
  		}		
	
	}
	  	
  		
	PATH_COST Cost_of_path(MatrixXf path, costmap_2d::Costmap2D* grid)
	{
	CELL prev_cell;
	CELL curr_cell;
	prev_cell.x = 0;
	prev_cell.y = 0;
	PATH_COST cost;
	cost.Lethal_cost = 0.0;
	cost.Travel_cost = Travel_cost_inc;
	cost.Inf_cost = 0.0;
	cost.collision = false;
	
	for(size_t i=0; i < path.cols(); i++)
	{
		grid->worldToMap((double) path(0,i),(double) path(1,i),curr_cell.x,curr_cell.y);
		//debug
	     //ROS_WARN("cell x:%d cell y:%d",curr_cell.x,curr_cell.y);
		
		//debug end
		if( (curr_cell.x != prev_cell.x) && (curr_cell.x != prev_cell.x) )
		{
			curr_cell.c = grid->getCost(curr_cell.x,curr_cell.y);
			if (curr_cell.c == LETHAL_OBSTACLE)
			{
				cost.Lethal_cost += Lethal_cost_inc;
				cost.collision = true;
			}
			if (curr_cell.c == INFLATED_OBSTACLE)
			{
				cost.Inf_cost += Inf_cost_inc; 
			}
			cost.Travel_cost +=  Travel_cost_inc;
			prev_cell = curr_cell;
		}
	}
	return cost;
	}
	
	
	MatrixXf Rover_vw(VectorXf V_input, VectorXf Omega_input, double b, double Ts,Vector3f x_0,Vector3f x_dot_0 , int sample, Vector3f x_dot_f)
	{

	MatrixXf x;
	x.setZero(3,sample);
    	MatrixXf x_dot;
    	//MatrixXd V_in;
    	//MatrixXd Omega_in;
    	MatrixXf NE_dot_temp;
    	MatrixXf Rot_temp;
    	MatrixXf V_temp;
        
    	double dt = Ts / ((double)sample); 
    	//x.setZero(3,sample);
    	x_dot.setZero(3,sample);
    	//V_in.setOnes(1,sample);
    	//Omega_in.setOnes(1,sample);
   	x.col(0) = x_0;
    	NE_dot_temp.setZero(2,sample);
   	NE_dot_temp.col(0) = x_dot_0.topRows(2);
    
    	Rot_temp.setIdentity(2,2);
    	V_temp.setZero(2,1);
    
    	for(size_t i=1; i < sample; i++)
    	  {
     		x(2,i) = x(2,i-1) + Omega_input(i);
     
     		Rot_temp(0,0) =    cos(x(2,i));
     		Rot_temp(0,1) = -b*sin(x(2,i));
     		Rot_temp(1,0) =    sin(x(2,i));
     		Rot_temp(1,1) =  b*cos(x(2,i));
     
     		V_temp(0,0)  = V_input(i);
     		V_temp(0,1)  = Omega_input(i);
     		NE_dot_temp.col(i) = Rot_temp * V_temp;
     		x_dot(0,i) = NE_dot_temp(0,i);
     		x_dot(1,i) = NE_dot_temp(1,i);
     		x_dot(2,i) = Omega_input(i);
     
     		x(0,i) = x(0,i-1)+x_dot(0,i)*dt;
     		x(1,i) = x(1,i-1)+x_dot(1,i)*dt;
        
    	   }
    	   
    	x_dot_f = x_dot.rightCols(sample-1);   
    	
    	
    	   
	nav_msgs::Path robot_path;
	robot_path.header.stamp = ros::Time::now();
	robot_path.header.frame_id = "laser";
	robot_path.poses = std::vector<geometry_msgs::PoseStamped> (sample);
	
	for(size_t i=0; i < sample; i++)
	  {
		robot_path.poses[i].pose.position.x = x(0,i);
		robot_path.poses[i].pose.position.y = x(1,i);
		robot_path.poses[i].pose.position.z = 0.0;
		//robot_path.poses[i].orientation = tf::createQuaternionMsgFromYaw(x(2,i));
	  }
	  
	path_pub_.publish(robot_path);
	return x;  
	}
	
	void traj_to_cloud(MatrixXf tra)
	{
		
		for(size_t i = 0; i < tra.cols(); i++)
		{
			pcl::PointXYZ point;
			point.x = tra(0,i);
			point.y = tra(1,i);
			point.z = 0.0;
			path_trace_pcl.points.push_back(point);
		}
		
	}
	
	void testCallBack()
	{
		// inputs
		ROS_INFO_ONCE("Test Start");
		
  		float V_in = 1.0;
  		float Omega_in = -0.05;
  		VectorXf V_input;
  		VectorXf Omega_input;
  		Vector2f V_curr_c;
  		V_curr_c(0) = V_in;
  		V_curr_c(1) = Omega_in;
  		
  		V_input.setOnes(sample);
  		Omega_input.setOnes(sample);
  		V_input = V_in * V_input;
  		Omega_input = Omega_in * Omega_input;
  		//ROS_WARN_STREAM_ONCE("Lin Speed tra " << V_input);
  		/*
  		std::cout << "Lin Speed tra" << "\n";
  		std::cout << V_input << "\n";
  		std::cout << "W Speed tra" << "\n";
  		std::cout << Omega_input << "\n";
  		*/
  		
  		double Ts = 3.0;
  		Vector3f x_0;
  		x_0 << 0.0, 0.0, 0.0;
  		Vector3f x_dot_0;
  		x_dot_0 << 0.0, 0.0, 0.0;
  		
  		//outputs
  		Vector3f x_dot_f;
  		MatrixXf x;
  		x.setZero(3,sample);
  		
  		//invoke
  		x = Rover_vw(V_input, Omega_input, b, Ts,x_0,x_dot_0 , sample, x_dot_f);
  		ROS_WARN_STREAM_ONCE("trajectory is " << x);
  	//ROS_INFO("trajectory length:%d   x_mid:%f", x.cols(), x(0,sample-8));
  	
  	//std::cout << x.cols() << "\n";
  		PATH_COST cost = Cost_of_path(x, master_grid_);
  		ROS_WARN_ONCE("lethal cost = %f",cost.Lethal_cost);
  		//ROS_WARN("lethal cost:%f, collision:%d", cost.Lethal_cost,cost.collision);
  		
  		if (cost.collision || (cost.Lethal_cost > 0.0))
  		{
  		// some logic to be implemented and the best path to be find by the method
  			if (!goal_present)
  			{
  				//ROS_INFO("trajectory");
  				//std::cout << x << std::endl;
  				nav_goal(0) = x(0,x.cols()-1);
  				nav_goal(1) = x(1,x.cols()-1);
  				nav_goal(2) = 0.0;
  			}
  			float D = 2.0;
  			size_t particle_no = 10;
  			size_t iteration = 5;
  			Matrix3f output;
  			MatrixXf output_tra;
  			bool solution_found;
  			if (!pso_analyse)
  			{
  				PSO_path_finder(nav_goal, D, V_curr_c, particle_no, iteration, output, output_tra, solution_found);
				pso_analyse = true;  			
  			}
  			
  		}		
	  	
  	}
  		
	
	void PSO_path_finder(Vector3f Goal,float D,Vector2f V_curr_c,size_t particle_no,size_t iteration,Matrix3f output, MatrixXf output_tra, bool solution_found)
	{
	ROS_INFO("PSO Starts!... GOAL:");
	std::cout << Goal << std::endl;
	
	//Definition
	Matrix3f x;  //patricle
	Matrix3f x_best;
	Matrix3f G;
	Matrix3f v;  //particle_inc
   	
	float G_cost = 1.0/0.0;
	float x_best_cost = 1.0/0.0;
	
	//PSO params
	float pso_inertia = 0.4;
	float c_1 = 0.3;
	float c_2 = 0.3;
	
	//Objective function params
	float Goal_gain = 0.6;
	float Cost_gain = 1.0;
	float Speed_gain = 0.4;
	
	bool Once_ = false; //debug
	
   
        /* Particle structure
             | V1 V2 V3 |
             | w1 w2 w3 |
             | D1 D2 D3 |
        */	
	// initializatio1n
	x << V_curr_c,  V_curr_c,  V_curr_c,
	       D/3   ,    D/3   ,    D/3   ;
	       
	ROS_INFO("Initial particle");
	std::cout << x << "\n";
	
	v <<  -0.00, -0.00, -0.00,
	       0.05, -0.00,  0.00,
	       0.00,  0.00,  0.00;
	        
	solution_found = false;
	G = x;
	x_best = x;
  		
  	double Ts;
  	Vector3f x_0;
  	x_0 << 0.0, 0.0, 0.0;
  	Vector3f x_dot_0;
  	x_dot_0 << 0.0, 0.0, 0.0;
  	
  	//outputs
  	Vector3f x_dot_f;
  	VectorXf V_in;
  	VectorXf Omega_in;
  	MatrixXf tra;
  	
  	V_in.setOnes(sample);
  	Omega_in.setOnes(sample);
  	
    	
	for (size_t k = 0; k < iteration; k++)
	{
		
    		
    		MatrixXf tra;
		tra.setZero(3,sample);
		for(size_t i=0; i < particle_no; i++)
		{
			float r_1  = ((float) (rand() % 200))/100 -1.0;
			float r_2  = ((float) (rand() % 200))/100 -1.0;
		ROS_INFO_STREAM("Particle:" << "\n" << x);	
		ROS_INFO("r_1:%f, r_2:%f", r_1,r_2);	
		
			//first part of trajectory: tra_0
			//Ts = (fabs(x(0,0))+fabs(x(0,1))+fabs(x(0,2)))/3/D;
			Ts = 3.0;
			
			int sub_sample = sample/1; /// /3
		
			for(size_t jj=0;jj < 1; jj++)  // This is for test it should be jj<3
			{
				for(size_t j= 0 + sub_sample*jj; j < sub_sample*(jj+1); j++)
				{
					V_in(j) = x(0,jj);
					Omega_in(j) = x(1,jj);
				}
			}
			/*
			//DEBUG -show
			if(!Once_){
				std::cout << "V_in" << "\n";
				std::cout <<  V_in  << "\n";
				std::cout << "Omega_in" << "\n";
				std::cout <<  Omega_in  << "\n";			
				Once_ = true;
			}
			//DEBUG - end*/
		 	
			//simulating the trajectory
			tra = Rover_vw(V_in, Omega_in, b, Ts,x_0,x_dot_0 , sample,x_dot_f);
			traj_to_cloud(tra);
			
			ROS_ERROR_STREAM("tra size  " << tra.cols() <<"  path trace size   "<< path_trace_pcl.points.size());
			
		ROS_INFO_STREAM_ONCE("trajectory is " << tra);
		
			
			Vector3f tra_tail = tra.rightCols(tra.cols()-1);
		ROS_WARN_ONCE("tra length: %d, tra_tail length: %d", tra.cols(), tra_tail.cols());
			tra_tail(2) = 0;
			
			//Calculating the cost of trajectory
			PATH_COST cost = Cost_of_path(tra, master_grid_);
			
		ROS_INFO_ONCE("cost of the path is %f",cost.Lethal_cost);
			
			float prop_speed = (fabs(x(0,0))+fabs(x(0,1))+fabs(x(0,2)))/3;
			
			//Defining the objective function
			float Ob_func = Goal_gain * sqrtf( pow((tra_tail(0)-Goal(0)), 2) + pow((tra_tail(1)-Goal(1)), 2) )    //effect of distance from the goal
				      + Cost_gain * (cost.Lethal_cost + cost.Inf_cost)				     	      //path cost
				      + Speed_gain * (V_curr_c(0) - prop_speed);			      		      //speed effect
		
		ROS_INFO("LETHAL COST: %f",cost.Lethal_cost);		      
		ROS_INFO("Ob_fun: %f",Ob_func);
				      
			if (Ob_func < x_best_cost)
			{
				x_best_cost = Ob_func;
				x_best = x;
				ROS_INFO("new value for x_best_cost");	
			}
			if (Ob_func < G_cost)
			{
				G_cost = Ob_func;
				G = x;
				output_tra = tra;
				if (!cost.collision) solution_found = true;
				ROS_INFO("new value for G_cost");
			}
			if(i==0) //Reseting X_best and its cost in each iteration
			{
				x_best = x;
				x_best_cost = Ob_func;
			}	  
			v = pso_inertia * v + c_1 * r_1 * (x_best - x) + c_2 * r_2 * (G - x);
			
			
		//ROS_WARN("Speed terms");
		//std::cout << v << "\n";	
		/*	
		ROS_WARN("Speed terms");
		ROS_INFO("1st:");
		std::cout << pso_inertia *v << "\n";
		
		ROS_INFO("2nd:");
		std::cout << c_1 * r_1 * (x_best - x) << "\n";
		
		ROS_INFO("3rd:");
		std::cout << c_2 * r_2 * (G - x) << "\n";
		
		ROS_ERROR("v:      x_best - x:  G - x");
		std::cout << v << "\n";
		ROS_ERROR("////////");
		std::cout << (x_best - x) << "\n";
		ROS_ERROR("////////");
		std::cout << (G - x) << "\n";
		*/	
			x = x+v;	 
		

	



		}
		// Publishing
			
		robot_opt_path.header.stamp = ros::Time::now();
			
 		robot_opt_path.header.frame_id = "laser";
   
   		
		robot_opt_path.poses = std::vector<geometry_msgs::PoseStamped> (sample);
		for(size_t i=0; i < sample; i++)
		{

			robot_opt_path.poses[i].pose.position.x = tra(0,i);
			robot_opt_path.poses[i].pose.position.y = tra(1,i);
			robot_opt_path.poses[i].pose.position.z = 0.0;
			//robot_path.poses[i].orientation = tf::createQuaternionMsgFromYaw(x(2,i));
		}
	  	
	  	path_solution_pub_.publish(robot_opt_path);
	  	//ROS_WARN("V0:%f   V1:%f   V2:%f", G(0,0), G(0,1), G(0,2));
		//ROS_WARN("w0:%f   w1:%f   w2:%f", G(1,0), G(1,1), G(1,2));
		
		// end pub	
	}
    
	output = G;
    
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
		
		

		Matrix4f transform_1 = Matrix4f::Identity();
		int count = 0;
		fill_costmap_test();
		
		sensor_msgs::PointCloud2 path_trace;
		
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
			
			
			if (!first_loop)
			{	
				
				float delta_x   = (curr_x - last_x);
				float delta_y   = (curr_y - last_y);
				float delta_yaw = (curr_yaw - last_yaw);
				

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
				//ROS_INFO("start moving pc!....");	
			}
		
			// Publishing cost_map pc
			pcl::toROSMsg(cost_map_cloud,costmap_cl);
    			costmap_cl.header.frame_id = "laser";
    			costmap_cl.header.stamp = ros::Time::now();		
			cost_map_cl_pub_.publish(costmap_cl);			
			
			if (count < 40)
			   	count++;
			else
				testCallBack();
				
			//Publish 
			pcl::toROSMsg(path_trace_pcl,path_trace);
    			path_trace.header.frame_id = "laser";
    			path_trace.header.stamp = ros::Time::now();
    			path_trace_pub_.publish(path_trace);
    			
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
		ros::Subscriber subFromJoystick_;
		ros::Subscriber subFromTrackSpeed_;
		ros::Subscriber subFromGoal_;
		// Publishers
		ros::Publisher obstcle_pub_;
		ros::Publisher obstcle_proj_pub_;
		ros::Publisher cost_map_cl_pub_;
		ros::Publisher repuslive_force_pub_;
		ros::Publisher path_pub_;
		ros::Publisher path_solution_pub_;
		ros::Publisher path_trace_pub_;
		
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

		//Obstacle avoidance variables
		Vector3f nav_goal;
		bool goal_present;
		nav_msgs::Path robot_opt_path;
		
		//Path finder
		pcl::PointCloud<pcl::PointXYZ> path_trace_pcl;
		
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcl_analyser");
	ros::NodeHandle node;

	ObstacleDetectorClass Obstacle_rec(node);
	
	Obstacle_rec.run();
	
	return 0;
}
