#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <ros/timer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/fpfh.h>

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::FPFHSignature33 DescriptorType; //pcl::SHOT352 DescriptorType; //pcl::PFHSignature125

std::string model_filename_;
std::string scene_filename_;

//Algorithm params
bool show_keypoints_ ;
bool show_correspondences_ ;
bool use_cloud_resolution_ ;
bool use_hough_ ;
float model_ss_ ;
float scene_ss_ ;
float rf_rad_ ;
float descr_rad_ ;
float cg_size_ ;
float cg_thresh_ ;
	

class ObjectFinderClass
{
	public:
		
	ObjectFinderClass(ros::NodeHandle& node)
	{
			// Node handle
			n_=node;

			//subscribers
			SubFromCloud_		= n_.subscribe("/Right_cloud", 5, &ObjectFinderClass::cloud_handle,this);
			SubFromObject_		= n_.subscribe("/Object_pcl", 1, &ObjectFinderClass::object_handle,this);

			
			// publishers
			//cloud_pub_ 			 = n_.advertise<sensor_msgs::PointCloud2> ("cloud_in", 1);
			

    			// Initializers
    			scene_available= false;
			model_available= false;

    			show_keypoints_ = false;
			show_correspondences_  = false;
			use_cloud_resolution_  = false;
			use_hough_ = true;
			model_ss_ = 0.01f;
			scene_ss_  = 0.03f;
			rf_rad_  = 0.015f;
			descr_rad_  = 0.20f;
			cg_size_  = 0.01f;
			cg_thresh_  = 5.0f;

	
	}
	
	
	void cloud_handle(const sensor_msgs::PointCloud2ConstPtr& data)
	{
		//scene_in.clear();
		pcl::fromROSMsg (*data, scene_in);
		if (!process) process = true;
		ROS_INFO("scene size = %d",(int) scene_in.points.size());	
		if(scene_in.points.size() > 10) scene_available = true;
	}
	
	void object_handle(const sensor_msgs::PointCloud2ConstPtr& msg)
	{
		pcl::fromROSMsg (*msg, model_in);
		model_available = true;
			
	
	}
	
	double computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
	{
  		double res = 0.0;
  		int n_points = 0;
  		int nres;
  		std::vector<int> indices (2);
  		std::vector<float> sqr_distances (2);
  		pcl::search::KdTree<PointType> tree;
  		tree.setInputCloud (cloud);

  		for (size_t i = 0; i < cloud->size (); ++i)
  		{
   		 	if (! pcl_isfinite ((*cloud)[i].x))
    			{
     		 		continue;
    			}
    			//Considering the second neighbor since the first is the point itself.
    			nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
   		 	if (nres == 2)
    			{
     				res += sqrt (sqr_distances[1]);
     		 		++n_points;
    			}
  		}
  		if (n_points != 0)
  		{
    			res /= n_points;
  		}
  		
  		return res;
	}
		
	void cloud_outlier_removal(pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr cloud_filtered)
	{
		pcl::StatisticalOutlierRemoval<PointType> sor;
  		sor.setInputCloud (cloud_in);
  		sor.setMeanK (50);
  		sor.setStddevMulThresh (1.0);
  		sor.filter (*cloud_filtered);
	}
	
	void cloud_voxel_filter(pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr cloud_out,float cube_size)
	{
		
  		pcl::VoxelGrid<PointType> VG;
  		VG.setInputCloud (cloud_in);
  		VG.setLeafSize (cube_size, cube_size, cube_size);
 		VG.filter (*cloud_out);	
	
	}
	
	
  	void run()
	{
		
  		pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
 		pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
  		pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
  		pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
  		pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
  		pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
  		pcl::PointCloud<NormalType>::Ptr model_key_normals (new pcl::PointCloud<NormalType> ());
  		pcl::PointCloud<NormalType>::Ptr scene_key_normals (new pcl::PointCloud<NormalType> ()); 		
  		pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
  		pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());
  		
  		
  		ros::Rate wait_rate(3);
  		ROS_INFO("Waiting for data");
  		while(!scene_available)
  		{
  			wait_rate.sleep();
  			ros::spinOnce();
			
			//std::cout << "Waiting for the data.";
  			//std::cout << ".";
  		}
  		ROS_INFO("Process starts!....");  		
  		
  		*model = model_in;
  		*scene = scene_in;
  		
  		std::vector<int> indices;
      		pcl::removeNaNFromPointCloud(*model,*model, indices);
      		pcl::removeNaNFromPointCloud(*scene,*scene, indices);
      		
      		
  		ROS_WARN("model size = %d",(int) model->points.size());
  		
  		ROS_WARN("scene global size = %d",(int) scene_in.points.size());
  		ROS_WARN("scene size = %d",(int) scene->points.size());
  		
  		ros::Duration(2.0).sleep();
  		scene_available = false;
  		
  			
  		if (use_cloud_resolution_)
  		{
    			float resolution = static_cast<float> (computeCloudResolution (model));
    			if (resolution != 0.0f)
    			{
      			model_ss_   *= resolution;
      			scene_ss_   *= resolution;
      			rf_rad_     *= resolution;
      			descr_rad_  *= resolution;
      			cg_size_    *= resolution;
    			}
		/*
    		std::cout << "Model resolution:       " << resolution << std::endl;
    		std::cout << "Model sampling size:    " << model_ss_ << std::endl;
    		std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
    		std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
    		std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
    		std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
    		*/
  		}
  		
  		ros::Rate loop_rate(2);
  		
  	
  		//std::cout << "Waiting for the data.";

  			
  		while(ros::ok())
  		{
  	
  		//if(process){
  		
  		
  		//
  		//  Downsample Clouds to Extract keypoints
  		//
	
		cloud_voxel_filter(model, model_keypoints, 0.1);
		cloud_voxel_filter(scene, scene_keypoints, 0.1);
  		
  		//
  		//  Compute Normals
  		//
  		pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  		norm_est.setKSearch (10);
  	
  		norm_est.setInputCloud (model);
  		norm_est.compute (*model_normals);

  		norm_est.setInputCloud (scene);
  		norm_est.compute (*scene_normals);
 
   		pcl::NormalEstimationOMP<PointType, NormalType> norm_est2;
  		norm_est2.setKSearch (10);
  		
  		norm_est2.setInputCloud (model_keypoints);
  		norm_est2.compute (*model_key_normals);
  		
  		norm_est2.setInputCloud (scene_keypoints);
  		norm_est2.compute (*model_key_normals);
  		
		/*
  		pcl::UniformSampling<PointType> uniform_sampling;
  		uniform_sampling.setInputCloud (model);
 		uniform_sampling.setRadiusSearch (model_ss_);
  		uniform_sampling.detectKeypoints(*model_keypoints);
  		std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

  		uniform_sampling.setInputCloud (scene);
  		uniform_sampling.setRadiusSearch (scene_ss_);
  		uniform_sampling.detectKeypoints(*scene_keypoints);
  		std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;
		*/
		
		
	//ROS_WARN("model: %d, model_keypoint: %d, model_normals:%d", (int) model->points.size(), (int) model_keypoints->points.size(), (int) model_normals->points.size());
	//ROS_WARN("scene: %d, scene_keypoint: %d, scene_normals:%d", (int) scene->points.size(), (int) scene_keypoints->points.size(), (int) scene_normals->points.size());
	//ros::Duration(10.0).sleep();
  		//
  		//  Compute Descriptor for keypoints
 		//
 	ROS_WARN("model descriptor start");
 		/*
  		pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  		descr_est.setRadiusSearch (descr_rad_);
		*/
		
		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> descr_est;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr pfhe_tree (new pcl::search::KdTree<pcl::PointXYZ> ());
		
		descr_est.setRadiusSearch (descr_rad_);
  		descr_est.setInputCloud (model_keypoints);//model_keypoints
  		descr_est.setInputNormals (model_normals);
  		//descr_est.setSearchMethod (pfhe_tree);
  		descr_est.setSearchSurface (model);
  		
  		
  		descr_est.compute (*model_descriptors);
  		
  		
  		

  		
  		
	//ros::Duration(10.0).sleep();
	ROS_WARN("scene descriptor start");
	
  		descr_est.setInputCloud (scene_keypoints);//scene_keypoints
  		descr_est.setInputNormals (scene_normals);
  		descr_est.setSearchSurface (scene);
  		descr_est.compute (*scene_descriptors);
	
  		//
  		//  Find Model-Scene Correspondences with KdTree
  		//
  		pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

  		pcl::KdTreeFLANN<DescriptorType> match_search;
  		match_search.setInputCloud (model_descriptors);
  		
  	std::cout << "model descriptors size " << model_descriptors->size () << std::endl;	
  	std::cout << "scene descriptors size " << scene_descriptors->size () << std::endl;
  		
  		//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  		for (size_t i = 0; i < scene_descriptors->size (); ++i)
  		{
    			std::vector<int> neigh_indices (1);
    			std::vector<float> neigh_sqr_dists (1);
    		/*	
    			if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
    			{
      				continue;
    			}
    		*/
    			int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    			
    			//  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    			if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) 
    			{
      				pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      				model_scene_corrs->push_back (corr);
    			}
  		}
  		std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;
  		//
 		//  Actual Clustering
  		//
  		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  		std::vector<pcl::Correspondences> clustered_corrs;
  		
  		
  	
  		//  Using Hough3D
  		if (use_hough_)
  		{
    		//
    		//  Compute (Keypoints) Reference Frames only for Hough
    		//
    			pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    			pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    			pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    			rf_est.setFindHoles (true);
    			rf_est.setRadiusSearch (rf_rad_);

    			rf_est.setInputCloud (model_keypoints);
    			rf_est.setInputNormals (model_normals);
    			rf_est.setSearchSurface (model);
    			rf_est.compute (*model_rf);

    			rf_est.setInputCloud (scene_keypoints);
    			rf_est.setInputNormals (scene_normals);
    			rf_est.setSearchSurface (scene);
    			rf_est.compute (*scene_rf);

    			//  Clustering
    			pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    			clusterer.setHoughBinSize (cg_size_);
    			clusterer.setHoughThreshold (cg_thresh_);
    			clusterer.setUseInterpolation (true);
    			clusterer.setUseDistanceWeight (false);

    			clusterer.setInputCloud (model_keypoints);
    			clusterer.setInputRf (model_rf);
    			clusterer.setSceneCloud (scene_keypoints);
   			clusterer.setSceneRf (scene_rf);
   			clusterer.setModelSceneCorrespondences (model_scene_corrs);

    			//clusterer.cluster (clustered_corrs);
    			clusterer.recognize (rototranslations, clustered_corrs);
  		}
  		else // Using GeometricConsistency
  		{
    			pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    			gc_clusterer.setGCSize (cg_size_);
    			gc_clusterer.setGCThreshold (cg_thresh_);

    			gc_clusterer.setInputCloud (model_keypoints);
    			gc_clusterer.setSceneCloud (scene_keypoints);
    			gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    			//gc_clusterer.cluster (clustered_corrs);
    			gc_clusterer.recognize (rototranslations, clustered_corrs);
  		}
  		
  			
  		//
  		//  Output results
  		//
  		std::cout << "Model instances found: " << rototranslations.size () << std::endl;
  		for (size_t i = 0; i < rototranslations.size (); ++i)
  		{
    			std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    			std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

    			// Print the rotation matrix and translation vector
    			Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    			Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

    			printf ("\n");
    			printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    			printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    			printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    			printf ("\n");
    			printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  		}
  		
       	
  		
  		process = false;
  		//}//process
  	
		ros::spinOnce();
		loop_rate.sleep();
		}
	
	}
  	

	protected:
	
	// Node Handler
	ros::NodeHandle n_;
		
	// Subscribers
	ros::Subscriber SubFromCloud_;
	ros::Subscriber SubFromObject_;
	
	// Publishers
	

		
	//Class Global Variables
	pcl::PointCloud<PointType> model_in;
	pcl::PointCloud<PointType> scene_in;
	bool process;
	bool scene_available;
	bool model_available;

	
	
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_finder");
	ros::NodeHandle node;

	ObjectFinderClass of(node);
	
	of.run();
	
	return 0;
}

