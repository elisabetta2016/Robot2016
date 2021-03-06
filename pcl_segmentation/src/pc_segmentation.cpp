#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>sudo
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <ros/ros.h>
#include <pcl/features/don.h>
#include <iostream>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

using namespace pcl;
using namespace std;

  pcl::search::Search<PointXYZ>::Ptr tree_for_normal;
  pcl::PointCloud<PointXYZ>::Ptr input_pc (new pcl::PointCloud<PointXYZ>);
  pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);
  pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);
  pcl::DifferenceOfNormalsEstimation<PointXYZ, PointNormal, PointNormal> differenceOfNormals;
  PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
  pcl::ConditionOr<PointNormal>::Ptr range_cond (new pcl::ConditionOr<PointNormal> ());
  pcl::NormalEstimationOMP<PointXYZ, PointNormal> normalEstimation;
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointNormal> euclideanClusterExctraction;
  pcl::search::KdTree<PointNormal>::Ptr segm_tree (new pcl::search::KdTree<PointNormal>);
  pcl::PCDWriter writer;
  
//piccolo raggio
  double scale1;

//grande raggio
  double scale2;

//soglia magnitudine
  double threshold;

//soglia per il filtro
  double segradius;

void setting_Kdtree_for_normal(){
if (input_pc->isOrganized ()) //se il pointcloud ha un dataset oganizzato
  {
    tree_for_normal.reset (new pcl::search::OrganizedNeighbor<PointXYZ> ());
  }
  else
  {
    tree_for_normal.reset (new pcl::search::KdTree<PointXYZ> (false));
  }

  // Set the input pointcloud for the search tree_for_normal
  tree_for_normal->setInputCloud (input_pc);

}
void calculatingDifference(){
 
  // Create output cloud for DoN results

  copyPointCloud<PointXYZ, PointNormal>(*input_pc, *doncloud);

  cout << "Calculating DoN... " << endl;
  differenceOfNormals.setInputCloud (input_pc);
  differenceOfNormals.setNormalScaleLarge (normals_large_scale);
  differenceOfNormals.setNormalScaleSmall (normals_small_scale);

  if (!differenceOfNormals.initCompute ()) //testa se i parametri passati sono corretti 
  {
    std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
    exit (EXIT_FAILURE);
  }

  // Compute DoN
  differenceOfNormals.computeFeature (*doncloud);//effettuo la differenza e salvo il risultato in doncloud
}
void calculating_small_and_big_normals(){

  normalEstimation.setInputCloud (input_pc);   // Create the normal estimation class, and pass the input dataset to it
  normalEstimation.setSearchMethod (tree_for_normal); 

  normalEstimation.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
  //numeric_Limits è inteso come limite massimo di rappresentazionormalEstimation per valori int o float.

  // calculate normals with the small scale
  cout << "Calculating normals for scale..." << scale1 << endl;
   //point cloud con valori x y z e il vettore normale

  normalEstimation.setRadiusSearch (scale1);
  normalEstimation.compute (*normals_small_scale);

  // calculate normals with the large scale
  cout << "Calculating normals for scale..." << scale2 << endl;

  normalEstimation.setRadiusSearch (scale2);
  normalEstimation.compute (*normals_large_scale);

  calculatingDifference();
}

void filter(){

  cout << "Filtering out DoN mag <= " << threshold << "..." << endl;
  range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
           new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))); 
	        // aggiunge una condizione -> (	  ^nome variabile, ^GT = maggiore di, ^parametro di condizione (soglia magnitudine)
  // Build the filter
  pcl::ConditionalRemoval<PointNormal> condrem (range_cond);
  condrem.setInputCloud (doncloud);

  pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);

  // Apply filter
  condrem.filter (*doncloud_filtered);

  doncloud = doncloud_filtered;

  // Save filtered output
  std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;

  writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false); 
}

void euclideanExctraction(){
  cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;
  segm_tree->setInputCloud (doncloud);
  euclideanClusterExctraction.setClusterTolerance (segradius);
  euclideanClusterExctraction.setMinClusterSize (50);
  euclideanClusterExctraction.setMaxClusterSize (100000);
  euclideanClusterExctraction.setSearchMethod (segm_tree);
  euclideanClusterExctraction.setInputCloud (doncloud);af
  euclideanClusterExctraction.extract (cluster_indices);
}


void don_segm (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*input, pcl_pc);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<PointXYZ>);

  pcl::fromPCLPointCloud2(pcl_pc, *cloud_in);
  writer.write<pcl::PointXYZ> ("beforeFiltering.pcd", *cloud_in, false); 
  ROS_INFO("Removing NaN");
  std::vector<int> mapping;
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, mapping);

  //Voxelgrid filtering
  ROS_INFO("Start voxel filtering");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_in);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (*cloud_filtered);

  input_pc = cloud_filtered;
writer.write<pcl::PointXYZ> ("afterFiltering.pcd", *input_pc, false); 
  //KD TREE
  setting_Kdtree_for_normal();
  if (scale1 >= scale2)
  {
    cerr << "Error: Large scale must be > small scale!" << endl;
    exit (EXIT_FAILURE);
  }
  //SETTING LARGE AND SMALL RADIUS AND CALCULATING DIFFERENCE
  calculating_small_and_big_normals();
 
  // Save DoN features

  writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false); 

  //il filtro è applicato per discriminare i punti in base al vettore risultante della loro DoN. 
  filter();

  euclideanExctraction();
  
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
  {
    pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<PointNormal>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster_don->points.push_back (doncloud->points[*pit]);
    }

    cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
    cloud_cluster_don->height = 1;
    cloud_cluster_don->is_dense = true;

    //Save cluster
    cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
    stringstream ss;
    ss << "don_cluster_" << j << ".pcd";
    writer.write<pcl::PointNormal> (ss.str (), *cloud_cluster_don, false);
  }
}

int
main (int argc, char *argv[])
{

  ros::init (argc, argv, "pc_segmentation");
  ros::NodeHandle nh;
  //ROS_INFO("I am here main 0 ");
  /*
  scale1 = 0.1;
  scale2 = 0.2;
  threshold = 0.3;
  segradius = 0.5;
  */
  ros::NodeHandle n("~");
  n.param("scale1", scale1, 0.2);
  n.param("scale2", scale2, 0.4);
  n.param("threshold", threshold, 0.2);
  n.param("segradius", segradius, 0.1);
  std::string topic_name;
  n.param<std::string>("topic", topic_name,"/assembled_cloud");

  ros::Subscriber sub = nh.subscribe (topic_name, 1, don_segm);
  //ROS_INFO("I am here main");
  ros::spin ();
}

