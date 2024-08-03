// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <utility>
#include <unordered_set>
#include "kdtree3D.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    // Create the filtering object
    
    pcl::VoxelGrid<PointT> sor;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);


    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr regionCloud (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> regionOfInterest(true); 
    regionOfInterest.setMin(minPoint);
    regionOfInterest.setMax(maxPoint);
    regionOfInterest.setInputCloud(cloudFiltered);
    regionOfInterest.filter(*regionCloud);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return regionCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    // Extract the inliers
    typename pcl::PointCloud<PointT>::Ptr road_pointcloud (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr obstacle_pointcloud (new pcl::PointCloud<PointT>);
    typename pcl::ExtractIndices<PointT> extract;

    for (auto i : inliers->indices){
        road_pointcloud->points.push_back(cloud->points[i]);
    }

    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstacle_pointcloud);  

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_pointcloud, road_pointcloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    
    //Create Segmentation Object
    pcl::SACSegmentation<PointT> seg;

    //Setting some parameters for segmentation of the plane 
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    //Extracting the largest plane from the point cloud

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    
    //Checking the inliers is not zero ie. segmentation has dont its job.
    if (inliers->indices.size () == 0)

    {
          std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new typename pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;
 
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int j = 0;
    for (const auto& cluster : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new typename pcl::PointCloud<PointT>);
        for (const auto& idx : cluster.indices) 
        {
            cloud_cluster->push_back((*cloud)[idx]);
        } 
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    
    clusters.push_back(cloud_cluster);

    j++;
    
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

template <typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3D_CUSTOM(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> finalInliers;

	while(maxIterations -- ){
		std::unordered_set<int> inliers;

		//As the inliers is unordered set it will always generate 3 unique numbers. 
		while (inliers.size()<3){
			inliers.insert(rand()%(cloud->points.size()));
		}

		float x1,x2,y1,x3,y2,y3,z1,z2,z3;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr ++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr ++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float 	A {(y2-y1)*(z3-z1)-(z2-z1)*(y3-y1)};
		float 	B {(z2-z1)*(x3-x1)-(x2-x1)*(z3-z1)};
		float 	C {(x2-x1)*(y3-y1)-(y2-y1)*(x3-x1)};
		float	D {-(A*x1+B*y1+C*z1)};

		for (int i {0};i <cloud->points.size(); i++){

			if(inliers.count(i)>0){
				continue;
			}

			PointT point = cloud->points[i];
			float x = point.x;
			float y = point.y;
			float z = point.z;

			float distance = fabs(A*x+B*y+C*z+D)/sqrt(A*A+B*B+C*C);

			if (distance<= distanceTol){
				inliers.insert(i);
			}

		}

		if (inliers.size() > finalInliers.size())
		{
			finalInliers = inliers;
		}
	}
	
	return finalInliers;

	}

//Implementing quiz using templates.
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneCUSTOM(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{

    std::unordered_set<int> inliers = Ransac3D_CUSTOM(cloud, maxIterations, distanceThreshold);

	typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers,cloudInliers);
    return segResult;
}

template <typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed ,KdTree* tree, float distanceTol){

	processed[indice] = true;
	cluster.push_back(indice);

	std::vector<int> nearest = tree->search(points[indice],distanceTol);

	for (int i : nearest)
	{
		if (!processed[i])
			clusterHelper(i,points,cluster,processed,tree,distanceTol);
	}


}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster_CUSTOM(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol,float minSize, float maxSize)
{

	std::vector<std::vector<int>> clusters; 
	std::vector<bool> processed (points.size(),false);

	int i = 0;
	while (i<points.size()){
		if (processed[i]){
			i++;
			continue;
		}

		std::vector<int> cluster;
		clusterHelper(i,points,cluster,processed,tree,distanceTol);
        if (cluster.size() >= minSize && cluster.size() <= maxSize)
        {
                
            clusters.push_back(cluster);
                
        }

		i++;
	}


	return clusters;

}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::clustering_CUSTOM(const typename pcl::PointCloud<PointT>::Ptr &cloud, float clusterTolerance, float minSize, float maxSize)
{
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    KdTree *tree3D = new KdTree;
    std::vector<std::vector<float>> inputPoints(cloud->points.size());

    for (int i = 0; i < cloud->points.size(); i++)
    {
        std::vector<float> inputPoint = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        tree3D->insert(inputPoint, i);
        inputPoints[i] = inputPoint;
    }

    std::vector<std::vector<int>> clusterIndices = euclideanCluster_CUSTOM(inputPoints, tree3D, clusterTolerance,minSize, maxSize);

    for (std::vector<int> cluster : clusterIndices)
    {

        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for (int index : cluster)
        {
            cloudCluster->points.push_back(cloud->points[index]); 
        }
        
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);

    }
    return clusters;
}
