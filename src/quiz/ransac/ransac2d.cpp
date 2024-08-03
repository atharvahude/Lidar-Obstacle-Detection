/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <utility>

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}


std::pair<int,int> generateRandomNumbers (int start, int end){
	//enter the starting and end to get random numbers from the range. Number will be inclusive

	//Seed
	std::srand(std::time(0));

	int num1 = start + std::rand() % (end - start + 1);
	int num2;
	
	do {
		num2 = start + std::rand() % (end - start + 1);
	}while (num1 == num2);

	return  std::make_pair(num1,num2);
}


std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	
	std::unordered_set<int> resultInliers ;
	
	for(int i {0}; i<maxIterations; i++)
	{	
		std::unordered_set<int> inliers ;
		auto nums = generateRandomNumbers(0,cloud->width-1);

		pcl::PointXYZ first_point = cloud->points[nums.first];
		pcl::PointXYZ second_point = cloud->points[nums.second];

		double xcoeff {second_point.x-first_point.x};
		double ycoeff {first_point.y - second_point.y};
		double constcoeff {(first_point.x * second_point.y)-(second_point.x * first_point.y)};

		double distance {0};

		for(int index = 0; index < cloud->points.size(); index++)
		{
			distance = std::abs((cloud->points[index].x*xcoeff)+(cloud->points[index].y*ycoeff)+constcoeff)/std::sqrt(xcoeff*xcoeff+ycoeff*ycoeff);

			if (distance <= distanceTol)
				{
					inliers.insert(index);
				}
		
		}

		if (inliers.size() > resultInliers.size())
		{
			resultInliers = inliers;
		}
	}
	
	return resultInliers;

}

std::unordered_set<int> RansacCustom(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> finalInliers;

	while(maxIterations -- ){
		std::unordered_set<int> inliers;

		//As the inliers is unordered set it will always generate 2 unique numbers. 
		while (inliers.size()<2){
			inliers.insert(rand()%(cloud->points.size()));
		}

		float x1,y1,x2,y2;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr ++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		float a = (y1-y2);
		float b = (x2-x1);
		float c = (x1*y2-x2*y1);

		for (int i {0};i <cloud->points.size(); i++){

			if(inliers.count(i)>0){
				continue;
			}

			pcl::PointXYZ point = cloud->points[i];
			float x3 = point.x;
			float y3 = point.y;

			float d = fabs(a*x3+b*y3+c)/sqrt(a*a+b*b);

			if (d<= distanceTol){
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


std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> finalInliers;

	while(maxIterations -- ){
		std::unordered_set<int> inliers;

		//As the inliers is unordered set it will always generate 2 unique numbers. 
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

			pcl::PointXYZ point = cloud->points[i];
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






int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 20, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
