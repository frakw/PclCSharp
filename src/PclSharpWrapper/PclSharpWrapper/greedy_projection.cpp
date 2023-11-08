#include "greedy_projection.h"

//PointCloud to Mesh using greedy projection
HEAD pcl::PolygonMesh* CallingConvention greedyProjection(pcl::PointCloud<pcl::PointXYZ>* cloud)
{
	if (cloud == nullptr) return nullptr;
	cout << "greedyProjection start" << endl;
	// Load input file into a PointCloud<T> with an appropriate type

	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile("bun0.pcd", cloud_blob);
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
	*/

	//* the data should be available in cloud

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud->makeShared());
	n.setInputCloud(cloud->makeShared());
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh* triangles = new pcl::PolygonMesh();

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(*triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	

	cout << "greedyProjection finished" << endl;

	return triangles;
	// Finish
}

HEAD void CallingConvention saveMesh(pcl::PolygonMesh* meshPtr, char* path, char* file_extension)
{
	
	string path_str = path;
	string file_extension_str = file_extension;

	if (meshPtr == nullptr || path_str.empty() || file_extension_str.empty())
	{
		cout << "error input" << endl;
		return;
	}

	if (file_extension_str[0] != '.')
	{
		file_extension_str = "." + file_extension_str;
	}

	if (file_extension_str == ".obj")
	{
		pcl::io::saveOBJFile(path_str, *meshPtr);
	}
	else if (file_extension_str == ".ply")
	{
		pcl::io::savePLYFile(path_str, *meshPtr);
	}
	else if (file_extension_str == ".stl")
	{
		pcl::io::savePolygonFileSTL(path_str, *meshPtr);
	}
	else if (file_extension_str == ".vtk")
	{
		pcl::io::saveVTKFile(path_str, *meshPtr);
	}
}