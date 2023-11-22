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

HEAD int CallingConvention planeModelSegmentation(pcl::PointCloud<pcl::PointXYZ>* cloud,int maxIteration = 1000,float distanceThreshold = 0.01f)
{
	//pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	//pcl::PCDReader reader;
	//reader.read ("table_scene_lms400.pcd", *cloud_blob);

	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points." << std::endl;

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud->makeShared());
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_filtered);

	//cloud_filtered = cloud->makeShared();

	// Convert to the templated PointCloud
	//pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

	// Write the downsampled version to disk
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("tmp_downsampled.pcd", *cloud_filtered, false);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setAxis(Eigen::Vector3f(1.0, 0.0, 0.0));
	seg.setEpsAngle(pcl::deg2rad((double)15.0f));
	seg.setMaxIterations (maxIteration);
	seg.setDistanceThreshold (distanceThreshold);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	int i = 0, nr_points = (int) cloud_filtered->size ();
	// While 30% of the original cloud is still there
	while (cloud_filtered->size () > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the inliers
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_p);
		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

		std::stringstream ss;
		ss << "tmp_" << i << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

		// Create the filtering object
		extract.setNegative (true);
		extract.filter (*cloud_f);
		cloud_filtered.swap (cloud_f);
		i++;
	}

	return i;
}

vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterExtraction(pcl::PointCloud<pcl::PointXYZ>* cloud, int minClusterSize, int maxClusterSize, float tolerance)
{
	//pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	//pcl::PCDReader reader;
	//reader.read ("table_scene_lms400.pcd", *cloud_blob);

	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points." << std::endl;

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud->makeShared());
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*cloud_filtered);

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(tolerance); // 2cm
	ec.setMinClusterSize(minClusterSize);
	ec.setMaxClusterSize(maxClusterSize);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);
	pcl::PCDWriter writer;
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
	for (const auto& cluster : cluster_indices)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (const auto& idx : cluster.indices) {
			cloud_cluster->push_back((*cloud_filtered)[idx]);
		} //*
		cloud_cluster->width = cloud_cluster->size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
		//std::stringstream ss;
		//ss << std::setw(4) << std::setfill('0') << j;
		//writer.write<pcl::PointXYZ>("seg_" + to_string(j) + ".pcd", *cloud_cluster, false); //*
		clusters.push_back(cloud_cluster);
	}
	return clusters;
}

double getPointCloudAverageHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	double totalHeight = 0.0;
	for (int i = 0; i < cloud->points.size(); i++)
	{
		pcl::PointXYZ point = cloud->points[i];
		totalHeight += point.z;
	}
	return totalHeight / (double)cloud->points.size();
}

struct ClusterAverageHeight
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;
	double averageHeight;
	ClusterAverageHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr _cloudPtr)
	{
		cloudPtr = _cloudPtr;
		updateAverage();
	}
	void updateAverage()
	{
		double totalHeight = 0.0;
		for (int i = 0; i < cloudPtr->points.size(); i++)
		{
			pcl::PointXYZ point = cloudPtr->points[i];
			totalHeight += point.z;
		}
		averageHeight = totalHeight / (double)cloudPtr->points.size();
	}
	void addCloud(const ClusterAverageHeight& target)
	{
		for (int i = 0; i < target.cloudPtr->points.size(); i++)
		{
			this->cloudPtr->push_back(target.cloudPtr->points[i]);
		}
		updateAverage();
	}
};

HEAD int CallingConvention layerExtraction(pcl::PointCloud<pcl::PointXYZ>* cloud,int minClusterSize = 100,int maxClusterSize = 25000, float tolerance = 0.02f, double clusterMergeThreshold = 0.01f)
{
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = clusterExtraction(cloud, minClusterSize, maxClusterSize, tolerance);
	vector<ClusterAverageHeight> layers;
	for (int i = 0; i < clusters.size(); i++)
	{
		layers.push_back(ClusterAverageHeight(clusters[i]));
		cout << "cluster " << i << " average height:" << layers.back().averageHeight << endl;
	}
	
	cout << "merging cluster into layer" << endl;
	for (int i = 0; i < layers.size(); i++)
	{
		for (int j = i + 1; j < layers.size(); j++)
		{
			if (fabs(layers[i].averageHeight - layers[j].averageHeight) < clusterMergeThreshold)
			{
				layers[i].addCloud(layers[j]);
				layers.erase(layers.begin() + j);
				j--;
			}
		}
	}
	cout << "sorting layer" << endl;
	for (int i = 0; i < layers.size(); i++)
	{
		for (int j = i + 1; j < layers.size(); j++)
		{
			if (layers[i].averageHeight > layers[j].averageHeight)
			{
				swap(layers[i], layers[j]);
			}
		}
	}
	cout << "outputing" << endl;
	pcl::PCDWriter writer;
	fstream layerDistanceFile;
	layerDistanceFile.open("layer_distance.txt", std::ofstream::out | std::ofstream::trunc);
	for (int i = 0; i < layers.size(); i++)
	{
		writer.write<pcl::PointXYZ>("layer_" + to_string(i) + ".pcd", *layers[i].cloudPtr, false);
		if (i != layers.size() - 1)
		{
			layerDistanceFile << layers[i + 1].averageHeight - layers[i].averageHeight << endl;
		}
	}
	layerDistanceFile.close();
	return layers.size();
}