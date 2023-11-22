#pragma once
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h> // for KdTree
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <string>
#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;
//定义导出方式：以C语言的方式导出，因为C语言方式函数名保持不变
#define EXTERNC extern "C"
//定义dll导出方式，此处是导出，如果是导入则为dllinport
#define HEAD EXTERNC __declspec(dllexport)
//定义调用约定，此处选择标准调用约定，也可以用c调用约定
#define CallingConvention __stdcall

//PointCloud to Mesh using greedy projection
HEAD pcl::PolygonMesh* CallingConvention greedyProjection(pcl::PointCloud<pcl::PointXYZ>* cloud);

//save mesh to file
HEAD void CallingConvention saveMesh(pcl::PolygonMesh* meshPtr, char* path, char* file_extension);

//Plane model segmentation
HEAD int CallingConvention planeModelSegmentation(pcl::PointCloud<pcl::PointXYZ>* cloud, int maxIteration, float distanceThreshold);

vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterExtraction(pcl::PointCloud<pcl::PointXYZ>* cloud, int minClusterSize, int maxClusterSize, float tolerance);

double getPointCloudAverageHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr);

HEAD int CallingConvention layerExtraction(pcl::PointCloud<pcl::PointXYZ>* cloud, int minClusterSize, int maxClusterSize, float tolerance,double clusterMergeThreshold);
