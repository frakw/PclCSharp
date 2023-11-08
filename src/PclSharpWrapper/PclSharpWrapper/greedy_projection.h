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
