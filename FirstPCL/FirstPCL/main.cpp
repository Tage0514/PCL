/*
*
* ━━━━━━神兽出没━━━━━━
* 　　　┏┓　　　┏┓
* 　　┏┛┻━━━┛┻┓
* 　　┃　　　　　　　┃
* 　　┃　　　━　　　┃
* 　　┃　┳┛　┗┳　┃
* 　　┃　　　　　　　┃
* 　　┃　　　┻　　　┃
* 　　┃　　　　　　　┃
* 　　┗━┓　　　┏━┛Code is far away from bug with the animal protecting
* 　　　　┃　　　┃    神兽保佑,代码无bug
* 　　　　┃　　　┃
* 　　　　┃　　　┗━━━┓
* 　　　　┃　　　　　　　┣┓
* 　　　　┃　　　　　　　┏┛
* 　　　　┗┓┓┏━┳┓┏┛
* 　　　　　┃┫┫　┃┫┫
* 　　　　　┗┻┛　┗┻┛
*
* ━━━━━━感觉萌萌哒━━━━━━
*/

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <vtkAutoInit.h>
#include "stdafx.h"
#include <strsafe.h>
#include "DepthBasics.h"
#include "resource.h"


//VTK_MODULE_INIT(vtkRenderingOpenGL)

using namespace std;
//int user_data;
//
//void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
//{
//	viewer.setBackgroundColor(1,1,1);
//	pcl::PointXYZ o;
//	o.x = 1.0;
//	o.y = 0;
//	o.z = 0;
//	viewer.addSphere(o, 0.25, "sphere", 0);
//	cout << "i only run once" << std::endl;
//}
//
//
//
//void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
//{
//	static unsigned count = 0;
//	stringstream ss;
//	ss << "Once per viewer loop: " << count++;
//	viewer.removeShape("text", 0);
//	viewer.addText(ss.str(), 200, 300, "text", 0);
//	//FIXME: possible race condition here:
//	user_data++;
//}



int main()
{
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("test_pcd.pcd", *cloud);*/
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	CDepthBasics application;
	float *vertices = new float[640 * 480 * 3];

	for (int i = 0; i < 480; i++) {
		for (int j = 0; j < 640; j++)
		{
			vertices[3 * (640 * i + j) + 0] = ((float)j - 320) * 0.01; // x
			vertices[3 * (640 * i + j) + 1] = -((float)i - 240) * 0.01; // y
			vertices[3 * (640 * i + j) + 2] = 0;
		}
	}

	application.CreateFirstConnected();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	while (1) {
		application.Update();
		for (int i = 0; i < 640 * 480; i++) {
			vertices[3 * i + 2] = application.depthValues[i] > 0 ? (float)application.depthValues[i] * 0.005 : 200;
		}

		// Fill in the cloud data
		cloud->width = 640;
		cloud->height = 480;
		cloud->points.resize(cloud->width * cloud->height);

		for (int i = 0; i < 640 * 480; i++)
		{
			cloud->points[i].x = vertices[3 * i];
			cloud->points[i].y = vertices[3 * i + 1];
			cloud->points[i].z = vertices[3 * i + 2];
		}

		/*std::cerr << "Cloud before filtering: " << std::endl;
		for (size_t i = 0; i < cloud->points.size(); ++i)
			std::cerr << "    " << cloud->points[i].x << " "
			<< cloud->points[i].y << " "
			<< cloud->points[i].z << std::endl;*/


		//blocks until the cloud is actually rendered

		viewer.showCloud(cloud);
		// Normal estimation*
		//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		//pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		//tree->setInputCloud(cloud);
		//n.setInputCloud(cloud);
		//n.setSearchMethod(tree);
		//n.setKSearch(20);
		//n.compute(*normals);
		////* normals should not contain the point normals + surface curvatures

		//// Concatenate the XYZ and normal fields*
		//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
		//pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
		////* cloud_with_normals = cloud + normals

		//// Create search tree*
		//pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
		//tree2->setInputCloud(cloud_with_normals);

		//// Initialize objects
		//pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		//pcl::PolygonMesh triangles;

		//// Set the maximum distance between connected points (maximum edge length)
		//gp3.setSearchRadius(0.1);

		//// Set typical values for the parameters
		//gp3.setMu(2.5);
		//gp3.setMaximumNearestNeighbors(100);
		//gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
		//gp3.setMinimumAngle(M_PI / 18); // 10 degrees
		//gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
		//gp3.setNormalConsistency(false);

		//// Get result
		//gp3.setInputCloud(cloud_with_normals);
		//gp3.setSearchMethod(tree2);
		//gp3.reconstruct(triangles);

		//// Additional vertex information
		//std::vector<int> parts = gp3.getPartIDs();
		//std::vector<int> states = gp3.getPointStates();

		//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		//viewer->setBackgroundColor(0, 0, 0);

		//viewer->addPolygonMesh(triangles, "triangles");

		//viewer->addCoordinateSystem(1.0);
		//viewer->initCameraParameters();
		//while (!viewer->wasStopped()) {
		//	viewer->spinOnce(100);
		//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		//}


	
	}
	
	/*use the following functions to get access to the underlying more advanced/powerful
	PCLVisualizer
	This will only get called once
	viewer.runOnVisualizationThreadOnce(viewerOneOff);*/
	//This will get called once per visualization iteration

	//viewer.runOnVisualizationThread(viewerPsycho);

	//while (!viewer.wasStopped())
	//{
	//	/*you can also do cool processing here
	//	FIXME: Note that this is running in a separate thread from viewerPsycho
	//	and you should guard against race conditions yourself...*/
	//	user_data++;
	//}
	return 0;
}



