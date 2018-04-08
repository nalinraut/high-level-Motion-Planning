#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>

int main (int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("point_cloud_data/rgbrgb.pcd", *cloud) == -1) {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return -1;
  }

  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  ne.setRadiusSearch(0.03);
  ne.setViewPoint(0, 0, 0);

  std::cout << "Computing...\n";
  ne.compute(*cloud_normals);
  //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  //std::vector<int> a;
  //pcl::removeNaNNormalsFromPointCloud(*cloud_normals, *cloud_normals, a);
  std::cout << "Done\n";
  
  std::cout << cloud_normals->points.size() << "\n";
  
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  
  viewer.setBackgroundColor (0.0, 0.0, 0.0);
  viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "rgbcloud");
  viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, cloud_normals, 20, 0.2, "normals");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "rgbcloud");
  
  ofstream file;
  file.open("normals.txt");
  for(int i=0; i<cloud_normals->points.size(); i++) {
    pcl::Normal cur = cloud_normals->points[i];
    if(isnan(cur.normal_x)) {
      continue;
    }
    file << cur.normal_x << " " << cur.normal_y << " " << cur.normal_z << "\n";
  }
  file.close();
  std::cout << "Done writing to file.\n";
  
  while(!viewer.wasStopped()) {
    viewer.spinOnce();
  }
  
  return 0;
}
