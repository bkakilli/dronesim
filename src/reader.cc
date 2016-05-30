#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <cstring>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "DronesimShm.hh"

typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> pclVis;

bool endProgram;
pclVis viewer;

void doSmth(float* pcd) {

   pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud (new pcl::PointCloud<pcl::PointXYZ>);

   /*
   for(int i=0; i<480; i++) {

      for(int j=0; j<640; j++) {
         pcl::PointXYZ ptXYZ;

         ptXYZ.x = pcd[4*(i*640+j)];
         ptXYZ.y = pcd[4*(i*640+j)+1];
         ptXYZ.z = pcd[4*(i*640+j)+2];

         pCloud->points.push_back(ptXYZ);
      }
   }*/

   for (int i = 0; i < trace_queue::BUFFERSIZE; i=i+4)
   {
      pcl::PointXYZ ptXYZ;

      ptXYZ.x = pcd[i];
      ptXYZ.y = pcd[i+1];
      ptXYZ.z = pcd[i+2];

      pCloud->points.push_back(ptXYZ);
   }

   viewer->removeAllPointClouds();
   viewer->addPointCloud<pcl::PointXYZ> (pCloud, "sample cloud");
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
   viewer->initCameraParameters ();
   viewer->spinOnce();

}
void my_handler(int s){
   endProgram = true;
}

int main ()
{

   struct sigaction sigIntHandler;

   sigIntHandler.sa_handler = my_handler;
   sigemptyset(&sigIntHandler.sa_mask);
   sigIntHandler.sa_flags = 0;

   sigaction(SIGINT, &sigIntHandler, NULL);

   endProgram = false;
   // Setup PCL visualizer
   viewer = pclVis(new pcl::visualization::PCLVisualizer ("3D Viewer"));
   viewer->setBackgroundColor (0, 0, 0);
   //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
   viewer->initCameraParameters ();

   ShmReader shmReader;

   while (!endProgram)
   { 
      shmReader.read();
      doSmth(shmReader.data->buffer);
      shmReader.releaseMem();
   }

   shmReader.data->p_end = true;
   

   viewer->close();

   return 0;
}