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

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "myhead.hh"

using namespace boost::interprocess;

typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> pclVis;

bool endProgram;
pclVis viewer;

void doSmth(float* pcd) {

   pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud (new pcl::PointCloud<pcl::PointXYZ>);

   for(int i=1; i<480; i++) {

      for(int j=1; j<640; j++) {
         pcl::PointXYZ ptXYZ;

         ptXYZ.x = pcd[4*(i*640+j)];
         ptXYZ.y = pcd[4*(i*640+j)+1];
         ptXYZ.z = pcd[4*(i*640+j)+2];

         pCloud->points.push_back(ptXYZ);
      }
   }

   viewer->removeAllPointClouds();
   viewer->addPointCloud<pcl::PointXYZ> (pCloud, "sample cloud");
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
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
   viewer->initCameraParameters ();

   //Create a shared memory object.
   shared_memory_object shm
      (open_only                    //only create
      ,"MySharedMemory"              //name
      ,read_write                   //read-write mode
      );

   try{
      //Map the whole shared memory in this process
      mapped_region region
         (shm                       //What to map
         ,read_write //Map it as read-write
         );

      //Get the address of the mapped region
      void * addr       = region.get_address();

      //Obtain a pointer to the shared structure
      trace_queue * data = static_cast<trace_queue*>(addr);

      float pcd[trace_queue::BUFFERSIZE];

      while (!endProgram)
      {
         scoped_lock<interprocess_mutex> lock(data->mutex);
         data->cond.wait(lock);
         data->reading = true;
         if (data->endConnection) {
            endProgram = true;
            break;
         }

         doSmth(data->buffer);
         //memcpy ( pcd, data->buffer, trace_queue::BUFFERSIZE );

         data->reading = false;
      }
   }
   catch(interprocess_exception &ex){
      std::cout << ex.what() << std::endl;
      return 1;
   }

   viewer->close();

   return 0;
}