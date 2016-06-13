#include <iostream>
#include <string>
#include <sstream>
#include <signal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "DronesimShm.hh"

//#include <boost/asio.hpp>
//#include <boost/array.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> pclVis;

bool endProgram;
pclVis viewer;
boost::array<float, trace_queue::BUFFERSIZE> socketBuffer;

enum constants {DOWNSAMPLE_RATIO=4, LEFT_ARROW=37, UP_ARROW, RIGHT_ARROW, DOWN_ARROW};

//void sendFloatArray(std::string host, int port, float floatArray[], int size)
//{
//   boost::asio::io_service ios;
//   boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(host), port);
//   boost::asio::ip::tcp::socket socket(ios);
//   socket.connect(endpoint);
//   usleep(100000);
//
//   std::memcpy(&socketBuffer[0], floatArray, size*4);
//   boost::system::error_code error;
//   socket.write_some(boost::asio::buffer(socketBuffer, size*4), error);
//
//   socket.close();
//}

float distanceTo( Eigen::Vector4f p ) {
   Eigen::Vector3f pt (1, 1, -(p(0)*1+p(1)*1+p(3))/p(2) );
   Eigen::Vector3f n( p.head<3>() );

   return std::abs( n.dot(pt) );
}


static float distance2f( const cv::Point2f &a, const cv::Point2f &b )
{
   return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}
static void largestEmptyCircle( cv::Rect &rect, std::vector<cv::Point2f> &points, float &lec_radius, cv::Point2f &lec_center )
{
   std::vector< cv::Point2f > ch;
   cv::convexHull(points, ch);

   cv::Subdiv2D subdiv(rect);
   for (std::vector<cv::Point2f>::iterator it = points.begin(); it != points.end(); it++)
      subdiv.insert(*it);

   std::vector<std::vector<cv::Point2f> > facets;
   std::vector<cv::Point2f> centers;
   subdiv.getVoronoiFacetList(std::vector<int>(), facets, centers);

   lec_radius = 0;
   for ( size_t i = 0; i < facets.size(); i++ )
   {
      for ( size_t j = 0; j < facets[i].size(); j++ )
      {
         if ( cv::pointPolygonTest(ch, facets[i][j], false) == +1 )
         {
            for ( size_t k = 0; k < centers.size(); k++ )
            {
               float d = distance2f(centers[i], facets[i][j]);
               if ( d > lec_radius )
               {
                  lec_radius = d;
                  lec_center = facets[i][j];
               }
            }
         }
      }
   }
}

void doSmth(float* pcd) {

   pcl::PointCloud<pcl::PointXYZ>::Ptr unfilteredCloud (new pcl::PointCloud<pcl::PointXYZ>);

   /*
   for(int i=0; i<480; i++) {

      for(int j=0; j<640; j++) {
         pcl::PointXYZ ptXYZ;

         ptXYZ.x = pcd[4*(i*640+j)];
         ptXYZ.y = pcd[4*(i*640+j)+1];
         ptXYZ.z = pcd[4*(i*640+j)+2];

         unfilteredCloud->points.push_back(ptXYZ);
      }
   }*/

//   try{
//
//      int pcdLen = trace_queue::BUFFERSIZE;
//      int headerCount = 6;
//      float sendArray[pcdLen*3/4+headerCount];
//      int pointCount = 0;
//
//      for (int i = 0; i < pcdLen; i=i+4*DOWNSAMPLE_RATIO)
//      {
//         int mode = i%(640*4*DOWNSAMPLE_RATIO);
//         if ( mode == 640*4 )
//            i = i+(640*4*DOWNSAMPLE_RATIO-mode);
//
//
//         if( !(pcd[i] == 0 && pcd[i+1] == 0 && pcd[i+2] == 0 && pcd[i+3] == 1) ) {
//            memcpy( /*headerOffset: */ headerCount*4 +sendArray+3*pointCount, pcd+i, 12);
//            pointCount++;
//         }
//      }
//
//      std::cout << "Point Count: " << pointCount << std::endl;
//
//      sendArray[0] = (3*pointCount+headerCount)*4;
//      sendArray[1] = 0;//translation[0];
//      sendArray[2] = 0;//translation[1];
//      sendArray[3] = 0;//translation[2];
//      sendArray[4] = 0.5;//theta;
//      sendArray[5] = 0;//timestamp;
//
//      sendFloatArray("128.230.214.68", 30000, sendArray, 3*pointCount+headerCount);
//   } catch (...){}

   for (int i = 0; i < trace_queue::BUFFERSIZE; i=i+4)
   {
      pcl::PointXYZ ptXYZ;

      ptXYZ.x = pcd[i];
      ptXYZ.y = pcd[i+1];
      ptXYZ.z = pcd[i+2];

      unfilteredCloud->points.push_back(ptXYZ);
   }

   pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudRot (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::VoxelGrid<pcl::PointXYZ> sor;
   sor.setInputCloud (unfilteredCloud);
   sor.setLeafSize (0.01f, 0.01f, 0.01f);
   sor.filter (*pCloudRot);

   pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud (new pcl::PointCloud<pcl::PointXYZ>);
   Eigen::Matrix4f rotMat = Eigen::Matrix4f::Identity();

   // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
   float theta = 0.5; // The angle of rotation in radians
   rotMat (1,1) = cos (theta);
   rotMat (1,2) = -sin(theta);
   rotMat (2,1) = sin (theta);
   rotMat (2,2) = cos (theta);
   pcl::transformPointCloud (*pCloudRot, *pCloud, rotMat);

   pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedPlane (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

   // Create the segmentation object
   pcl::SACSegmentation<pcl::PointXYZ> seg;
   // Mandatory
   seg.setModelType (pcl::SACMODEL_PLANE);
   seg.setMethodType (pcl::SAC_RANSAC);
   seg.setDistanceThreshold (0.05);

   // Create the filtering object
   pcl::ExtractIndices<pcl::PointXYZ> extract;

   viewer->removeAllPointClouds();

   int totalNumOfPoints = pCloud->points.size();
   int palette[30][3] = {
                        {255,0,0},
                        {0,255,0},
                        {0,0,255},
                        {255,255,0},
                        {255,0,255},
                        {0,255,255},
                        {255,128,0},
                        {255,0,128},
                        {128,255,0},
                        {128,0,255},
                        {255,0,0},
                        {0,255,0},
                        {0,0,255},
                        {255,255,0},
                        {255,0,255},
                        {255,0,0},
                        {0,255,0},
                        {0,0,255},
                        {255,255,0},
                        {255,0,255},
                        {0,255,255},
                        {255,128,0},
                        {255,0,128},
                        {128,255,0},
                        {128,0,255},
                        {255,0,0},
                        {0,255,0},
                        {0,0,255},
                        {255,255,0},
                        {255,0,255}
                     };
   int planeCount=0;
   //std::vector< std::vector<float> > planes;
   std::vector< Eigen::Vector4f > planes;
   std::vector<float> planeDistances;
   std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > planePoints;
   while ( pCloud->points.size() > totalNumOfPoints*0.15 )
   {
      seg.setInputCloud (pCloud);
      seg.segment (*inliers, *coefficients);

      //std::cout << "Model coefficients: " << coefficients->values[0] << " " 
      //                                    << coefficients->values[1] << " "
      //                                    << coefficients->values[2] << " " 
      //                                    << coefficients->values[3] << std::endl;

      extract.setInputCloud (pCloud);
      extract.setIndices (inliers);
      extract.setNegative(false);
      extract.filter (*segmentedPlane);
      // Do smth with segmentedPlane
      
      planes.push_back( Eigen::Vector4f(coefficients->values.data()) );
      planeDistances.push_back( distanceTo(planes.back()) );
      planePoints.push_back( segmentedPlane->makeShared() );

      //planes.push_back( coefficients->values );

      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(segmentedPlane, palette[planeCount][0], palette[planeCount][1], palette[planeCount][2]);
      std::stringstream ss;
      ss << "cloud_" << planeCount;
      viewer->addPointCloud<pcl::PointXYZ> (segmentedPlane, single_color, ss.str().c_str());

      extract.setNegative (true);
      extract.filter (*pCloud);
      planeCount++;
   }

   viewer->spinOnce();

   std::vector<int> hIndices;
   Eigen::Vector3f worldNormal(0,1,0);
   float COS_VALUE=0.965926;

   for (int i = 0; i < planes.size(); ++i)
   {
      Eigen::Vector3f normal( planes[i].head<3>() );
      if ( std::abs( normal.dot(worldNormal) ) > COS_VALUE )
         hIndices.push_back(i);

      std::cout << "Model coefficients: " << planes[i](0) << " " 
                                          << planes[i](1) << " "
                                          << planes[i](2) << " "
                                          << planes[i](3) << " "
                   "Distance: "           << planeDistances[i] << std::endl;
      
   }

   float dMax=0, dMin=FLT_MAX;
   int lowerPlane=-1, upperPlane=-1;
   std::cout << "Horizontal ones: ";
   for (std::vector<int>::const_iterator i = hIndices.begin(); i != hIndices.end(); ++i) {
      if( planeDistances[*i] > dMax ) {
         dMax = planeDistances[*i];
         lowerPlane = *i;
      }
      if( planeDistances[*i] < dMin ) {
         dMin = planeDistances[*i];
         upperPlane = *i;
      }
      std::cout << *i << " ";
   }
   std::cout << std::endl;

   std::cout << "Lower plane index: " << lowerPlane << "\n" << std::endl;
   // Landing area detection
   // get the uppermost horizontal plane first
   pcl::PointCloud<pcl::PointXYZ>::Ptr currentPlane = planePoints[upperPlane];
   
   // Create binary image 

   int test=0;
   float maxX=0, maxY=0, minX=FLT_MAX, minY=FLT_MAX;
   for (int i=0; i<currentPlane->points.size(); i++)
   {
      pcl::PointXYZ* pt = &currentPlane->points[i];
      if(pt->x > maxX)
         maxX = pt->x;
      if(pt->z > maxY)
         maxY = pt->z;
      if(pt->x < minX)
         minX = pt->x;
      if(pt->z < minY)
         minY = pt->z;

      /*if (test<100){
         std::cout << "x: " << pt->x << " - y: " << pt->y << " - z: " << pt->z << std::endl;
      }
      test++;*/
   }

   int xGrid=256;
   float gridSize=(maxX-minX)/xGrid;
   int yGrid = ceil((maxY-minY)/gridSize);
   cv::Mat binImg = cv::Mat::zeros(yGrid, xGrid, CV_8UC1);

   for (int i=0; i<currentPlane->points.size(); i++)
   {
      pcl::PointXYZ* pt = &currentPlane->points[i];
      int xInd = (pt->x - minX) / gridSize;
      int yInd = (pt->z - minY) / gridSize;
      if (xInd < xGrid && yInd < yGrid) {
         binImg.data[yInd * xGrid + xInd] = 255;
      }
   }

   // Apply morphology
   int seSize=4;
   cv::Mat se = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*seSize + 1, 2*seSize+1 ));
   cv::morphologyEx(binImg, binImg, cv::MORPH_DILATE, se);
   cv::morphologyEx(binImg, binImg, cv::MORPH_CLOSE, se);

   // Find connected components
   cv::Mat labels = cv::Mat::zeros(binImg.size(), CV_32S);
   cv::Mat outImg = cv::Mat::zeros(binImg.size(), CV_8UC3);
   cv::Mat stats, centroids;
   int N = cv::connectedComponentsWithStats(binImg, labels, stats, centroids, 8, CV_32S);
   /*std::cout << "N: " << N << std::endl;
   std::cout << "stats:" << std::endl << "(area)" << std::endl << stats.col(4).rowRange(1,N) << std::endl;
   cv::Mat compSort;
   cv::sortIdx(stats.col(4).rowRange(1,N), compSort, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
   std::cout << "compSort:" << compSort+1 << std::endl;*/
   
   for (int i = 1; i < N; ++i)
   {
      //outImg.setTo(cv::Scalar(palette[i][0],palette[i][1],palette[i][2]), labels == i);
      std::vector<cv::Point2f> points;
      cv::Mat locations, edges;
      cv::Canny(labels == i, edges, 50, 150);
      cv::findNonZero(edges, locations);
      locations.copyTo(points);
      outImg.setTo(cv::Scalar(palette[i][0],palette[i][1],palette[i][2]), edges);
      float radi;
      cv::Point2f center;
      /*cv::Rect rect(stats.at<int>(i,0),
                                   stats.at<int>(i,1),
                                   stats.at<int>(i,2),
                                   stats.at<int>(i,3));*/
      cv::Rect rect(0,0,binImg.cols, binImg.rows);                             
      largestEmptyCircle( rect, points, radi, center );

      circle(outImg, center, 2, cv::Scalar(255, 255, 255));
      circle(outImg, center, radi, cv::Scalar(255, 255, 255), 2);
   }

   cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
   cv::imshow("Display window", outImg);
   cv::waitKey(1);

}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
   std::cout << "keycode=" << event.getKeyCode ();
  if (event.getKeyCode () == 'w' && event.keyDown ())
  {
    std::cout << "arrow down" << std::endl;
  }
  if (event.getKeyCode () == 'w' && event.keyUp ())
  {
    std::cout << "arrow up" << std::endl;
  }
}

void my_handler(int s){
   endProgram = true;
}


int
main (int argc, char** argv)
{
   struct sigaction sigIntHandler;

   sigIntHandler.sa_handler = my_handler;
   sigemptyset(&sigIntHandler.sa_mask);
   sigIntHandler.sa_flags = 0;

   sigaction(SIGINT, &sigIntHandler, NULL);

   /*pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), pCloud_blob (new pcl::PCLPointCloud2);
   pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

   // Fill in the cloud data
   pcl::PCDReader reader;
   reader.read ("table_scene_lms400.pcd", *cloud_blob);

   std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

   // Create the filtering object: downsample the dataset using a leaf size of 1cm
   pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
   sor.setInputCloud (cloud_blob);
   sor.setLeafSize (0.01f, 0.01f, 0.01f);
   sor.filter (*pCloud_blob);

   // Convert to the templated PointCloud
   pcl::fromPCLPointCloud2 (*pCloud_blob, *pCloud);

   std::cerr << "PointCloud after filtering: " << pCloud->width * pCloud->height << " data points." << std::endl;
*/
   viewer = pclVis(new pcl::visualization::PCLVisualizer ("3D Viewer"));
   viewer->setSize(720,480);
   //viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
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
   
   //shmReader.data->p_end = true;

   /*while (!viewer->wasStopped ())
   {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   }*/

   viewer->close();

   return (0);
}