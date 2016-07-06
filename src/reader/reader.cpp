#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <math.h>
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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

using namespace Eigen;

typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> pclVis;

bool endProgram;
pclVis viewer;
MatrixXf csvMat, poseList;
std::string folderPath;

int orderId;
pcl::PointXYZ landingPoint;

ShmReader shmReader;

enum constants {DOWNSAMPLE_RATIO=4, LEFT_ARROW=37, UP_ARROW, RIGHT_ARROW, DOWN_ARROW};
enum droneMovement { EMPTY, DRONE_TAKEOFF, DRONE_LAND_ON, DRONE_INIT_POSE, DRONE_SETPOSE,
                     DRONE_ROTATE_LEFT, DRONE_ROTATE_RIGHT, DRONE_UP, DRONE_DOWN, DRONE_LEFT, DRONE_RIGTH, DRONE_FORWARD, DRONE_BACK};

   int conv[][3] = {
      {0,1,0},
      {2,0,2},
      {2,1,2},
      {1,2,1},
      {1,0,1},
      {0,2,0},
      {0,1,2},
      {1,2,0},
      {2,0,1},
      {0,2,1},
      {2,1,0},
      {1,0,2}
   };

Vector3f quaternionToEuler ( Quaternionf quat, int i0=0, int i1=1, int i2=0 ) {
   return quat.toRotationMatrix().eulerAngles(i0,i1,i2);
}

Vector3f q2e(Quaternionf qu) {

   Vector3f vec;
   qu.normalize();

   float sqw;
   float sqx;
   float sqy;
   float sqz;

   sqw = qu.w() * qu.w();
   sqx = qu.x() * qu.x();
   sqy = qu.y() * qu.y();
   sqz = qu.z() * qu.z();

   // Roll
   vec(0) = atan2(2 * (qu.y()*qu.z() + qu.w()*qu.x()),
        sqw - sqx - sqy + sqz);

   // Pitch
   float sarg = -2 * (qu.x()*qu.z() - qu.w() * qu.y());
   vec(1) = sarg <= -1.0 ? -0.5*M_PI :
      (sarg >= 1.0 ? 0.5*M_PI : asin(sarg));

   // Yaw
   vec(2) = atan2(2 * (qu.x()*qu.y() + qu.w()*qu.z()),
        sqw + sqx - sqy - sqz);

   return vec;

}

int getCorrespondingImage (MatrixXf &poseList, VectorXf &pose) {
   int res = -1;

   /*Vector3f rot = pose.tail<3>();
   cout << rot << endl;
   Matrix3f rotList = poseList.rightCols(3);
   cout << rotList << endl;*/
   // VectorXf prod = (poseList.rightCols(3)) * (pose.tail<3>());
   //cout << prod.transpose() << endl;
   // cout << "Angle: " << 360/M_PI*acos(prod(0)) << endl;

   float dist;
   float minDist = 100;
   float COS_THRESH = 0.965926;
   for (int i = 0; i < poseList.rows(); ++i)
   {
      // if (prod(i) > COS_THRESH) {
         dist = (poseList.row(i).leftCols(3) - pose.head<3>().transpose()).squaredNorm();
         if (dist < minDist) {
            dist = minDist;
            res = i;
         }
      // }
   }

   return res;
}

void readCSV (std::string fname, MatrixXf &mat) {
   std::ifstream file(fname.c_str());

   int lineCount=0;
   std::string line, val;
   while (getline(file, line)) {

      int word=0;
      std::stringstream linestream(line);
      std::vector<float> vec;
      unsigned long time_ms, init_ms;

      while (getline(linestream, val, ',')) {
         std::istringstream ss(val);
         float f;

         if(word==1){
            char* nullChr;
            time_ms = std::strtol(val.c_str(),&nullChr,10);
            if(lineCount==0)
               init_ms = time_ms;

            f = (float)(time_ms-init_ms)/1000;
         }
         else
            ss >> f;

         vec.push_back(f);

         word++;
      }

      Map<RowVectorXf> row(vec.data(), vec.size());
      mat.conservativeResize(mat.rows()+1, vec.size());
      mat.row(mat.rows()-1) = row;

      lineCount++;
   }
}

float distanceTo( Vector4f p ) {
   Vector3f pt (1, 1, -(p(0)*1+p(1)*1+p(3))/p(2) );
   Vector3f n( p.head<3>() );

   return std::abs( n.dot(pt) );
}

float distance2f( const cv::Point2f &a, const cv::Point2f &b )
{
   return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

void largestEmptyCircle( cv::Rect &rect, std::vector<cv::Point2f> &points, float &lec_radius, cv::Point2f &lec_center )
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

void doSmth(float* pcd, float* _pose) {

   pcl::PointCloud<pcl::PointXYZ>::Ptr unfilteredCloud (new pcl::PointCloud<pcl::PointXYZ>);

   VectorXf pose = Map<VectorXf>(_pose, 7);
   Quaternionf rot(_pose+3);
   // cout << "RotMat:\n" << rot.toRotationMatrix() << endl;
   //pose.segment(3,3) = q2e(rot);
   //pose = pose.topRows(6);
//
   // int matchedIndex = getCorrespondingImage(poseList, pose);
   MatrixXf::Index minIdx;
   float min = (csvMat.col(1).array()-shmReader.data->currentTime).abs().minCoeff(&minIdx);
   int matchedIndex = (int) minIdx;;

   std::cout << "Matched: " << matchedIndex << std::endl;
   if (matchedIndex >= 0) {
     std::stringstream ss;
     ss << folderPath << "/image_" << matchedIndex << ".jpg";
     cv::Mat photo;
     photo = cv::imread(ss.str(), CV_LOAD_IMAGE_COLOR);
     cv::namedWindow( "RGB Correspondence", cv::WINDOW_AUTOSIZE );   // Create a window for display.
     cv::imshow( "RGB Correspondence", photo ); 
     cv::waitKey(1);
   }


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

      ptXYZ.x = -pcd[i];
      ptXYZ.y = pcd[i+1];
      ptXYZ.z = pcd[i+2];

      unfilteredCloud->points.push_back(ptXYZ);
   }

   pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::VoxelGrid<pcl::PointXYZ> sor;
   sor.setInputCloud (unfilteredCloud);
   sor.setLeafSize (0.01f, 0.01f, 0.01f);
   sor.filter (*pCloud);

   // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
   Matrix4f rotMat = Matrix4f::Identity();
   Matrix3f coordSysRotate;
   coordSysRotate << 0, 0, 1,
                     1, 0, 0,
                     0, 1, 0;

   // Revert quadcopter rotation. But before, change the coordinate system so that rotation will be done around correct axis. Then change coordinate system to original.
   rotMat.topLeftCorner<3,3>() = coordSysRotate.transpose()*rot.toRotationMatrix()*coordSysRotate;

   // Apply transformation   =>  C'.R.C.PointCloud
   pcl::transformPointCloud (*pCloud, *pCloud, rotMat);

   pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedPlane (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

   // Create the segmentation object
   pcl::SACSegmentation<pcl::PointXYZ> seg;
   // Mandatory
   seg.setModelType (pcl::SACMODEL_PLANE);
   seg.setMethodType (pcl::SAC_RANSAC);
   seg.setDistanceThreshold (0.05);


   int totalNumOfPoints = pCloud->points.size();
   int palette[][3] = {
                        {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {255, 0, 255}, {0, 255, 255}, {128, 0, 0}, {0, 128, 0}, {0, 0, 128}, {128, 128, 0}, {128, 0, 128}, {0, 128, 128}, {192, 0, 0}, {0, 192, 0}, {0, 0, 192}, {192, 192, 0}, {192, 0, 192}, {0, 192, 192}, {64, 0, 0}, {0, 64, 0}, {0, 0, 64}, {64, 64, 0}, {64, 0, 64}, {0, 64, 64}, {32, 0, 0}, {0, 32, 0}, {0, 0, 32}, {32, 32, 0}, {32, 0, 32}, {0, 32, 32}, {96, 0, 0}, {0, 96, 0}, {0, 0, 96}, {96, 96, 0}, {96, 0, 96}, {0, 96, 96}, {160, 0, 0}, {0, 160, 0}, {0, 0, 160}, {160, 160, 0}, {160, 0, 160}, {0, 160, 160}, {224, 0, 0}, {0, 224, 0}, {0, 0, 224}, {224, 224, 0}, {224, 0, 224}, {0, 224, 224}
                     };
   int planeCount=0;
   
   std::vector< Vector4f > planes;
   std::vector<float> planeDistances;
   std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > planePoints;

   // Create the filtering object
   pcl::ExtractIndices<pcl::PointXYZ> extract;
   // std::cout << "Before0: " << planeCount << std::endl;
   while ( pCloud->points.size() > 100/*totalNumOfPoints*0.1*/ )
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
      
      //std::cout << "width: " << segmentedPlane->width << std::endl;
      planes.push_back( Vector4f(coefficients->values.data()) );
      planeDistances.push_back( distanceTo(planes.back()) );
      planePoints.push_back( segmentedPlane->makeShared() );

      extract.setNegative (true);
      extract.filter (*pCloud);
      planeCount++;
   }
   // std::cout << "Before1: " << planeCount << std::endl;
   if (planeCount < 1)
      return;

   viewer->removeAllPointClouds();
   viewer->removeAllShapes();

   // Merge planes
   for (int i = 0; i < planes.size()-1; ++i) {
      for (int j = i+1; j < planes.size(); ++j) {
         Vector4f p1 = planes[i];
         Vector4f p2 = planes[j];
         if ( std::abs(p1.head<3>().dot(p2.head<3>())) > 0.985 ) {  // Check orientation. Appx less than 10 degree
            if ( std::abs(p1(3)-p2(3)) < 0.05 ) {  // Check distance. Less than 5cm

               if (planePoints[i]->width < planePoints[j]->width ) {
                  planeDistances[i] = planeDistances[j];
                  planes[i] = planes[j];
               }
               *(planePoints[i]) += *(planePoints[j]);

               planes.erase(planes.begin()+j);
               planePoints.erase(planePoints.begin()+j);
               planeDistances.erase(planeDistances.begin()+j);

            }
         }
      }
   }
   // std::cout << "After: " << planes.size() << std::endl;

   std::vector<int> hIndices;
   Vector3f worldNormal(0,1,0);
   float COS_VALUE=0.985;

   for (int i = 0; i < planes.size(); ++i)
   {

      // Visualize merged planes
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(planePoints[i], palette[i][0], palette[i][1], palette[i][2]);
      std::stringstream ss;
      ss << "cloud_" << i;
      viewer->addPointCloud<pcl::PointXYZ> (planePoints[i], single_color, ss.str().c_str());

      // Find horizontal planes
      Vector3f normal( planes[i].head<3>() );
      // std::cout << "Normal of " << i << ":" << normal.transpose() << std::endl;
      if ( std::abs( normal.dot(worldNormal) ) > COS_VALUE )
         hIndices.push_back(i);

      //std::cout << "Model coefficients: " << planes[i](0) << " " 
      //                                    << planes[i](1) << " "
      //                                    << planes[i](2) << " "
      //                                    << planes[i](3) << " "
      //             "Distance: "           << planeDistances[i] << std::endl;
      
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

   //std::cout << "Lower plane index: " << lowerPlane << std::endl;
   // Landing area detection
   float largestRadi=0;
   // get the uppermost horizontal plane first
   for (std::vector<int>::const_iterator cIdx = hIndices.begin(); cIdx != hIndices.end(); ++cIdx) {

      int currentPlaneIndex = *cIdx;
      if(currentPlaneIndex != lowerPlane) {

         pcl::PointCloud<pcl::PointXYZ>::Ptr currentPlane = planePoints[currentPlaneIndex];
         
         // Create binary image 

         float maxX=0, maxZ=0, minX=FLT_MAX, minZ=FLT_MAX;
         for (int i=0; i<currentPlane->points.size(); i++)
         {
            pcl::PointXYZ* pt = &currentPlane->points[i];
            if(pt->x > maxX)
               maxX = pt->x;
            if(pt->z > maxZ)
               maxZ = pt->z;
            if(pt->x < minX)
               minX = pt->x;
            if(pt->z < minZ)
               minZ = pt->z;
         }

         int xGrid=256;
         float gridSize=(maxX-minX)/xGrid;
         int zGrid = ceil((maxZ-minZ)/gridSize);
         cv::Mat binImg = cv::Mat::zeros(zGrid, xGrid, CV_8UC1);

         for (int i=0; i<currentPlane->points.size(); i++)
         {
            pcl::PointXYZ* pt = &currentPlane->points[i];
            int xInd = (pt->x - minX) / gridSize;
            int yInd = (pt->z - minZ) / gridSize;
            if (xInd < xGrid && yInd < zGrid) {
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
         // cv::Mat outImg;
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
            cv::Mat locations, edges, bordered;
            cv::copyMakeBorder(labels, bordered, 1, 1, 1, 1, cv::BORDER_CONSTANT, 0);    // Add 1px border to edge image to prevent centers to form at the edges of the image
            cv::Canny(bordered == i, edges, 50, 150);    // Find edge image
            cv::findNonZero(edges, locations);     // Get edge elements to locations matrix

            std::vector<cv::Point2f> points;
            locations.copyTo(points);
            // if (i==1)
            //    outImg = cv::Mat::zeros(edges.size(), CV_8UC3);
            // outImg.setTo(cv::Scalar(palette[i][0],palette[i][1],palette[i][2]), edges);   // Draw painted edges using edge image as mask

            float radi;
            cv::Point2f center;
            /*cv::Rect rect(stats.at<int>(i,0),
                                         stats.at<int>(i,1),
                                         stats.at<int>(i,2),
                                         stats.at<int>(i,3));*/
            cv::Rect rect(0, 0, edges.cols, edges.rows);                         
            largestEmptyCircle( rect, points, radi, center );

            // Correction for manually added border on the edge image
            center.x = center.x-1;
            center.y = center.y-1;
            if (center.x < 0) center.x = 0;
            if (center.y < 0) center.y = 0;
            if (center.x >= xGrid) center.x = xGrid-1;
            if (center.y >= zGrid) center.y = zGrid-1;

            if(radi > largestRadi) {
               largestRadi = radi;
            
               float x = center.x*gridSize + minX;
               float z = center.y*gridSize + minZ;
               float y = -(planes[currentPlaneIndex](0)*x
                        +  planes[currentPlaneIndex](2)*z
                        +  planes[currentPlaneIndex](3)) 
                        /  planes[currentPlaneIndex](1);

               landingPoint.x = x;
               landingPoint.y = y;
               landingPoint.z = z;
            }

            // circle(outImg, center, 2, cv::Scalar(255, 255, 255));
            // circle(outImg, center, radi, cv::Scalar(255, 255, 255), 2);
         }

         // cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
         // cv::imshow("Display window", outImg);
         // cv::waitKey(1);
      }
   }

   float minRadiToLand = 1;
   if (largestRadi > minRadiToLand) {
      viewer->addSphere(landingPoint, 0.05, 255, 255, 255);
   }
   //viewer->addCoordinateSystem (1.0);
   viewer->spinOnce();
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
   std::cout << "keycode=" << event.getKeyCode ();
   if (event.keyDown ()) {
      switch (event.getKeyCode ()) {
         case 't': orderId = DRONE_TAKEOFF;     break;
         case 'l': orderId = DRONE_LAND_ON;     break;
         case 'w': orderId = DRONE_FORWARD;     break;
         case 's': orderId = DRONE_BACK;        break;
         case 'a': orderId = DRONE_LEFT;        break;
         case 'd': orderId = DRONE_RIGTH;       break;
         case '8': orderId = DRONE_UP;          break;
         case '2': orderId = DRONE_DOWN;        break;
         case '4': orderId = DRONE_ROTATE_LEFT; break;
         case '6': orderId = DRONE_ROTATE_RIGHT;break;
         case 'i': orderId = DRONE_INIT_POSE;   break;
      }
   }
}

void my_handler(int s){
   endProgram = true;
}

void handleOrder(int _orderId, float* _pose = NULL) {

   float* order = shmReader.data->targetPose;
   bool& newOrder = shmReader.data->newOrder;

   newOrder = true;

   // order array is all zeros here.
   Quaternionf qqq(shmReader.data->pose+3);
   Vector3f target = Vector3f(0,0,0);

   switch (_orderId) {

      case DRONE_TAKEOFF:
         target = Vector3f(0, 0, 2 - shmReader.data->pose[2]);
         break;
      case DRONE_LAND_ON:
         target = Vector3f(landingPoint.z, landingPoint.x, landingPoint.y);
         break;
      case DRONE_ROTATE_LEFT:
         order[5] = M_PI/4;
         break;
      case DRONE_ROTATE_RIGHT:
         order[5] = -M_PI/4;
         break;
      case DRONE_UP:
         target = Vector3f(0, 0, 0.2);
         break;
      case DRONE_DOWN:
         target = Vector3f(0, 0, -0.2);
         break;
      case DRONE_FORWARD:
         target = Vector3f(0.2, 0, 0);
         break;
      case DRONE_BACK:
         target = Vector3f(-0.2, 0, 0);
         break;
      case DRONE_RIGTH:
         target = Vector3f(0, -0.2, 0);
         break;
      case DRONE_LEFT:
         target = Vector3f(0, 0.2, 0);
         break;
      case DRONE_INIT_POSE:   // Not working
         order[3] = poseList.row(0)(3);
         order[4] = poseList.row(0)(4);
         order[5] = poseList.row(0)(5);
         std::cout << "Init pose set" << std::endl;
         break;
      case DRONE_SETPOSE:
         target = Vector3f(_pose[0] - shmReader.data->pose[0],
                           _pose[1] - shmReader.data->pose[1],
                           _pose[2] - shmReader.data->pose[2]);
         //order[3] = _pose[3] - shmReader.data->pose[3];
         //order[4] = _pose[4] - shmReader.data->pose[4];
         //order[5] = _pose[5] - shmReader.data->pose[5];
         break;
      default: newOrder = false;
      break;
   }

   target = qqq.toRotationMatrix()*target;
   order[0] = target(0);
   order[1] = target(1);
   order[2] = target(2);

   orderId = EMPTY;
}


int
main (int argc, char** argv)
{
   struct sigaction sigIntHandler;

   sigIntHandler.sa_handler = my_handler;
   sigemptyset(&sigIntHandler.sa_mask);
   sigIntHandler.sa_flags = 0;

   sigaction(SIGINT, &sigIntHandler, NULL);

   folderPath = argv[1];

   readCSV(folderPath+".pose", csvMat);

   poseList = csvMat.rightCols(7);
   // Convert quaternion to rotations
   for (int i=0; i<poseList.rows(); i++) {
      Quaternionf q( poseList.row(i)(6),
                     poseList.row(i)(3),
                     poseList.row(i)(4),
                     poseList.row(i)(5));
      Vector3f rot = q2e(q);
      poseList(i,3) = rot(0);
      poseList(i,4) = rot(1);
      poseList(i,5) = rot(2);
   }
   poseList = poseList.leftCols(6);

   viewer = pclVis(new pcl::visualization::PCLVisualizer ("3D Viewer"));
   viewer->setSize(640,480);
   viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
   viewer->setBackgroundColor (0, 0, 0);
   viewer->initCameraParameters ();
   // viewer->addCoordinateSystem (1.0);
   
   // std::clock_t prev_c = std::clock(), curr_c;
   // ((curr_c - prev_c) / (double)(CLOCKS_PER_SEC / 1000) >= 1000 && iter < poseList.rows())

   // Set animation
   shmReader.read();
   int dim = 4;
   for (int i=0; i<poseList.rows(); i++) {
      shmReader.data->keyFrames[dim*i+0] = csvMat.row(i)(1);
      shmReader.data->keyFrames[dim*i+1] = csvMat.row(i)(3);
      shmReader.data->keyFrames[dim*i+2] = -csvMat.row(i)(2);
      shmReader.data->keyFrames[dim*i+3] = csvMat.row(i)(4)+1.2;
   }
   shmReader.data->frameCount = poseList.rows();
   shmReader.data->startAnimationSignal = true;
   shmReader.releaseMem();
   
   // for (int i = 0; i < 10; ++i)
   // {
   //    cout << i << ". " << (long) csvMat(i,1) << endl;
   // }


   while (!endProgram)
   { 
      shmReader.read();

      if (shmReader.data->p_end){   // Check server if exits
         endProgram = true;
      }
      else {
         doSmth(shmReader.data->buffer, shmReader.data->pose);

         // Handle orders
         handleOrder(orderId);
      }

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