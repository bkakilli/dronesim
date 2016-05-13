/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <functional>

#include "DepthCameraPlugin.hh"

using namespace gazebo;
using namespace std;
GZ_REGISTER_SENSOR_PLUGIN(DepthCameraPlugin)

/////////////////////////////////////////////////
DepthCameraPlugin::DepthCameraPlugin()
: SensorPlugin(), width(0), height(0), depth(0)
{
}

/////////////////////////////////////////////////
DepthCameraPlugin::~DepthCameraPlugin()
{
  this->parentSensor.reset();
  this->depthCamera.reset();
}

/////////////////////////////////////////////////
void DepthCameraPlugin::Load(sensors::SensorPtr _sensor,
                              sdf::ElementPtr /*_sdf*/)
{
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor);
  this->depthCamera = this->parentSensor->DepthCamera();

  if (!this->parentSensor)
  {
    gzerr << "DepthCameraPlugin not attached to a depthCamera sensor\n";
    return;
  }

  this->width = this->depthCamera->ImageWidth();
  this->height = this->depthCamera->ImageHeight();
  this->depth = this->depthCamera->ImageDepth();
  this->format = this->depthCamera->ImageFormat();

  this->newDepthFrameConnection = this->depthCamera->ConnectNewDepthFrame(
      std::bind(&DepthCameraPlugin::OnNewDepthFrame,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

  this->newRGBPointCloudConnection = this->depthCamera->ConnectNewRGBPointCloud(
      std::bind(&DepthCameraPlugin::OnNewRGBPointCloud,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

  this->newImageFrameConnection = this->depthCamera->ConnectNewImageFrame(
      std::bind(&DepthCameraPlugin::OnNewImageFrame,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void DepthCameraPlugin::OnNewDepthFrame(const float *_image,
    unsigned int _width, unsigned int _height,
    unsigned int /*_depth*/, const std::string &/*_format*/)
{
  float min, max;
  min = 1000;
  max = 0;
  for (unsigned int i = 0; i < _width * _height; i++)
  {
    if (_image[i] > max)
      max = _image[i];
    if (_image[i] < min)
      min = _image[i];
  }

  int index =  ((_height * 0.5) * _width) + _width * 0.5;
  printf("W[%u] H[%u] MidPoint[%d] Dist[%f] Min[%f] Max[%f]\n",
      width, height, index, _image[index], min, max);

  /*rendering::Camera::SaveFrame(_image, this->width,
    this->height, this->depth, this->format,
    "/tmp/depthCamera/me.jpg");
    */
}

/////////////////////////////////////////////////
bool control = true;
void DepthCameraPlugin::OnNewRGBPointCloud(const float * _pcd,
                unsigned int _width, unsigned int _height,
                unsigned int /*_depth*/, const std::string & _format)
{
  
  if(control){

    int pcdSize = _width * _height;
    ofstream myfile;

    myfile.open ("/home/burak/workspace/dronesim/example.pcd");
    
    myfile << "VERSION .7\n"
           << "FIELDS x y z rgb\n"
           << "SIZE 4 4 4 4\n"
           << "TYPE F F F F\n"
           << "COUNT 1 1 1 1\n"
           << "WIDTH " << _width << "\n"
           << "HEIGHT " << _height << "\n"
           << "VIEWPOINT 0 0 0 1 0 0 0\n"
           << "POINTS " << _width*_height << "\n"
           << "DATA ascii\n";

    for (int i = 0; i < 4*pcdSize; i=i+4) {
      myfile  << _pcd[i] << " "
              << _pcd[i+1] << " "
              << _pcd[i+2] << " "
              << _pcd[i+3] << "\n";
    }
    /*for (int i = 0; i < 4*pcdSize; i=i+4) {
      myfile  << _pcd[i] << " "
              << _pcd[i+1] << " "
              << _pcd[i+2] << "\n";
    }*/
    myfile.close();
  }
}

/////////////////////////////////////////////////
void DepthCameraPlugin::OnNewImageFrame(const unsigned char * /*_image*/,
                              unsigned int /*_width*/,
                              unsigned int /*_height*/,
                              unsigned int /*_depth*/,
                              const std::string &/*_format*/)
{
  /*rendering::Camera::SaveFrame(_image, this->width,
    this->height, this->depth, this->format,
    "/tmp/depthCamera/me.jpg");
    */
}