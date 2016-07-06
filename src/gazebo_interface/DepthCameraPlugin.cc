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
  std::cout << "Terminating" << std::endl;
  shmWriter.data->p_end = true;
  shmWriter.data->cond.notify_all();

  this->parentSensor.reset();
  this->depthCamera.reset();
}

void DepthCameraPlugin::posePointsToAnimation(physics::ModelPtr &_model, float* posePoints, int pointCount) {
  // timestamp(ms) - x - y - z - r - p - y
  int dim = 4;
  math::Vector3     pos = _model->GetRelativePose().pos;
  math::Quaternion  rot = _model->GetRelativePose().rot;

  double duration = posePoints[dim*pointCount-dim]-posePoints[0];
  // create the animation
  anim = gazebo::common::PoseAnimationPtr(
        new gazebo::common::PoseAnimation("newanim", duration, false));

  common::PoseKeyFrame *key;

  for (int i = 0; i < pointCount; ++i)
  {
    key = anim->CreateKeyFrame(posePoints[dim*i]);
    key->Translation(ignition::math::Vector3d(posePoints[dim*i+1], posePoints[dim*i+2], posePoints[dim*i+3]));
    key->Rotation(ignition::math::Quaterniond(rot.w, rot.x, rot.y, rot.z));
  }

  _model->SetAnimation(anim);
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

  printf("w:%d h:%d\n", width, height);


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
  

  physics::WorldPtr world = physics::get_world(this->parentSensor->WorldName());
  quadcopter = world->GetModel("quadcopter_model");

  count = 0;


  /*math::Vector3 vec = this->parentSensor->Pose().Rot().Euler();
  std::cout << "x:" << vec.x << " y:" << vec.y << " z:" << vec.z << std::endl;
  std::cout << "Parent:" << this->parentSensor->ParentName() << std::endl;

  vec = cameraModel->GetRelativePose().rot.GetAsEuler();
  std::cout << "x:" << vec.x << " y:" << vec.y << " z:" << vec.z << std::endl;*/
  
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
bool save = true;
/////////////////////////////////////////////////
void DepthCameraPlugin::OnNewRGBPointCloud(const float * _pcd,
                unsigned int _width, unsigned int _height,
                unsigned int /*_depth*/, const std::string & _format)
{
  math::Vector3     pos = quadcopter->GetRelativePose().pos;
  math::Quaternion  rot = quadcopter->GetRelativePose().rot;
  //std::cout << "x:" << pos.x << " y:" << pos.y << " z:" << pos.z << std::endl;
  //std::cout << "w:" << rot.w << " x:" << rot.x << " y:" << rot.y << " z:" << rot.z << std::endl;
  // std::cout << "X:" << rot.GetXAxis() << "\nY:" << rot.GetYAxis() << "\nZ:" << rot.GetZAxis() << std::endl;
  // std::cout << "RotMat:\n" << rot.GetAsMatrix3() << std::endl;
  // std::cout << "Inv:\n" << rot.GetAsMatrix3().Inverse() << std::endl;
  float pose[7] = { (float)pos.x, 
                    (float)pos.y, 
                    (float)pos.z, 
                    (float)rot.x, 
                    (float)rot.y, 
                    (float)rot.z, 
                    (float)rot.w};

  // ignition::math::Pose3d sensorPose = this->parentSensor->Pose();
  // cout << "Sensor position: " << sensorPose.Pos() << endl;
  // cout << "Sensor rotation: " << sensorPose.Rot() << endl;
  // math::Vector3 eul = rot.GetAsEuler();
  // std::cout << eul.x << "\t" << eul.y << "\t" << eul.z << std::endl;

  if(!shmWriter.data->reading){

    if (shmWriter.data->newOrder) {
         std::cout << "new order" << std::endl;

      quadcopter->StopAnimation();

      float* order = shmWriter.data->targetPose;
      double duration = std::sqrt(order[0]*order[0] + order[1]*order[1] + order[2]*order[2]);
      if (duration == 0)
        duration = 0.1;

        // create the animation
      anim = gazebo::common::PoseAnimationPtr(
            new gazebo::common::PoseAnimation("newanim", duration, false));

      gazebo::common::PoseKeyFrame *key;

      // set starting location of the box
      key = anim->CreateKeyFrame(0);
      key->Translation(ignition::math::Vector3d(pos.x, pos.y, pos.z));
      key->Rotation(ignition::math::Quaterniond(rot.w, rot.x, rot.y, rot.z));

      key = anim->CreateKeyFrame(duration);
      key->Translation(ignition::math::Vector3d(pos.x+order[0], pos.y+order[1], pos.z+order[2]));
      key->Rotation(ignition::math::Quaterniond(rot.GetRoll()+order[3], rot.GetPitch()+order[4], rot.GetYaw()+order[5]));

      for (int i = 0; i < 7; ++i)
        order[i] = 0;
      shmWriter.data->newOrder = false;

      quadcopter->SetAnimation(anim);
    }

    if (shmWriter.data->startAnimationSignal) {
      posePointsToAnimation(quadcopter, shmWriter.data->keyFrames, shmWriter.data->frameCount);
      shmWriter.data->startAnimationSignal = false;
    }

    if (anim)
      shmWriter.data->currentTime = anim->GetTime();
    memcpy ( shmWriter.data->buffer, _pcd, trace_queue::BUFFERSIZE*4 );
    memcpy ( shmWriter.data->pose, pose, 7*4 );
    shmWriter.data->cond.notify_all();
  }

  if(shmWriter.data->p_end) {
    raise(SIGINT);
  }

  if (save) {

    ofstream myfile;
    myfile.open ("example.pcd");
    
    myfile << "VERSION .7\n"
           << "FIELDS x y z\n"
           << "SIZE 4 4 4\n"
           << "TYPE F F F\n"
           << "COUNT 1 1 1\n"
           << "WIDTH " << _width << "\n"
           << "HEIGHT " << _height << "\n"
           << "VIEWPOINT " << pose[0] << " "
                           << pose[1] << " "
                           << pose[2] << " "
                           << pose[3] << " "
                           << pose[4] << " "
                           << pose[5] << " "
                           << pose[6] << "\n"
           << "POINTS " << _width*_height << "\n"
           << "DATA ascii\n";

    for (int i = 0; i < 4*_width*_height; i=i+4) {
      myfile  << _pcd[i] << " "
              << _pcd[i+1] << " "
              << _pcd[i+2] << "\n";
    }
    /*for (int i = 0; i < 4*pcdSize; i=i+4) {
      myfile  << _pcd[i] << " "
              << _pcd[i+1] << " "
              << _pcd[i+2] << "\n";
    }*/
    myfile.close();
    // save = false;
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