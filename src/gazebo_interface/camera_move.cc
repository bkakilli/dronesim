#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class CameraMove : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // create the animation
      gazebo::common::PoseAnimationPtr anim(
            // name the animation "test",
            // make it last 10 seconds,
            // and set it on a repeat loop
            new gazebo::common::PoseAnimation("test", 20.0, true));

      gazebo::common::PoseKeyFrame *key;

      // set starting location of the box
      key = anim->CreateKeyFrame(0);
      key->Translation(ignition::math::Vector3d(5, 0, 1));
      key->Rotation(ignition::math::Quaterniond(0, 0, -3.1429));

      // set waypoint location after 2 seconds
      key = anim->CreateKeyFrame(5.0);
      key->Translation(ignition::math::Vector3d(0, 5, 1));
      key->Rotation(ignition::math::Quaterniond(0, 0, -1.5707));


      key = anim->CreateKeyFrame(10.0);
      key->Translation(ignition::math::Vector3d(-5, 0, 1));
      key->Rotation(ignition::math::Quaterniond(0, 0, 0));


      key = anim->CreateKeyFrame(15.0);
      key->Translation(ignition::math::Vector3d(0, -5, 1));
      key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));


      key = anim->CreateKeyFrame(20.0);
      key->Translation(ignition::math::Vector3d(5, 0, 1));
      key->Rotation(ignition::math::Quaterniond(0, 0, 3.1429));

      // set final location equal to starting location
      /*key = anim->CreateKeyFrame(10);
      key->Translation(ignition::math::Vector3d(0, 0, 0));
      key->Rotation(ignition::math::Quaterniond(0, 0, 0));*/

      // set the animation
      //this->model->SetAnimation(anim);

    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CameraMove)
}