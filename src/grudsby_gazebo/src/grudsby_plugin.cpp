#ifndef _GRUDSBY_PLUGIN_HH
#define _GRUDSBY_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "ros/ros.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
class GrudsbyPlugin : public ModelPlugin
{
  /// \brief Gazebo model joints
private:
  physics::ModelPtr model;
  physics::JointPtr left_joint;
  physics::JointPtr right_joint;

  /// \brief A pointer to the NodeHandle
private:
  std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief ROS subscribers
private:
  ros::Subscriber leftCmdSub;
  ros::Subscriber rightCmdSub;

  /// \brief Constructor
public:
  GrudsbyPlugin()
  {
  }

public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Safety check
    if (_model->GetJointCount() == 0)
    {
      std::cerr << "Invalid joint count, plugin not loaded \n";
      return;
    }

    // store model pointer
    this->model = _model;

    // get wheel joints
    this->left_joint = _model->GetJoint("grudsby_new::left_axle_joint");
    this->right_joint = model->GetJoint("grudsby_new::right_axle_joint");

    double left_velocity = 0;
    double right_velocity = 0;

    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client",
      ros::init_options::NoSigintHandler);
}

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    // Create a named topic, and subscribe to it.
    this->leftCmdSub = this->rosNode->subscribe("/arduino/lwheel_vtarget", 10, &GrudsbyPlugin::leftCmdCallback, this);
    this->rightCmdSub = this->rosNode->subscribe("/arduino/rwheel_vtarget", 10, &GrudsbyPlugin::rightCmdCallback, this);
  }

private:
  /// \brief Handle left wheel arduino message
  /// \param[in] _msg A float value that is used to set the velocity of the left wheel
  void leftCmdCallback(const std_msgs::Float32::ConstPtr& _msg)
  {
    this->model->GetJointController()->SetVelocityTarget(this->left_joint->GetScopedName(), _msg->data);
  }
  /// \brief Handle right wheel arduino message
  /// \param[in] _msg A float value that is used to set the velocity of the right wheel
  void rightCmdCallback(const std_msgs::Float32::ConstPtr& _msg)
  {
    this->model->GetJointController()->SetVelocityTarget(this->right_joint->GetScopedName(), _msg->data);
  }
};

GZ_REGISTER_MODEL_PLUGIN(GrudsbyPlugin)
}
#endif