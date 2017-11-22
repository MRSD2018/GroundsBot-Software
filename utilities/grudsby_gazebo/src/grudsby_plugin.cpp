#ifndef _GRUDSBY_PLUGIN_HH
#define _GRUDSBY_PLUGIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
class GrudsbyPlugin : public ModelPlugin
{
    /// \brief Constructor
  private:
    physics::ModelPtr model;
    physics::JointPtr left_joint;
    physics::JointPtr right_joint;
    
    transport::NodePtr node;
    transport::SubscriberPtr sub;

  public:
    GrudsbyPlugin() {}

  public:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Safety check
        if (_model->GetJointCount() == 0)
        {
            std::cerr << "Invalid joint count, plugin not loaded \n";
            return;
        }

        //store model pointer
        this->model = _model;

        //get wheel joints
        this->left_joint = _model->GetJoint("grudsby_new::left_axle_joint");
        this->right_joint = model->GetJoint("grudsby_new::right_axle_joint");
        // this->left_joint = _model->GetJoints()[0];
        // std::cerr << this->left_joint->GetName();
        // this->right_joint = _model->GetJoints()[1];
        // std::cerr << this->right_joint->GetName();

        double left_velocity = 0;
        double right_velocity = 0;

        //create transport nodes (gazebo)
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init(this->model->GetWorld()->GetName());

        //create a topic name (gazebo)
        std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

        //subscribe to topic, register callback
        this->sub = this->node->Subscribe(topicName, &GrudsbyPlugin::OnMsg, this);
    }

    void SetVelocity(const double &_vel_left, const double &_vel_right)
    {
        this->model->GetJointController()->SetVelocityTarget(
            this->left_joint->GetScopedName(), _vel_left);
        this->model->GetJointController()->SetVelocityTarget(
            this->right_joint->GetScopedName(), _vel_right);
    }

  private:
    void OnMsg(ConstVector3dPtr &_msg)
    {
        this->SetVelocity(_msg->x(), _msg->y());
    }
};

GZ_REGISTER_MODEL_PLUGIN(GrudsbyPlugin)
}
#endif