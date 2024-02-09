#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>
#include <xarm_msgs/srv/set_joint_angles.hpp>

namespace gazebo
{
  class SetInitialJointAnglesPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      this->node = gazebo_ros::Node::Get(_sdf);
      this->model = _model;
      // Create a service server
      this->service = this->node->create_service<xarm_msgs::srv::SetJointAngles>(
      "/set_joint_angles", std::bind(&SetInitialJointAnglesPlugin::OnSetJointAngles, this, std::placeholders::_1, std::placeholders::_2));
      // Check if the initial_joint_angles element exists
      if (_sdf->HasElement("initial_joint_angles"))
      {
        // Get the initial_joint_angles element
        sdf::ElementPtr jointAnglesElem = _sdf->GetElement("initial_joint_angles");

        // Iterate through all the joint elements
        sdf::ElementPtr jointElem = jointAnglesElem->GetFirstElement();
        while (jointElem)
        {
          // Get the joint name and angle
          std::string jointName = jointElem->Get<std::string>("name");
          double angle = jointElem->Get<double>("angle");

          // Here, you can apply the angle to the joint as required
          _model->GetJoint(jointName)->SetPosition(0, angle);

          // Move to the next joint element
          jointElem = jointElem->GetNextElement();
        }
      }
    }
  private:
      void OnSetJointAngles(const std::shared_ptr<xarm_msgs::srv::SetJointAngles::Request> request,
                     std::shared_ptr<xarm_msgs::srv::SetJointAngles::Response> response)
{
    // Check if the size of joint_names and angles arrays are the same
    if (request->joint_names.size() != request->angles.size())
    {
        RCLCPP_ERROR(this->node->get_logger(), "The size of joint_names and angles arrays must be equal");
        response->success = false;
        return;
    }

    // Iterate over the joint names and angles
    for (size_t i = 0; i < request->joint_names.size(); ++i)
    {
        const std::string& joint_name = request->joint_names[i];
        double angle = request->angles[i];

        // Find and set the joint angle
        gazebo::physics::JointPtr joint = this->model->GetJoint(joint_name);
        if (!joint)
        {
            RCLCPP_ERROR(this->node->get_logger(), "Joint '%s' not found in model", joint_name.c_str());
            response->success = false;       
            return;
        }

        joint->SetPosition(0, angle);  // Assuming single DOF joint
        RCLCPP_DEBUG(this->node->get_logger(), "Joint '%s' set to position %f", joint_name.c_str(), angle);
    }

    response->success = true;
    return;
}
      rclcpp::Node::SharedPtr node;
      physics::ModelPtr model;
      rclcpp::Service<xarm_msgs::srv::SetJointAngles>::SharedPtr service;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SetInitialJointAnglesPlugin)
}
