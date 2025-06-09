#include <robotiq_gazebo/mimic_joint_plugin.h>
#include <ignition/math/Helpers.hh>

namespace gazebo
{

MimicJointPlugin::MimicJointPlugin()
{
  kill_sim = false;
  joint_.reset();
  mimic_joint_.reset();
}

MimicJointPlugin::~MimicJointPlugin()
{
  // Desconectar não é mais necessário explicitamente no Gazebo 11.
  this->updateConnection.reset();
  kill_sim = true;
}

void MimicJointPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  ros::NodeHandle model_nh;
  model_ = _parent;
  world_ = model_->GetWorld();

  if (!model_)
  {
    ROS_ERROR("Parent model is NULL! MimicJointPlugin could not be loaded.");
    return;
  }

  if(!ros::isInitialized())
  {
    ROS_ERROR("A ROS node for Gazebo has not been initialized, unable to load plugin.");
    return;
  }

  robot_namespace_ = "/";
  if(_sdf->HasElement("robotNamespace"))
  {
    robot_namespace_ = _sdf->Get<std::string>("robotNamespace");
  }

  if(!_sdf->HasElement("joint"))
  {
    ROS_ERROR("No joint element present. MimicJointPlugin could not be loaded.");
    return;
  }
  joint_name_ = _sdf->Get<std::string>("joint");

  if(!_sdf->HasElement("mimicJoint"))
  {
    ROS_ERROR("No mimicJoint element present. MimicJointPlugin could not be loaded.");
    return;
  }
  mimic_joint_name_ = _sdf->Get<std::string>("mimicJoint");

  has_pid_ = false;
  if(_sdf->HasElement("hasPID"))
  {
    has_pid_ = true;

    const ros::NodeHandle nh(model_nh, robot_namespace_ + "/gazebo_ros_control/pid_gains/" + mimic_joint_name_);
    double p, i, d;
    nh.param("p", p, 0.0);
    nh.param("i", i, 0.0);
    nh.param("d", d, 0.0);

    pid_ = control_toolbox::Pid(p, i, d);
  }

  multiplier_ = _sdf->Get<double>("multiplier", 1.0).first;
  offset_ = _sdf->Get<double>("offset", 0.0).first;
  sensitiveness_ = _sdf->Get<double>("sensitiveness", 0.0).first;
  max_effort_ = _sdf->Get<double>("maxEffort", 1.0).first;

  joint_ = model_->GetJoint(joint_name_);
  if(!joint_)
  {
    ROS_ERROR("No joint named %s. MimicJointPlugin could not be loaded.", joint_name_.c_str());
    return;
  }

  mimic_joint_ = model_->GetJoint(mimic_joint_name_);
  if(!mimic_joint_)
  {
    ROS_ERROR("No (mimic) joint named %s. MimicJointPlugin could not be loaded.", mimic_joint_name_.c_str());
    return;
  }

  if(!has_pid_)
  {
    mimic_joint_->SetParam("fmax", 0, max_effort_);
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&MimicJointPlugin::UpdateChild, this));
}

void MimicJointPlugin::UpdateChild()
{
  double dt = world_->Physics()->GetMaxStepSize();
  ros::Duration period(dt);

  double angle = joint_->Position(0) * multiplier_ + offset_;
  double mimic_angle = mimic_joint_->Position(0);

  if (std::abs(angle - mimic_angle) >= sensitiveness_)
  {
    if(has_pid_)
    {
      double error = angle - mimic_angle;
      double effort = ignition::math::clamp(
          pid_.computeCommand(error, period),
          -max_effort_, max_effort_);

      mimic_joint_->SetForce(0, effort);
    }
    else
    {
      mimic_joint_->SetPosition(0, angle);
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(MimicJointPlugin)

} // namespace gazebo
