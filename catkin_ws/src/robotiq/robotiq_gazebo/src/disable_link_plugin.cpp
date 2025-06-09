/*********************************************************************
 * Software License Agreement (BSD License)
 * Copyright (c) 2014, Konstantinos Chatzilygeroudis
 * Copyright (c) 2016, CRI Lab at Nanyang Technological University
 * All rights reserved.
 *********************************************************************/

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <ros/ros.h>

namespace gazebo
{

class DisableLinkPlugin : public ModelPlugin
{
public:
  DisableLinkPlugin() : ModelPlugin() {}

  virtual ~DisableLinkPlugin() {}

  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    model_ = _parent;
    world_ = model_->GetWorld();

    if (!_sdf->HasElement("link"))
    {
      ROS_ERROR_STREAM("DisableLinkPlugin: <link> element is missing in SDF. Plugin not loaded.");
      return;
    }

    link_name_ = _sdf->Get<std::string>("link");

    link_ = model_->GetLink(link_name_);
    if (!link_)
    {
      ROS_WARN_STREAM("DisableLinkPlugin: Link [" << link_name_ << "] not found in model [" 
                       << model_->GetName() << "].");
      return;
    }

    link_->SetEnabled(false);
    ROS_INFO_STREAM("DisableLinkPlugin: Link [" << link_name_ << "] disabled.");
  }

private:
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  std::string link_name_;
};

// Register plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(DisableLinkPlugin)

} // namespace gazebo
