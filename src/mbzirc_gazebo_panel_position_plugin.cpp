/*
 * Copyright (c) 2016, JSK Robotics Laboratory, The University of Tokyo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <mbzirc_gazebo_panel_position_plugin.h>
#include <string>
#include <algorithm>    // std::random_shuffle
#include <ignition/math/Pose3.hh>

double randomDouble(double min, double max)
{
  return ((double) random()/RAND_MAX)*(max-min) + min;
}

namespace gazebo
{

GazeboPanelPosition::GazeboPanelPosition()
{
}

GazeboPanelPosition::~GazeboPanelPosition()
{
  event::Events::DisconnectWorldUpdateBegin(update_connection_);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboPanelPosition::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gzwarn << "The GazeboPanelPosition plugin is DEPRECATED in ROS hydro." << std::endl;

  model_ = _model;
  world_ = _model->GetWorld();
  link_ = _model->GetLink();
  link_name_ = link_->GetName();
  namespace_.clear();

  terminated_ = false;

  // load parameters from sdf
  if (_sdf->HasElement("robotNamespace")) namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();

  if (_sdf->HasElement("bodyName") && _sdf->GetElement("bodyName")->GetValue())
    {
      link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
      link_ = _model->GetLink(link_name_);
    }

  if (!link)
  {
    ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }


  if (!_sdf->HasElement("randomize_wrenches")) {
    ROS_INFO("Panel position plugin missing <randomize_wrenches>, defaults to false");
    is_random_wrenches_ = -1;
  } else {
    is_random_wrenches_ = _sdf->GetElement("randomize_wrenches")->Get<int>();
  }

  if (!_sdf->HasElement("randomize_location")) {
    ROS_INFO("Panel position plugin missing <randomize_location>, defaults to false");
    is_random_location_ = -1;
  } else {
    is_random_location_ = _sdf->GetElement("randomize_location")->Get<int>();
  }


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPanelPosition::Update, this));
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboPanelPosition::Update()
{
  //boost::mutex::scoped_lock scoped_lock(lock);

  if ( terminated_ )
    {
      return;
    }


  double x, y, yaw;
  if (is_random_location_)
  {
    // Set pose
    double r,a;
    r = randomDouble(MIN_RANGE, MAX_RANGE);
    a = randomDouble(-M_PI, 0);
    yaw = randomDouble(0, 2*M_PI);

    x = r*cos(a) + STARTING_ZONE_X;
    y = r*sin(a) + STARTING_ZONE_Y;

    model_->SetLinkWorldPose(math::Pose(x, y, PANEL_Z, 0, 0, yaw), link_);
  }
  else
  {
    // Read current pose
    math::Pose p;
    p = model_->GetWorldPose();
    x = p.pos.x;
    y = p.pos.y;
    yaw = p.rot.GetYaw();
  }

  // === Insert wrenches ===
  // Create a new transport node
  transport::NodePtr node(new transport::Node());
  node->Init(world_->GetName());
  transport::PublisherPtr factoryPub = node->Advertise<msgs::Factory>("~/factory");

  msgs::Factory msg;

  // Randomize wrenches
  static const double arr[] = {0.20,0.25,0.30,0.35,0.40,0.45};
  std::vector<double> vec (arr, arr + sizeof(arr) / sizeof(arr[0]) );

  if (is_random_wrenches_)
  {
    std::random_shuffle ( vec.begin(), vec.end() );
  }

  // Insert wrenches
  InsertValveStem("model://stem", x, y, 0.0225, -0.155, yaw);
  InsertWrench("model://wrench12", x, y, 0.165, vec[0], yaw);
  InsertWrench("model://wrench13", x, y, 0.165, vec[1], yaw);
  InsertWrench("model://wrench14", x, y, 0.165, vec[2], yaw);
  InsertWrench("model://wrench15", x, y, 0.165, vec[3], yaw);
  InsertWrench("model://wrench18", x, y, 0.165, vec[4], yaw);
  InsertWrench("model://wrench19", x, y, 0.165, vec[5], yaw);

  terminated_ = true;
}


void GazeboPanelPosition::InsertWrench(std::string model_name, double x, double y, double x_offset, double y_offset, double yaw)
{
  // Create a new transport node
  transport::NodePtr node(new transport::Node());
  node->Init(world_->GetName());
  transport::PublisherPtr factoryPub = node->Advertise<msgs::Factory>("~/factory");

  msgs::Factory msg;
  msg.set_sdf_filename(model_name);

  // Calculate wrench position
  double x_inc = x_offset*cos(yaw) - y_offset*sin(yaw);
  double y_inc = x_offset*sin(yaw) + y_offset*cos(yaw);

  // Set position in Gazebo
#if GAZEBO_MAJOR_VERSION >= 6
  msgs::Set(msg.mutable_pose(), ignition::math::Pose3d(
              ignition::math::Vector3d(x + x_inc, y + y_inc, WRENCH_Z),
              ignition::math::Quaterniond(0, 0, yaw)));
#else
  // Gazebo2 support
  msgs::Set(msg.mutable_pose(), math::Pose(x + x_inc, y + y_inc, WRENCH_Z, 0, 0, yaw));
#endif

  // Publish
  factoryPub->Publish(msg);
}

void GazeboPanelPosition::InsertValveStem(std::string model_name, double x, double y, double x_offset, double y_offset, double yaw)
{
  // Create a new transport node
  transport::NodePtr node(new transport::Node());
  node->Init(world_->GetName());
  transport::PublisherPtr factoryPub = node->Advertise<msgs::Factory>("~/factory");

  msgs::Factory msg;
  msg.set_sdf_filename(model_name);

  // Calculate wrench position
  double x_inc = x_offset*cos(yaw) - y_offset*sin(yaw);
  double y_inc = x_offset*sin(yaw) + y_offset*cos(yaw);

  // Set position in Gazebo
#if GAZEBO_MAJOR_VERSION >= 6
  msgs::Set(msg.mutable_pose(), ignition::math::Pose3d(
              ignition::math::Vector3d(x + x_inc, y + y_inc, VALVE_STEM_Z),
              ignition::math::Quaterniond(0, -M_PI_2, yaw)));
#else
  // Gazebo2 support
  msgs::Set(msg.mutable_pose(), math::Pose(x + x_inc, y + y_inc, VALVE_STEM_Z, 0, -M_PI_2, yaw));
#endif

  // Publish
  factoryPub->Publish(msg);
}


////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboPanelPosition::Reset()
{
  terminated_ = false;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboPanelPosition)

}  // namespace gazebo
