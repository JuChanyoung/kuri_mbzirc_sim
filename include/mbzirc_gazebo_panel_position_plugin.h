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

#ifndef MBZIRC_GAZEBO_PANEL_PLUGIN_H
#define MBZIRC_GAZEBO_PANEL_PLUGIN_H

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/RayShape.hh>


#include <stdio.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace gazebo
{

class GazeboPanelPosition : public ModelPlugin
{
public:
  GazeboPanelPosition();
  virtual ~GazeboPanelPosition();

  // http://www.mbzirc.com/assets/files/MBZIRC-Challenge-Description-Document-V2-7SEP2015.pdf
  const double MIN_RANGE = 20;
  const double MAX_RANGE = 40;

  const double PANEL_Z = 0.975;
  const double WRENCH_Z = 1.2;
  const double VALVE_STEM_Z = 0.98;
  const double STARTING_ZONE_X = 0;
  const double STARTING_ZONE_Y = 24.5;



protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  virtual void Reset();
  virtual void InsertWrench(std::string model_name, double x, double y, double x_offset, double y_offset, double yaw);
  virtual void InsertValveStem(std::string model_name, double x, double y, double x_offset, double y_offset, double yaw);

private:

  physics::WorldPtr world_;
  physics::LinkPtr link_;
  physics::JointPtr joint_;
  physics::ModelPtr model_;

  std::string link_name_;
  std::string namespace_;

  //boost::mutex lock;

  event::ConnectionPtr update_connection_;

  bool terminated_;

};

}

#endif
