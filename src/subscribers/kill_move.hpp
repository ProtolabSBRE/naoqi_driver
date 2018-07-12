/*
 * Copyright 2015 Aldebaran
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


#ifndef KILL_MOVE_HPP
#define KILL_MOVE_HPP

/*
 * LOCAL includes
 */
#include "subscriber_base.hpp"

/*
 * ROS includes
 */
#include <ros/ros.h>
#include <std_msgs/Empty.h>

namespace naoqi
{
namespace subscriber
{

class KillMoveSubscriber: public BaseSubscriber<KillMoveSubscriber>
{
public:
  KillMoveSubscriber( const std::string& name, const std::string& topic, const qi::SessionPtr& session);
  ~KillMoveSubscriber(){}

  void reset( ros::NodeHandle& nh );
  void callback( const std_msgs::EmptyConstPtr &msg);

private:
  qi::AnyObject p_motion_;
  ros::Subscriber sub_killmove_;
};

} // subscriber
}// naoqi

#endif // KILL_MOVE_HPP
