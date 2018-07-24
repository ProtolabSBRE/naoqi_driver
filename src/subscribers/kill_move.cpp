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

/*
 * LOCAL includes
 */
#include "kill_move.hpp"

/*
 * ROS includes
 */
//#include <tf/transform_datatypes.h>
#include "../helpers/transform_helpers.hpp"

namespace naoqi
{
namespace subscriber
{

KillMoveSubscriber::KillMoveSubscriber(
        const std::string& name,
        const std::string& topic,
        const qi::SessionPtr& session):
        BaseSubscriber(
            name,
            topic,
            session),
        p_motion_( session->service("ALMotion")) {}

void KillMoveSubscriber::reset(ros::NodeHandle& nh) {
  sub_killmove_ = nh.subscribe(topic_, 10, &KillMoveSubscriber::callback, this);
  is_initialized_ = true;
}

void KillMoveSubscriber::callback(const std_msgs::EmptyConstPtr &msg) {
    this->p_motion_.async<void>("killMove");
}

} //publisher
} // naoqi
