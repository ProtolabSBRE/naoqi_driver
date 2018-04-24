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
#include "look_at.hpp"

/*
 * ROS includes
 */
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "../helpers/transform_helpers.hpp"

namespace naoqi
{
namespace subscriber
{

LookAtSubscriber::LookAtSubscriber (
        const std::string &name,
        const std::string &topic,
        const qi::SessionPtr &session,
        const boost::shared_ptr<tf2_ros::Buffer> &tf2_buffer):
    BaseSubscriber(name, topic, session),
    p_tracker_(session->service("ALTracker")),
    tf2_buffer_(tf2_buffer){}


void LookAtSubscriber::reset(ros::NodeHandle &nh) {
    this->sub_lookAt_ = nh.subscribe(this->topic_,
                                     10,
                                     &LookAtSubscriber::callback,
                                     this);
    this->is_initialized_ = true;
}



void LookAtSubscriber::callback(
        const naoqi_bridge_msgs::PoseStampedWithSpeedConstPtr &msg) {

    int                frame;
    float              speedPercentage;
    std::vector<float> position;

    speedPercentage = msg->speed_percentage;

    if (speedPercentage < 0)
        speedPercentage = 0;

    else if (speedPercentage > 1)
        speedPercentage = 1;

    if (msg->pose_stamped.header.frame_id == "torso")
        frame = 0;

    else if (msg->pose_stamped.header.frame_id == "base_link")
        frame = 2;

    else if (msg->pose_stamped.header.frame_id == "odom")
        frame = 1;

    else
        frame = -1;

    if (frame != -1) {
        position.push_back(msg->pose_stamped.pose.position.x);
        position.push_back(msg->pose_stamped.pose.position.y);
        position.push_back(msg->pose_stamped.pose.position.z);

        this->p_tracker_.async<void>("lookAt",
                                     position,
                                     frame,
                                     speedPercentage,
                                     false);
    }

    else {
        geometry_msgs::PoseStamped pose_msg_bf;

        bool canTransform = this->tf2_buffer_->canTransform(
                    "torso",
                    msg->pose_stamped.header.frame_id,
                    ros::Time(0),
                    ros::Duration(2));

        if (!canTransform) {
            std::cout << "Cannot transform from "
                      << msg->pose_stamped.header.frame_id
                      << " to torso" << std::endl;
            return;
        }

        try {
            geometry_msgs::PoseStamped poseStamped;
            poseStamped = msg->pose_stamped;

            this->tf2_buffer_->transform(poseStamped,
                                         pose_msg_bf,
                                         "torse",
                                         ros::Time(0),
                                         msg->pose_stamped.header.frame_id);

            position.push_back(pose_msg_bf.pose.position.x);
            position.push_back(pose_msg_bf.pose.position.y);
            position.push_back(pose_msg_bf.pose.position.z);

            this->p_tracker_.async<void>("lookAt",
                                         position,
                                         0,
                                         speedPercentage,
                                         false);

        } catch (const tf2::LookupException &e) {
            std::cout << e.what() << std::endl;

        } catch (const tf2::ExtrapolationException &e) {
            std::cout << "Received an error on the time lookup" << std::endl;
        }
    }
}

}
}
