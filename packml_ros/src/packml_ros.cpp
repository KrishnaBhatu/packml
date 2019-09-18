/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2017 Shaun Edwards
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "packml_ros/packml_ros.h"
#include "packml_sm/packml_stats_snapshot.h"
#include "packml_sm/packml_stats_itemized.h"

#include <packml_msgs/utils.h>
#include "packml_msgs/ItemizedStats.h"

namespace packml_ros
{

PackmlRos::PackmlRos(ros::NodeHandle nh, ros::NodeHandle pn, std::shared_ptr<packml_sm::AbstractStateMachine> sm)
  : nh_(nh), pn_(pn), sm_(sm)
{
  ros::NodeHandle packml_node("~/packml");

  status_pub_ = packml_node.advertise<packml_msgs::Status>("status", 10, true);
  stats_pub_ = packml_node.advertise<packml_msgs::Stats>("stats", 10, true);
  stats_transaction_pub_ = packml_node.advertise<packml_msgs::Stats>("stats_transaction", 10, true);

  trans_server_ = packml_node.advertiseService("transition", &PackmlRos::transRequest, this);
  reset_stats_server_ = packml_node.advertiseService("reset_stats", &PackmlRos::resetStats, this);
  get_stats_server_ = packml_node.advertiseService("get_stats", &PackmlRos::getStats, this);
  load_stats_server_ = packml_node.advertiseService("load_stats", &PackmlRos::loadStats, this);

  status_msg_ = packml_msgs::initStatus(pn.getNamespace());

  if (!pn_.getParam("stats_publish_period", stats_publish_period_))
  {
    ROS_WARN_STREAM("Missing param: stats_publish_period. Defaulting to 1 second");
    stats_publish_period_ = 1;
  }
  if(stats_publish_period_ <= 0)
  {
    ROS_WARN_STREAM("stats_publish_period <= 0. stats will not be published regularly");
  }
  else
  {
    stats_timer_ = nh_.createTimer(ros::Duration(stats_publish_period_), &PackmlRos::publishStatsCb, this);
  }

  if (!pn_.getParam("stats_transaction_publish_period", stats_transaction_publish_period_))
  {
    ROS_WARN_STREAM("Missing param: stats_transaction_publish_period. Defaulting to 15 minutes");
    stats_transaction_publish_period_ = 900;
  }
  if(stats_transaction_publish_period_ <= 0)
  {
    ROS_WARN_STREAM("stats_transaction_publish_period <= 0. Stats transactions will not be published");
  }
  else
  {
    stats_transaction_timer_ = nh_.createTimer(ros::Duration(stats_transaction_publish_period_),
                                               &PackmlRos::publishStatsTransactionCb, this);
  }

  sm_->stateChangedEvent.bind_member_func(this, &PackmlRos::handleStateChanged);
  sm_->activate();

}

PackmlRos::~PackmlRos()
{
  if (sm_ != nullptr)
  {
    sm_->stateChangedEvent.unbind_member_func(this, &PackmlRos::handleStateChanged);
  }
}

void PackmlRos::spin()
{
  while (ros::ok())
  {
    spinOnce();
    ros::Duration(0.001).sleep();
  }
  return;
}

void PackmlRos::spinOnce()
{
  ros::spinOnce();
}

bool PackmlRos::transRequest(packml_msgs::Transition::Request& req, packml_msgs::Transition::Response& res)
{
  bool command_rtn = false;
  bool command_valid = true;
  int command_int = static_cast<int>(req.command);
  std::stringstream ss;
  ROS_DEBUG_STREAM("Evaluating transition request command: " << command_int);

  switch (command_int)
  {
    case req.ABORT:
    case req.ESTOP:
      command_rtn = sm_->abort();
      break;
    case req.CLEAR:
      command_rtn = sm_->clear();
      break;
    case req.HOLD:
      command_rtn = sm_->hold();
      break;
    case req.RESET:
      command_rtn = sm_->reset();
      break;
    case req.START:
      command_rtn = sm_->start();
      break;
    case req.STOP:
      command_rtn = sm_->stop();
      break;
    case req.SUSPEND:
      command_rtn = sm_->suspend();
      break;
    case req.UNHOLD:
      command_rtn = sm_->unhold();
      break;
    case req.UNSUSPEND:
      command_rtn = sm_->unsuspend();
      break;

    default:
      command_valid = false;
      break;
  }
  if (command_valid)
  {
    if (command_rtn)
    {
      ss << "Successful transition request command: " << command_int;
      ROS_INFO_STREAM(ss.str());
      res.success = true;
      res.error_code = res.SUCCESS;
      res.message = ss.str();
    }
    else
    {
      ss << "Invalid transition request command: " << command_int;
      ROS_ERROR_STREAM(ss.str());
      res.success = false;
      res.error_code = res.INVALID_TRANSITION_REQUEST;
      res.message = ss.str();
    }
  }
  else
  {
    ss << "Unrecognized transition request command: " << command_int;
    ROS_ERROR_STREAM(ss.str());
    res.success = false;
    res.error_code = res.UNRECOGNIZED_REQUEST;
    res.message = ss.str();
  }
}

void PackmlRos::handleStateChanged(packml_sm::AbstractStateMachine& state_machine,
                                   const packml_sm::StateChangedEventArgs& args)
{
  ROS_DEBUG_STREAM("Publishing state change: " << args.name << "(" << args.value << ")");

  status_msg_.header.stamp = ros::Time().now();
  int cur_state = static_cast<int>(args.value);
  if (packml_msgs::isStandardState(cur_state))
  {
    status_msg_.state.val = cur_state;
    status_msg_.sub_state = packml_msgs::State::UNDEFINED;
  }
  else
  {
    status_msg_.sub_state = cur_state;
  }

  status_pub_.publish(status_msg_);
  publishStats();
}

void PackmlRos::getCurrentStats(packml_msgs::Stats& out_stats)
{
  packml_sm::PackmlStatsSnapshot stats_snapshot;
  sm_->getCurrentStatSnapshot(stats_snapshot);
  out_stats = populateStatsMsg(stats_snapshot);
}


void PackmlRos::getStatsTransaction(packml_msgs::Stats &out_stats)
{
  packml_sm::PackmlStatsSnapshot stats_snapshot;
  sm_->getCurrentStatSnapshot(stats_snapshot);
  out_stats = populateStatsMsg(stats_snapshot);
}

packml_msgs::Stats PackmlRos::populateStatsMsg(const packml_sm::PackmlStatsSnapshot& stats_snapshot)
{
  packml_msgs::Stats stats_msg;

  stats_msg.idle_duration.data.fromSec(stats_snapshot.idle_duration);
  stats_msg.exe_duration.data.fromSec(stats_snapshot.exe_duration);
  stats_msg.held_duration.data.fromSec(stats_snapshot.held_duration);
  stats_msg.susp_duration.data.fromSec(stats_snapshot.susp_duration);
  stats_msg.cmplt_duration.data.fromSec(stats_snapshot.cmplt_duration);
  stats_msg.stop_duration.data.fromSec(stats_snapshot.stop_duration);
  stats_msg.abort_duration.data.fromSec(stats_snapshot.abort_duration);
  stats_msg.duration.data.fromSec(stats_snapshot.duration);
  stats_msg.fail_count = stats_snapshot.fail_count;
  stats_msg.success_count = stats_snapshot.success_count;
  stats_msg.availability = stats_snapshot.availability;
  stats_msg.performance = stats_snapshot.performance;
  stats_msg.quality = stats_snapshot.quality;
  stats_msg.overall_equipment_effectiveness = stats_snapshot.overall_equipment_effectiveness;

  stats_msg.error_items.clear();
  for (const auto& itemized_it : stats_snapshot.itemized_error_map)
  {
    packml_msgs::ItemizedStats stat;
    stat.id = itemized_it.second.id;
    stat.count = itemized_it.second.count;
    stat.duration.data.fromSec(itemized_it.second.duration);
    stats_msg.error_items.push_back(stat);
  }

  stats_msg.quality_items.clear();
  for (const auto& itemized_it : stats_snapshot.itemized_quality_map)
  {
    packml_msgs::ItemizedStats stat;
    stat.id = itemized_it.second.id;
    stat.count = itemized_it.second.count;
    stat.duration.data.fromSec(itemized_it.second.duration);
    stats_msg.quality_items.push_back(stat);
  }

  stats_msg.header.stamp = ros::Time::now();
  return stats_msg;
}

packml_sm::PackmlStatsSnapshot PackmlRos::populateStatsSnapshot(const packml_msgs::Stats &msg)
{
  packml_sm::PackmlStatsSnapshot snapshot;

  snapshot.cycle_count = msg.cycle_count;
  snapshot.success_count = msg.success_count;
  snapshot.fail_count = msg.fail_count;
  snapshot.throughput = msg.throughput;
  snapshot.availability = msg.availability;
  snapshot.performance = msg.performance;
  snapshot.quality = msg.quality;
  snapshot.overall_equipment_effectiveness = msg.overall_equipment_effectiveness;

  snapshot.duration = msg.duration.data.toSec();
  snapshot.idle_duration = msg.idle_duration.data.toSec();
  snapshot.exe_duration = msg.exe_duration.data.toSec();
  snapshot.held_duration = msg.held_duration.data.toSec();
  snapshot.susp_duration = msg.susp_duration.data.toSec();
  snapshot.cmplt_duration = msg.cmplt_duration.data.toSec();
  snapshot.stop_duration = msg.stop_duration.data.toSec();
  snapshot.abort_duration = msg.abort_duration.data.toSec();

  std::map<int16_t, packml_sm::PackmlStatsItemized> itemized_error_map;
  for (const auto& error_item : msg.error_items)
  {
    packml_sm::PackmlStatsItemized item;
    item.id = error_item.id;
    item.count = error_item.count;
    item.duration = error_item.duration.data.toSec();
    itemized_error_map.insert(std::pair<int16_t, packml_sm::PackmlStatsItemized>(error_item.id, item));
  }
  snapshot.itemized_error_map = itemized_error_map;

  std::map<int16_t, packml_sm::PackmlStatsItemized> itemized_quality_map;
  for (const auto& quality_item : msg.quality_items)
  {
    packml_sm::PackmlStatsItemized item;
    item.id = quality_item.id;
    item.count = quality_item.count;
    item.duration = quality_item.duration.data.toSec();
    itemized_quality_map.insert(std::pair<int16_t, packml_sm::PackmlStatsItemized>(quality_item.id, item));
  }
  snapshot.itemized_quality_map = itemized_quality_map;

  return snapshot;
}

bool PackmlRos::getStats(packml_msgs::GetStats::Request& req, packml_msgs::GetStats::Response& response)
{
  packml_msgs::Stats stats;
  getCurrentStats(stats);
  response.stats = stats;

  return true;
}

bool PackmlRos::resetStats(packml_msgs::ResetStats::Request& req, packml_msgs::ResetStats::Response& response)
{
  packml_msgs::Stats stats;
  getCurrentStats(stats);
  response.last_stat = stats;

  sm_->resetStats();

  return true;
}

void PackmlRos::publishStatsCb(const ros::TimerEvent&)
{
  publishStats();
}

void PackmlRos::publishStats()
{
  // Check if stats_publish_period changed
  float stats_publish_period_new;
  if (pn_.getParam("stats_publish_period", stats_publish_period_new))
  {
    if (stats_publish_period_new != stats_publish_period_ && stats_publish_period_new > 0)
    {
      stats_timer_ = nh_.createTimer(ros::Duration(stats_publish_period_new), &PackmlRos::publishStatsCb, this);
    }
  }

  packml_msgs::Stats stats;
  getStatsTransaction(stats);
  stats_pub_.publish(stats);
}

void PackmlRos::publishStatsTransactionCb(const ros::TimerEvent &timer_event)
{
  // Check if stats_transaction_publish_period changed
  float stats_transaction_publish_period_new;
  if (pn_.getParam("stats_transaction_publish_period", stats_transaction_publish_period_new))
  {
    if (stats_transaction_publish_period_new != stats_transaction_publish_period_ && stats_transaction_publish_period_new > 0)
    {
      stats_transaction_timer_ = nh_.createTimer(ros::Duration(stats_transaction_publish_period_new),
                                                 &PackmlRos::publishStatsCb, this);
    }
  }

  packml_msgs::Stats stats;
  getCurrentStats(stats);
  stats_transaction_pub_.publish(stats);
}

bool PackmlRos::loadStats(packml_msgs::LoadStats::Request &req, packml_msgs::LoadStats::Response &response)
{
  packml_sm::PackmlStatsSnapshot snapshot = populateStatsSnapshot(req.stats);
  sm_->loadStats(snapshot);

  return true;
}
}  // namespace kitsune_robot
