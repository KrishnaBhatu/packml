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

#include <packml_ros/utils.h>
#include "packml_msgs/ItemizedStats.h"

#include "packml_ros/packml_ros.h"

#include <packml_sm/boost/packml_events.h>
#include <packml_sm/common.h>
#include "packml_sm/packml_stats_snapshot.h"
#include "packml_sm/packml_stats_itemized.h"

namespace packml_ros
{

PackmlRos::PackmlRos(ros::NodeHandle nh, ros::NodeHandle pn, std::shared_ptr<packml_sm::PackmlStateMachineContinuous> sm)
  : nh_(nh), pn_(pn), sm_(sm)
{
  ros::NodeHandle packml_node("~/packml");

  status_pub_ = packml_node.advertise<packml_msgs::Status>("status", 10, true);
  stats_pub_ = packml_node.advertise<packml_msgs::Stats>("stats", 10, true);
  incremental_stats_pub_ = packml_node.advertise<packml_msgs::Stats>("incremental_stats", 10, true);

  command_server_ = packml_node.advertiseService("send_command", &PackmlRos::commandRequest, this);
  reset_stats_server_ = packml_node.advertiseService("reset_stats", &PackmlRos::resetStats, this);
  get_stats_server_ = packml_node.advertiseService("get_stats", &PackmlRos::getStats, this);
  load_stats_server_ = packml_node.advertiseService("load_stats", &PackmlRos::loadStats, this);
  events_server_ = packml_node.advertiseService("send_event", &PackmlRos::eventRequest, this);
  invoke_state_change_server_ = packml_node.advertiseService("invoke_state_change", &PackmlRos::triggerStateChange, this);
  inc_stat_server_ = packml_node.advertiseService("inc_stat", &PackmlRos::incStatRequest, this);

  status_msg_ = initStatus(pn.getNamespace());

  if (!pn_.getParam("stats_publish_rate", stats_publish_rate_))
  {
    ROS_WARN_STREAM("Missing param: stats_publish_rate. Defaulting to 1 second");
    stats_publish_rate_ = 1;
  }
  if(stats_publish_rate_ <= 0)
  {
    ROS_WARN_STREAM("stats_publish_rate <= 0. stats will not be published regularly");
  }
  else
  {
    stats_timer_ = nh_.createTimer(ros::Duration(stats_publish_rate_), &PackmlRos::publishStatsCb, this);
  }

  if (!pn_.getParam("incremental_stats_publish_rate", incremental_stats_publish_rate_))
  {
    ROS_WARN_STREAM("Missing param: incremental_stats_publish_rate. Defaulting to 15 minutes");
    incremental_stats_publish_rate_ = 900;
  }
  if(incremental_stats_publish_rate_ <= 0)
  {
    ROS_WARN_STREAM("incremental_stats_publish_rate <= 0. Incremental stats will not be published");
  }
  else
  {
    incremental_stats_timer_ = nh_.createTimer(ros::Duration(incremental_stats_publish_rate_),
                                               &PackmlRos::publishIncrementalStatsCb, this);
  }

  if (!pn_.getParam("ideal_cycle_time", ideal_cycle_time_))
  {
    ROS_WARN_STREAM("Missing param: ideal_cycle_time. Defaulting to 0.1 second");
    ideal_cycle_time_ = 0.1;
  }

  sm_->stateChangedEvent.bind_member_func(this, &PackmlRos::handleStateChanged);
  sm_->activate();
  sm_->setIdealCycleTime(ideal_cycle_time_);

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

bool PackmlRos::commandRequest(packml_msgs::SendCommand::Request& req, packml_msgs::SendCommand::Response& res)
{
  bool command_rtn = false;
  bool command_valid = true;
  int command_int = static_cast<int>(req.command);
  std::stringstream ss;
  ROS_DEBUG_STREAM("Evaluating transition request command: " << command_int);

  commandGuard(command_int, command_valid, command_rtn);

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

  return true;
}

bool PackmlRos::eventRequest(packml_msgs::SendEvent::Request& req, packml_msgs::SendEvent::Response& res)
{
  auto event_id = req.event_id;
  res.result = eventGuard(event_id);
  return true;
}

bool PackmlRos::triggerStateChange(packml_msgs::InvokeStateChange::Request& req, packml_msgs::InvokeStateChange::Response& res)
{
  sm_->invokeStateChangedEvent(req.name, static_cast<packml_sm::StatesEnum>(req.state_enum));
  return true;
}

bool PackmlRos::incStatRequest(packml_msgs::IncrementStat::Request& req, packml_msgs::IncrementStat::Response& res)
{
    return incStat(req.metric, req.step);
}

void PackmlRos::handleStateChanged(packml_sm::AbstractStateMachine& state_machine,
                                   const packml_sm::StateChangedEventArgs& args)
{
  ROS_DEBUG_STREAM("Publishing state change: " << args.name << "(" << args.value << ")");

  status_msg_.header.stamp = ros::Time().now();
  int cur_state = static_cast<int>(args.value);
  if (isStandardState(cur_state))
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


void PackmlRos::getIncrementalStats(packml_msgs::Stats &out_stats)
{
  packml_sm::PackmlStatsSnapshot stats_snapshot;
  sm_->getCurrentIncrementalStatSnapshot(stats_snapshot);
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

  for (const auto& itemized_it : stats_snapshot.itemized_error_map)
  {
    packml_msgs::ItemizedStats stat;
    stat.id = itemized_it.second.id;
    stat.count = itemized_it.second.count;
    stat.duration.data.fromSec(itemized_it.second.duration);
    stats_msg.error_items.push_back(stat);
  }

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
  // Check if stats_publish_rate changed
  double stats_publish_rate_new;
  if (pn_.getParam("stats_publish_rate", stats_publish_rate_new))
  {
    if (stats_publish_rate_new != stats_publish_rate_ && stats_publish_rate_new > 0)
    {
      stats_timer_ = nh_.createTimer(ros::Duration(stats_publish_rate_new), &PackmlRos::publishStatsCb, this);
    }
  }

  packml_msgs::Stats stats;
  getCurrentStats(stats);
  stats_pub_.publish(stats);
}

void PackmlRos::publishIncrementalStatsCb(const ros::TimerEvent &timer_event)
{
  // Check if incremental_stats_publish_rate changed
  double incremental_stats_publish_rate_new;
  if (pn_.getParam("incremental_stats_publish_rate", incremental_stats_publish_rate_new))
  {
    if (incremental_stats_publish_rate_new != incremental_stats_publish_rate_ && incremental_stats_publish_rate_new > 0)
    {
      incremental_stats_timer_ = nh_.createTimer(ros::Duration(incremental_stats_publish_rate_new),
                                                 &PackmlRos::publishIncrementalStatsCb, this);
    }
  }

  packml_msgs::Stats stats;
  getIncrementalStats(stats);
  incremental_stats_pub_.publish(stats);
}

bool PackmlRos::loadStats(packml_msgs::LoadStats::Request &req, packml_msgs::LoadStats::Response &response)
{
  packml_sm::PackmlStatsSnapshot snapshot = populateStatsSnapshot(req.stats);
  sm_->loadStats(snapshot);

  return true;
}

bool PackmlRos::commandGuard(const int& command_int, bool& command_valid, bool& command_rtn)
{
  command_valid = true;
  command_rtn = false;

  switch (command_int)
  {
    case static_cast<int>(packml_sm::CmdEnum::ABORT):
      command_rtn = sm_->abort();
      break;
    case static_cast<int>(packml_sm::CmdEnum::CLEAR):
      command_rtn = sm_->clear();
      break;
    case static_cast<int>(packml_sm::CmdEnum::HOLD):
      command_rtn = sm_->hold();
      break;
    case static_cast<int>(packml_sm::CmdEnum::RESET):
      command_rtn = sm_->reset();
      break;
    case static_cast<int>(packml_sm::CmdEnum::START):
      command_rtn = sm_->start();
      break;
    case static_cast<int>(packml_sm::CmdEnum::STOP):
      command_rtn = sm_->stop();
      break;
    case static_cast<int>(packml_sm::CmdEnum::SUSPEND):
      command_rtn = sm_->suspend();
      break;
    case static_cast<int>(packml_sm::CmdEnum::UNHOLD):
      command_rtn = sm_->unhold();
      break;
    case static_cast<int>(packml_sm::CmdEnum::UNSUSPEND):
      command_rtn = sm_->unsuspend();
      break;

    default:
      command_valid = false;
  }
  return command_valid && command_rtn;
}

bool PackmlRos::eventGuard(const int& event_id)
{
  switch(event_id)
  {
    case static_cast<int>(packml_sm::EventsEnum::STATE_COMPLETE):
      sm_->triggerEvent(packml_sm::state_complete_event());
      break;
    case static_cast<int>(packml_sm::EventsEnum::HOLD):
      sm_->triggerEvent(packml_sm::hold_event());
      break;
    case static_cast<int>(packml_sm::EventsEnum::UNHOLD):
      sm_->triggerEvent(packml_sm::unhold_event());
      break;
    case static_cast<int>(packml_sm::EventsEnum::SUSPEND):
      sm_->triggerEvent(packml_sm::suspend_event());
      break;
    case static_cast<int>(packml_sm::EventsEnum::UNSUSPEND):
      sm_->triggerEvent(packml_sm::unsuspend_event());
      break;
    case static_cast<int>(packml_sm::EventsEnum::RESET):
      sm_->triggerEvent(packml_sm::reset_event());
      break;
    case static_cast<int>(packml_sm::EventsEnum::CLEAR):
      sm_->triggerEvent(packml_sm::clear_event());
      break;
    default:
      ROS_ERROR_STREAM_NAMED("packml", "Event request called with invalid event_id: " << event_id);
      return false;
  }
  return true;
}

bool PackmlRos::incStat(const int& metric, const double& step)
{
  bool result = true;

  switch(metric)
  {
    case static_cast<int32_t>(packml_sm::MetricIDEnum::CYCLE_INC_ID):
      //do nothing, we don't have a way to increment cycle yet.
      break;
    case static_cast<int32_t>(packml_sm::MetricIDEnum::SUCCESS_INC_ID):
      sm_->incrementSuccessCount();
      break;
    case static_cast<int32_t>(packml_sm::MetricIDEnum::FAILURE_INC_ID):
      sm_->incrementFailureCount();
      break;
    default:
      if(metric >= static_cast<int32_t>(packml_sm::MetricIDEnum::MIN_QUALITY_ID)
        && metric <= static_cast<int32_t>(packml_sm::MetricIDEnum::MAX_QUALITY_ID))
      {
        sm_->incrementQualityStatItem(metric, step);
        break;
      }
      else if(metric >= static_cast<int32_t>(packml_sm::MetricIDEnum::MIN_ERROR_ID)
        && metric <= static_cast<int32_t>(packml_sm::MetricIDEnum::MAX_ERROR_ID))
      {
        sm_->incrementErrorStatItem(metric, step);
        break;
      }
      else
      {
        ROS_ERROR_STREAM_NAMED("packml", "Increment stat request called with invalid metric id: " << metric);
        result = false;
      }
  }
  return result;
}
}  // namespace kitsune_robot
