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
#ifndef PACKML_ROS_H
#define PACKML_ROS_H

#include <ros/ros.h>

#include <packml_msgs/GetStats.h>
#include <packml_msgs/IncrementStat.h>
#include <packml_msgs/InvokeStateChange.h>
#include <packml_msgs/LoadStats.h>
#include <packml_msgs/ResetStats.h>
#include <packml_msgs/SendCommand.h>
#include <packml_msgs/SendEvent.h>
#include <packml_msgs/Status.h>
#include <packml_msgs/Stats.h>
#include <packml_sm/abstract_state_machine.h>
#include <packml_sm/boost/packml_state_machine_continuous.h>
#include <packml_sm/common.h>

namespace packml_ros
{
class PackmlRos
{
public:

  /**
   * @brief Constructor for PackmlRos
   *
   * @param nh Private node handle
   * @param pn Node handle
   * @param sm Packml state machine
   */
  PackmlRos(ros::NodeHandle nh, ros::NodeHandle pn, std::shared_ptr<packml_sm::PackmlStateMachineContinuous> sm);

  /**
   * @brief Destructor for PackmlRos
   */
  ~PackmlRos();

  /**
   * @brief Overloads ros::spin()
   */
  void spin();

  /**
   * @brief Overloads ros::spinOnce()
   */
  void spinOnce();

protected:
  ros::NodeHandle nh_;                                  /** Node handle */
  ros::NodeHandle pn_;                                  /** Private node handle */
  std::shared_ptr<packml_sm::PackmlStateMachineContinuous> sm_; /** Packml state machine */
  ros::Publisher status_pub_;                           /** Publisher for Packml status */
  ros::Publisher stats_pub_;                            /** Publisher for Packml stats */
  ros::Publisher incremental_stats_pub_;                /** Publisher for incremental Packml stats */
  ros::ServiceServer command_server_;                   /** Advertises service to send commands to Packml state machine */
  ros::ServiceServer reset_stats_server_;               /** Advertises service for resetting stats */
  ros::ServiceServer get_stats_server_;                 /** Advertises service for getting stats */
  ros::ServiceServer load_stats_server_;                /** Advertises service for loading stats */
  ros::ServiceServer events_server_;                    /** Advertises service to send events to Packml state machine */
  ros::ServiceServer invoke_state_change_server_;       /** Advertises service to invoke a state change */
  ros::ServiceServer inc_stat_server_;                  /** Advertises service to increment a stat*/
  packml_msgs::Status status_msg_;                      /** Message containing Packml status */
  double stats_publish_rate_;                           /** Rate at which rolling Packml stats are calculated and published */
  double incremental_stats_publish_rate_;               /** Rate at which incremental Packml stats are calculated and published */
  double ideal_cycle_time_;				/** Ideal time for a cycle used to calculate performance and OEE*/
  ros::Timer stats_timer_;                              /** Timer used to publish Packml stats */
  ros::Timer incremental_stats_timer_;                  /** Timer used to publish incremental Packml stats */

  /**
   * @brief Processes external packml commands
   * @details See OMAC PackML Unit/Machine Implementation Guide for command details
   *
   * @return True if request is processed, False otherwise
   */
  bool commandRequest(packml_msgs::SendCommand::Request& req, packml_msgs::SendCommand::Response& res);

  /**
   * @brief Processes external packml events
   * @details See OMAC PackML Unit/Machine Implementation Guide for event details
   *
   * @return True if request is processed, False otherwise
   */
  bool eventRequest(packml_msgs::SendEvent::Request& req, packml_msgs::SendEvent::Response& res);

  /**
   * @brief Processes state change invocations
   * @details Does not update actual state, but allows updates timers to track the desired state
   *
   * @return True if request is processed, False otherwise
   */
  bool triggerStateChange(packml_msgs::InvokeStateChange::Request& req, packml_msgs::InvokeStateChange::Response& res);

  /**
   * @brief Processes request to increment statistics
   *
   * @return True if stat was processed successfully, False otherwise
   */
  bool incStatRequest(packml_msgs::IncrementStat::Request& req, packml_msgs::IncrementStat::Response& res);

private:
  /**
   * @brief Event callback triggered on state change
   *
   * @param state_machine Packml state machine
   * @param args Arguments for bound function
   */
  void handleStateChanged(packml_sm::AbstractStateMachine& state_machine, const packml_sm::StateChangedEventArgs& args);

  /**
   * @brief Populates stats message with current Packml stats
   *
   * @param out_stats Stats message
   */
  void getCurrentStats(packml_msgs::Stats& out_stats);

  bool incStat(const int& metric, const double& step);

  /**
   * @brief Pepulates stats message with current incremental Packml stats
   *
   * @param out_stats Stats message
   */
  void getIncrementalStats(packml_msgs::Stats &out_stats);

  /**
   * @brief Converts stats snapshot to stats message
   *
   * @param stats_snapshot data structure with stats data
   * @return stats message
   */
  packml_msgs::Stats populateStatsMsg(const packml_sm::PackmlStatsSnapshot& stats_snapshot);

  /**
   * @brief Service callback for getting stats
   *
   * @param req GetStats srv request
   * @param response GesStats srv response
   * @return success bool
   */
  bool getStats(packml_msgs::GetStats::Request& req, packml_msgs::GetStats::Response& response);

  /**
   * @brief Service callback for resetting stats
   *
   * @param req ResetStats srv request
   * @param response ResetStats srv response
   * @return success bool
   */
  bool resetStats(packml_msgs::ResetStats::Request& req, packml_msgs::ResetStats::Response& response);

  /**
   * @brief Service callback for loading stats
   *
   * @param req LoadStats srv request
   * @param response LoadStats srv response
   * @return success bool
   */
  bool loadStats(packml_msgs::LoadStats::Request& req, packml_msgs::LoadStats::Response& response);

  /**
   * @brief Converts stats message to stats snapshot
   *
   * @param msg stats message
   * @return stats snapshot
   */
  packml_sm::PackmlStatsSnapshot populateStatsSnapshot(const packml_msgs::Stats& msg);

  /**
   * @brief Timer callback for publishing Packml stats
   *
   * @param timer_event ROS TimerEvent; not used
   */
  void publishStatsCb(const ros::TimerEvent& timer_event);

  /**
   * @brief Timer callback for publishing incremental Packml stats
   *
   * @param timer_event ROS TimerEvent; not used
   */
  void publishIncrementalStatsCb(const ros::TimerEvent &timer_event);

  /**
   * @brief Publishes Packml stats
   */
  void publishStats();

  /**
   * @brief Evaluates validity and effect of command request
   * @details Issues the command, if valid, and returns result
   *
   * @param command_int Requested command to issue
   * @param command_valid True if command is in the implemented standard, False otherwise
   * @param command_rtn True if the command was accepted in the current context, False otherwise
   * @return True if command is valid and was accpeted in the current context, False otherwise
   */
  bool commandGuard(const int& command_int, bool &command_valid, bool &command_rtn);

  /**
   * @brief Evaluates validity of event and triggers if valid
   *
   * @param event_id Requested event to trigger
   * @return True if the event is valid and was triggered in the state machine, false otherwise
   */
  bool eventGuard(const int& event_id);
};
}  // namespace packml_ros

#endif  // PACKML_ROS_H
