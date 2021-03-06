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

#include <packml_msgs/Status.h>
#include "packml_sm/common.h"

namespace packml_ros
{
bool isStandardState(int state)
{
  switch (state)
  {
    case packml_msgs::State::ABORTED:
    case packml_msgs::State::ABORTING:
    case packml_msgs::State::CLEARING:
    case packml_msgs::State::COMPLETE:
    case packml_msgs::State::COMPLETING:
    case packml_msgs::State::EXECUTE:
    case packml_msgs::State::HELD:
    case packml_msgs::State::HOLDING:
    case packml_msgs::State::IDLE:
    case packml_msgs::State::OFF:
    case packml_msgs::State::RESETTING:
    case packml_msgs::State::STARTING:
    case packml_msgs::State::STOPPED:
    case packml_msgs::State::STOPPING:
    case packml_msgs::State::SUSPENDED:
    case packml_msgs::State::SUSPENDING:
    case packml_msgs::State::UNHOLDING:
    case packml_msgs::State::UNSUSPENDING:
      return true;
      break;
    default:
      return false;
      break;
  }
}

packml_msgs::Status initStatus(std::string node_name)
{
  packml_msgs::Status status;
  status.header.frame_id = node_name;
  status.state.val = packml_msgs::State::UNDEFINED;
  status.sub_state = packml_msgs::State::UNDEFINED;
  status.mode.val = packml_msgs::Mode::UNDEFINED;
  // TODO: need better definition on errors
  status.error = 0;
  status.sub_error = 0;
  return status;
}

}  // namespace packml_msgs

#ifndef UTILS_H
#define UTILS_H

#endif  // UTILS_H
