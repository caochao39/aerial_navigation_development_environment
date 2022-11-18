/*
 * Copyright (c) 2020, Open Source Robotics Foundation.
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
 *     * Neither the name of the copyright holder nor the names of its
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

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <SDL.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>

#include "joy/joy.hpp"

namespace joy
{

Joy::Joy(const rclcpp::NodeOptions & options)
: rclcpp::Node("joy_node", options)
{
  dev_id_ = static_cast<int>(this->declare_parameter("device_id", 0));

  dev_name_ = this->declare_parameter("device_name", std::string(""));

  // The user specifies the deadzone to us in the range of 0.0 to 1.0.  Later on
  // we'll convert that to the range of 0 to 32767.  Note also that negatives
  // are not allowed, as this is a +/- value.
  scaled_deadzone_ = this->declare_parameter("deadzone", 0.05);
  if (scaled_deadzone_ < 0.0 || scaled_deadzone_ > 1.0) {
    throw std::runtime_error("Deadzone must be between 0.0 and 1.0");
  }
  unscaled_deadzone_ = 32767.0 * scaled_deadzone_;
  // According to the SDL documentation, this always returns a value between
  // -32768 and 32767.  However, we want to report a value between -1.0 and 1.0,
  // hence the "scale" dividing by 32767.  Also note that SDL returns the axes
  // with "forward" and "left" as negative.  This is opposite to the ROS
  // conventionof "forward" and "left" as positive, so we invert the axes here
  // as well.  Finally, we take into account the amount of deadzone so we truly
  // do get value between -1.0 and 1.0 (and not -deadzone to +deadzone).
  scale_ = static_cast<float>(-1.0 / (1.0 - scaled_deadzone_) / 32767.0);

  autorepeat_rate_ = this->declare_parameter("autorepeat_rate", 20.0);
  if (autorepeat_rate_ < 0.0) {
    throw std::runtime_error("Autorepeat rate must be >= 0.0");
  } else if (autorepeat_rate_ > 1000.0) {
    throw std::runtime_error("Autorepeat rate must be <= 1000.0");
  } else if (autorepeat_rate_ > 0.0) {
    autorepeat_interval_ms_ = static_cast<int>(1000.0 / autorepeat_rate_);
  } else {
    // If the autorepeat rate is set to 0, the user doesn't want us to
    // publish unless an event happens.  We still wake up every 200
    // milliseconds to check if we need to quit.
    autorepeat_interval_ms_ = 200;
  }

  sticky_buttons_ = this->declare_parameter("sticky_buttons", false);

  coalesce_interval_ms_ = static_cast<int>(this->declare_parameter("coalesce_interval_ms", 1));
  if (coalesce_interval_ms_ < 0) {
    throw std::runtime_error("coalesce_interval_ms must be positive");
  }
  // Make sure to initialize publish_soon_time regardless of whether we are going
  // to use it; this ensures that we are always using the correct time source.
  publish_soon_time_ = this->now();

  pub_ = create_publisher<sensor_msgs::msg::Joy>("joy", 10);

  feedback_sub_ = this->create_subscription<sensor_msgs::msg::JoyFeedback>(
    "joy/set_feedback", rclcpp::QoS(10), std::bind(
      &Joy::feedbackCb, this,
      std::placeholders::_1));

  future_ = exit_signal_.get_future();

  // In theory we could do this with just a timer, which would simplify the code
  // a bit.  But then we couldn't react to "immediate" events, so we stick with
  // the thread.
  event_thread_ = std::thread(&Joy::eventThread, this);
}

Joy::~Joy()
{
  exit_signal_.set_value();
  event_thread_.join();
  if (haptic_ != nullptr) {
    SDL_HapticClose(haptic_);
  }
  if (joystick_ != nullptr) {
    SDL_JoystickClose(joystick_);
  }
  SDL_Quit();
}

void Joy::feedbackCb(const std::shared_ptr<sensor_msgs::msg::JoyFeedback> msg)
{
  if (haptic_ == nullptr) {
    // No ability to do feedback, so ignore.
    return;
  }

  if (msg->type != sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE) {
    // We only support rumble
    return;
  }

  if (msg->id != 0) {
    // There can be only one (rumble)
    return;
  }

  if (msg->intensity < 0.0 || msg->intensity > 1.0) {
    // We only accept intensities between 0 and 1.
    return;
  }

  // We purposely ignore the return value; if it fails, what can we do?
  SDL_HapticRumblePlay(haptic_, msg->intensity, 1000);
}

float Joy::convertRawAxisValueToROS(int16_t val)
{
  // SDL reports axis values between -32768 and 32767.  To make sure
  // we report out scaled value between -1.0 and 1.0, we add one to
  // the value iff it is exactly -32768.  This makes all of the math
  // below work properly.
  if (val == -32768) {
    val = -32767;
  }

  // Note that we do all of the math in double space below.  This ensures
  // that the values stay between -1.0 and 1.0.
  double double_val = static_cast<double>(val);
  // Apply the deadzone semantic here.  This allows the deadzone
  // to be "smooth".
  if (double_val > unscaled_deadzone_) {
    double_val -= unscaled_deadzone_;
  } else if (double_val < -unscaled_deadzone_) {
    double_val += unscaled_deadzone_;
  } else {
    double_val = 0.0;
  }

  return static_cast<float>(double_val * scale_);
}

bool Joy::handleJoyAxis(const SDL_Event & e)
{
  bool publish = false;

  if (e.jaxis.which != joystick_instance_id_) {
    return publish;
  }

  if (e.jaxis.axis >= joy_msg_.axes.size()) {
    RCLCPP_WARN(get_logger(), "Saw axes too large for this device, ignoring");
    return publish;
  }

  float last_axis_value = joy_msg_.axes.at(e.jaxis.axis);
  joy_msg_.axes.at(e.jaxis.axis) = convertRawAxisValueToROS(e.jaxis.value);
  if (last_axis_value != joy_msg_.axes.at(e.jaxis.axis)) {
    if (coalesce_interval_ms_ > 0 && !publish_soon_) {
      publish_soon_ = true;
      publish_soon_time_ = this->now();
    } else {
      rclcpp::Duration time_since_publish_soon = this->now() - publish_soon_time_;
      if (time_since_publish_soon.nanoseconds() >= RCL_MS_TO_NS(coalesce_interval_ms_)) {
        publish = true;
        publish_soon_ = false;
      }
    }
  }
  // else no change, so don't publish

  return publish;
}

bool Joy::handleJoyButtonDown(const SDL_Event & e)
{
  bool publish = false;

  if (e.jbutton.which != joystick_instance_id_) {
    return publish;
  }

  if (e.jbutton.button >= joy_msg_.buttons.size()) {
    RCLCPP_WARN(get_logger(), "Saw button too large for this device, ignoring");
    return publish;
  }

  if (sticky_buttons_) {
    // For sticky buttons, invert 0 -> 1 or 1 -> 0
    joy_msg_.buttons.at(e.jbutton.button) = 1 - joy_msg_.buttons.at(e.jbutton.button);
  } else {
    joy_msg_.buttons.at(e.jbutton.button) = 1;
  }
  publish = true;

  return publish;
}

bool Joy::handleJoyButtonUp(const SDL_Event & e)
{
  bool publish = false;

  if (e.jbutton.which != joystick_instance_id_) {
    return publish;
  }

  if (e.jbutton.button >= joy_msg_.buttons.size()) {
    RCLCPP_WARN(get_logger(), "Saw button too large for this device, ignoring");
    return publish;
  }

  if (!sticky_buttons_) {
    joy_msg_.buttons.at(e.jbutton.button) = 0;
    publish = true;
  }

  return publish;
}

bool Joy::handleJoyHatMotion(const SDL_Event & e)
{
  bool publish = false;

  if (e.jhat.which != joystick_instance_id_) {
    return publish;
  }

  // The hats are the last axes in the axes list.  There are two axes per hat;
  // the first of the pair is for left (positive) and right (negative), while
  // the second of the pair is for up (positive) and down (negative).

  // Determine which pair we are based on e.jhat.hat
  int num_axes = SDL_JoystickNumAxes(joystick_);
  if (num_axes < 0) {
    RCLCPP_WARN(get_logger(), "Failed to get axes: %s", SDL_GetError());
    return publish;
  }
  size_t axes_start_index = num_axes + e.jhat.hat * 2;
  // Note that we check axes_start_index + 1 here to ensure that we can write to
  // either the left/right axis or the up/down axis that corresponds to this hat.
  if ((axes_start_index + 1) >= joy_msg_.axes.size()) {
    RCLCPP_WARN(get_logger(), "Saw hat too large for this device, ignoring");
    return publish;
  }

  if (e.jhat.value & SDL_HAT_LEFT) {
    joy_msg_.axes.at(axes_start_index) = 1.0;
  }
  if (e.jhat.value & SDL_HAT_RIGHT) {
    joy_msg_.axes.at(axes_start_index) = -1.0;
  }
  if (e.jhat.value & SDL_HAT_UP) {
    joy_msg_.axes.at(axes_start_index + 1) = 1.0;
  }
  if (e.jhat.value & SDL_HAT_DOWN) {
    joy_msg_.axes.at(axes_start_index + 1) = -1.0;
  }
  if (e.jhat.value == SDL_HAT_CENTERED) {
    joy_msg_.axes.at(axes_start_index) = 0.0;
    joy_msg_.axes.at(axes_start_index + 1) = 0.0;
  }
  publish = true;

  return publish;
}

void Joy::handleJoyDeviceAdded(const SDL_Event & e)
{
  if (!dev_name_.empty()) {
    int num_joysticks = SDL_NumJoysticks();
    if (num_joysticks < 0) {
      RCLCPP_WARN(get_logger(), "Failed to get the number of joysticks: %s", SDL_GetError());
      return;
    }
    bool matching_device_found = false;
    for (int i = 0; i < num_joysticks; ++i) {
      const char * name = SDL_JoystickNameForIndex(i);
      if (name == nullptr) {
        RCLCPP_WARN(get_logger(), "Could not get joystick name: %s", SDL_GetError());
        continue;
      }
      if (std::string(name) == dev_name_) {
        // We found it!
        matching_device_found = true;
        dev_id_ = i;
        break;
      }
    }
    if (!matching_device_found) {
      RCLCPP_WARN(
        get_logger(), "Could not get joystick with name %s: %s",
        dev_name_.c_str(), SDL_GetError());
      return;
    }
  }

  if (e.jdevice.which != dev_id_) {
    return;
  }

  joystick_ = SDL_JoystickOpen(dev_id_);
  if (joystick_ == nullptr) {
    RCLCPP_WARN(get_logger(), "Unable to open joystick %d: %s", dev_id_, SDL_GetError());
    return;
  }

  // We need to hold onto this so that we can properly remove it on a
  // remove event.
  joystick_instance_id_ = SDL_JoystickGetDeviceInstanceID(dev_id_);
  if (joystick_instance_id_ < 0) {
    RCLCPP_WARN(get_logger(), "Failed to get instance ID for joystick: %s", SDL_GetError());
    SDL_JoystickClose(joystick_);
    joystick_ = nullptr;
    return;
  }

  int num_buttons = SDL_JoystickNumButtons(joystick_);
  if (num_buttons < 0) {
    RCLCPP_WARN(get_logger(), "Failed to get number of buttons: %s", SDL_GetError());
    SDL_JoystickClose(joystick_);
    joystick_ = nullptr;
    return;
  }
  joy_msg_.buttons.resize(num_buttons);

  int num_axes = SDL_JoystickNumAxes(joystick_);
  if (num_axes < 0) {
    RCLCPP_WARN(get_logger(), "Failed to get number of axes: %s", SDL_GetError());
    SDL_JoystickClose(joystick_);
    joystick_ = nullptr;
    return;
  }
  int num_hats = SDL_JoystickNumHats(joystick_);
  if (num_hats < 0) {
    RCLCPP_WARN(get_logger(), "Failed to get number of hats: %s", SDL_GetError());
    SDL_JoystickClose(joystick_);
    joystick_ = nullptr;
    return;
  }
  joy_msg_.axes.resize(num_axes + num_hats * 2);

  // Get the initial state for each of the axes
  for (int i = 0; i < num_axes; ++i) {
    int16_t state;
    if (SDL_JoystickGetAxisInitialState(joystick_, i, &state)) {
      joy_msg_.axes.at(i) = convertRawAxisValueToROS(state);
    }
  }

  haptic_ = SDL_HapticOpenFromJoystick(joystick_);
  if (haptic_ != nullptr) {
    if (SDL_HapticRumbleInit(haptic_) < 0) {
      // Failed to init haptic.  Clean up haptic_.
      SDL_HapticClose(haptic_);
      haptic_ = nullptr;
    }
  } else {
    RCLCPP_INFO(get_logger(), "No haptic (rumble) available, skipping initialization");
  }

  RCLCPP_INFO(
    get_logger(), "Opened joystick: %s.  deadzone: %f",
    SDL_JoystickName(joystick_), scaled_deadzone_);
}

void Joy::handleJoyDeviceRemoved(const SDL_Event & e)
{
  if (e.jdevice.which != joystick_instance_id_) {
    return;
  }

  joy_msg_.buttons.resize(0);
  joy_msg_.axes.resize(0);
  if (haptic_ != nullptr) {
    SDL_HapticClose(haptic_);
    haptic_ = nullptr;
  }
  if (joystick_ != nullptr) {
    SDL_JoystickClose(joystick_);
    joystick_ = nullptr;
  }
}

void Joy::eventThread()
{
  std::future_status status;
  rclcpp::Time last_publish = this->now();

  do {
    if (joystick_ == nullptr) {
      if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC) < 0) {
        throw std::runtime_error("SDL could not be initialized: " + std::string(SDL_GetError()));
      }
    }

    bool should_publish = false;
    SDL_Event e;
    int wait_time_ms = autorepeat_interval_ms_;
    if (publish_soon_) {
      wait_time_ms = std::min(wait_time_ms, coalesce_interval_ms_);
    }
    int success = SDL_WaitEventTimeout(&e, wait_time_ms);
    if (success == 1) {
      // Succeeded getting an event
      if (e.type == SDL_JOYAXISMOTION) {
        should_publish = handleJoyAxis(e);
      } else if (e.type == SDL_JOYBUTTONDOWN) {
        should_publish = handleJoyButtonDown(e);
      } else if (e.type == SDL_JOYBUTTONUP) {
        should_publish = handleJoyButtonUp(e);
      } else if (e.type == SDL_JOYHATMOTION) {
        should_publish = handleJoyHatMotion(e);
      } else if (e.type == SDL_JOYDEVICEADDED) {
        handleJoyDeviceAdded(e);
      } else if (e.type == SDL_JOYDEVICEREMOVED) {
        handleJoyDeviceRemoved(e);
      } else {
        RCLCPP_INFO(get_logger(), "Unknown event type %d", e.type);
      }
    } else {
      // We didn't succeed, either because of a failure or because of a timeout.
      // If we are autorepeating and enough time has passed, set should_publish.
      rclcpp::Time now = this->now();
      rclcpp::Duration diff_since_last_publish = now - last_publish;
      if ((autorepeat_rate_ > 0.0 &&
        RCL_NS_TO_MS(diff_since_last_publish.nanoseconds()) >= autorepeat_interval_ms_) ||
        publish_soon_)
      {
        last_publish = now;
        should_publish = true;
        publish_soon_ = false;
      }
    }

    if (joystick_ != nullptr && should_publish) {
      joy_msg_.header.frame_id = "joy";
      joy_msg_.header.stamp = this->now();

      pub_->publish(joy_msg_);
    }

    if (joystick_ == nullptr) {
      SDL_Quit();
    }

    status = future_.wait_for(std::chrono::seconds(0));
  } while (status == std::future_status::timeout);
}

}  // namespace joy

RCLCPP_COMPONENTS_REGISTER_NODE(joy::Joy)
