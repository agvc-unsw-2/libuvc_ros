/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2012 Ken Tossell
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include "libuvc_camera/camera_driver.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <image_transport/camera_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <libuvc/libuvc.h>
#include <yaml-cpp/yaml.h>
#include <unistd.h>

namespace libuvc_camera {

CameraDriver::CameraDriver(ros::NodeHandle nh, ros::NodeHandle priv_nh)
  : nh_(nh), priv_nh_(priv_nh),
    state_(kInitial),
    ctx_(NULL), dev_(NULL), devh_(NULL), rgb_frame_(NULL),
    it_(nh_),
    config_server_(mutex_, priv_nh_),
    config_changed_(false),
    cinfo_manager_(nh), cinfo_manager_right_(nh) {
}

CameraDriver::~CameraDriver() {
  if (rgb_frame_)
    uvc_free_frame(rgb_frame_);

  if (ctx_)
    uvc_exit(ctx_);  // Destroys dev_, devh_, etc.
}

bool CameraDriver::Start() {
  assert(state_ == kInitial);

  uvc_error_t err;

  err = uvc_init(&ctx_, NULL);

  if (err != UVC_SUCCESS) {
    uvc_perror(err, "ERROR: uvc_init");
    return false;
  }

  state_ = kStopped;

  config_server_.setCallback(boost::bind(&CameraDriver::ReconfigureCallback, this, _1, _2));

  return state_ == kRunning;
}

void CameraDriver::Stop() {
  boost::recursive_mutex::scoped_lock(mutex_);

  assert(state_ != kInitial);

  if (state_ == kRunning)
    CloseCamera();

  assert(state_ == kStopped);

  uvc_exit(ctx_);
  ctx_ = NULL;

  state_ = kInitial;
}

void CameraDriver::setupCameraInfo(UVCCameraConfig &new_config)
{
	std::string left_url, right_url;

	if (new_config.camera_match_url != "") {
		ROS_INFO_STREAM("Opening file " << new_config.camera_match_url);
		YAML::Node config = YAML::LoadFile(new_config.camera_match_url);
		std::string hostname, serial;

		std::stringstream ss;
		ss << "hostname" << " 2>&1";
		std::string cmd = ss.str();

		int buffer_size = 256;
		char buffer[buffer_size];
		FILE *stream = popen(cmd.c_str(), "r");
		if (stream) {
			while (!feof(stream)) {
				if (fgets(buffer, buffer_size, stream) != NULL) {
					hostname.append(buffer);
				}
			}
			pclose(stream);
			// any output should be an error
			if (hostname.length() > 0) {
				// remove the new line
				hostname.resize(hostname.size() - 1);
				ROS_WARN("Got hostname: %s", hostname.c_str());
			}
		} else {
			ROS_WARN("Could not get hostname");
		}

		if (config[hostname]) {
			serial = config[hostname].as<std::string>();
			ROS_INFO_STREAM("Using camera " << serial << " for machine " << hostname);
			left_url =  new_config.camera_info_url + serial + ".yaml";
			right_url = new_config.camera_info_url_right + serial + ".yaml";
		} else {
			ROS_ERROR_STREAM("Cannot find the serial number of the camera for this machine (" << hostname << ")");
		}
	} else {
		left_url = new_config.camera_info_url;
		right_url = new_config.camera_info_url_right;
	}
	
	ROS_INFO_STREAM("Camera cal files are " << left_url << " and " << right_url);

    cinfo_manager_.loadCameraInfo(left_url);
	if (is_stereo_) {
		cinfo_manager_right_.loadCameraInfo(right_url);

		cinfo_manager_.setCameraName("left_camera");
		cinfo_manager_right_.setCameraName("right_camera");
	} else {
		cinfo_manager_.setCameraName("camera");	
	}
}

#define ZED_wINDEX_GENERIC 768
#define ZED_wVALUE_GENERIC 0x200
#define ZED_ITEM_AUTOEXP 0x22
#define ZED_ITEM_GAIN 0x25
#define ZED_ITEM_EXPOSURE 0x24
#define ZED_LEFT 1
#define ZED_RIGHT 2
#define ZED_GENERIC_LENGTH 64
#define ZED_EXPOSURE_MIN 0xc0
#define ZED_EXPOSURE_MAX 0x1d40
#define ZED_GAIN_MIN 0x99
#define ZED_GAIN_MAX 0x7ff
#define ZED_LATENCY 0.040

int CameraDriver::zedSetGeneric(uint8_t item, uint8_t camera, uint8_t *data)
{
	uint8_t buf[ZED_GENERIC_LENGTH];
	buf[0] = 7;
	buf[1] = item;
	buf[2] = camera;
	memcpy(&buf[3], data, ZED_GENERIC_LENGTH - 3);

	return uvc_set_ctrl_generic(devh_, ZED_wVALUE_GENERIC, ZED_wINDEX_GENERIC,
			buf, ZED_GENERIC_LENGTH);
}

int CameraDriver::zedSetAutoExp(bool autoexp)
{
	uint8_t data[ZED_GENERIC_LENGTH] = {0};
	if (autoexp) {
		return zedSetGeneric(ZED_ITEM_AUTOEXP, 1, data);
	} else {
		zedSetGeneric(ZED_ITEM_AUTOEXP, 0, data);
		zedSetGeneric(0x26, 1, data);
		zedSetGeneric(0x27, 1, data);
		zedSetGeneric(0x26, 1, data);
	}
}

int CameraDriver::zedSetGain(uint16_t gain)
{
	if (gain < ZED_GAIN_MIN) {
		gain = ZED_GAIN_MIN;
	} else if (gain > ZED_GAIN_MAX) {
		gain = ZED_GAIN_MAX;
	}

	uint8_t data[ZED_GENERIC_LENGTH] = {0};
	data[0] = gain >> 8;
	data[1] = gain & 255;

	zedSetGeneric(ZED_ITEM_GAIN, ZED_LEFT, data);
	return zedSetGeneric(ZED_ITEM_GAIN, ZED_RIGHT, data);
}

int CameraDriver::zedSetExposure(uint16_t exposure)
{
	if (exposure < ZED_EXPOSURE_MIN) {
		exposure = ZED_EXPOSURE_MIN;
	} else if (exposure > ZED_EXPOSURE_MAX) {
		exposure = ZED_EXPOSURE_MAX;
	}

	uint8_t data[ZED_GENERIC_LENGTH] = {0};
	data[0] = exposure >> 8;
	data[1] = exposure & 255;

	zedSetGeneric(ZED_ITEM_EXPOSURE, ZED_LEFT, data);
	return zedSetGeneric(ZED_ITEM_EXPOSURE, ZED_RIGHT, data);
}

void CameraDriver::ReconfigureCallback(UVCCameraConfig &new_config, uint32_t level) {
  boost::recursive_mutex::scoped_lock(mutex_);

  if ((level & kReconfigureClose) == kReconfigureClose) {
    if (state_ == kRunning)
      CloseCamera();
  }

  if (state_ == kStopped) {
    OpenCamera(new_config);
  }

  if (new_config.camera_info_url != config_.camera_info_url ||
		  new_config.camera_info_url_right != config_.camera_info_url_right ||
		  new_config.camera_match_url != config_.camera_match_url) {
	  setupCameraInfo(new_config);
  }

  decimate_factor_ = new_config.decimate_factor;

  if (state_ == kRunning) {
#define PARAM_INT(name, fn, value) if (new_config.name != config_.name) { \
      int val = (value);                                                \
      if (uvc_set_##fn(devh_, val)) {                                   \
        ROS_WARN("Unable to set " #name " to %d", val);                 \
      }                                                                 \
    }

    PARAM_INT(brightness, brightness, new_config.brightness);
    PARAM_INT(contrast, contrast, new_config.contrast);
    PARAM_INT(saturation, saturation, new_config.saturation);
    PARAM_INT(hue, hue, new_config.hue);

	if (is_zed_camera_) {
		int r = 0;
		if (r = new_config.gain != config_.gain) {
			if (zedSetGain(new_config.gain)) {
				ROS_WARN("Unable to set gain to %d, error %d", new_config.gain, r);
			}
		}
		
		if (new_config.exposure_absolute != config_.exposure_absolute) {
			if (zedSetExposure(new_config.exposure_absolute * 1000)) {
				ROS_WARN("Unable to set exposure to %lf, error %d", new_config.exposure_absolute, r);
			}
		}

		if (new_config.auto_exposure != config_.auto_exposure) {
			if (zedSetAutoExp(new_config.auto_exposure ? 1 : 0)) {
				ROS_WARN("Unable to set auto_exposure to %d, error %d", new_config.auto_exposure, r);
			}
			usleep(1000000);
		}
	} else {
		PARAM_INT(auto_focus, focus_auto, new_config.auto_focus ? 1 : 0);
		PARAM_INT(focus_absolute, focus_abs, new_config.focus_absolute);
		PARAM_INT(iris_absolute, iris_abs, new_config.iris_absolute);
		PARAM_INT(scanning_mode, scanning_mode, new_config.scanning_mode);
		PARAM_INT(auto_exposure, ae_mode, 1 << new_config.auto_exposure);
		PARAM_INT(gain, gain, new_config.gain);
		PARAM_INT(exposure_absolute, exposure_abs, new_config.exposure_absolute * 10000);
		PARAM_INT(auto_exposure_priority, ae_priority, new_config.auto_exposure_priority);
	}

    /*if (new_config.pan_absolute != config_.pan_absolute || new_config.tilt_absolute != config_.tilt_absolute) {
      if (uvc_set_pantilt_abs(devh_, new_config.pan_absolute, new_config.tilt_absolute)) {
        ROS_WARN("Unable to set pantilt to %d, %d", new_config.pan_absolute, new_config.tilt_absolute);
        new_config.pan_absolute = config_.pan_absolute;
        new_config.tilt_absolute = config_.tilt_absolute;
      }
    }*/
    // TODO: roll_absolute
    // TODO: privacy
    // TODO: backlight_compensation
    // TODO: power_line_frequency
    // TODO: auto_hue
    // TODO: sharpness
    // TODO: gamma
    // TODO: auto_white_balance
    // TODO: white_balance_temperature
    // TODO: white_balance_BU
    // TODO: white_balance_RV
  }

  config_ = new_config;
}

void CameraDriver::ImageCallback(uvc_frame_t *frame) {
  decimate_counter_++;
  if (decimate_counter_ < decimate_factor_) {
    return;
  }
  decimate_counter_ = 0;

  ros::Time timestamp = ros::Time(frame->capture_time.tv_sec, frame->capture_time.tv_usec);
  if (is_zed_camera_) {
    timestamp = ros::Time::now() - ros::Duration(ZED_LATENCY);
  }

  boost::recursive_mutex::scoped_lock(mutex_);

  assert(state_ == kRunning);
  assert(rgb_frame_);

  uvc_frame_t *output_frame = frame;

  if (frame->frame_format == UVC_FRAME_FORMAT_BGR){
    image_.encoding = "bgr8";
  } else if (frame->frame_format == UVC_FRAME_FORMAT_RGB){
    image_.encoding = "rgb8";
  } else if (frame->frame_format == UVC_FRAME_FORMAT_UYVY) {
    image_.encoding = "yuv422";
  } else if (frame->frame_format == UVC_FRAME_FORMAT_YUYV) {
    uvc_error_t conv_ret = uvc_yuyv2bgr(frame, rgb_frame_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    image_.encoding = "bgr8";
	output_frame = rgb_frame_;
  } else if (frame->frame_format == UVC_FRAME_FORMAT_MJPEG) {
    // FIXME: uvc_any2bgr does not work on "mjpeg" format, so use uvc_mjpeg2rgb directly.
//    uvc_error_t conv_ret = uvc_mjpeg2rgb(frame, rgb_frame_);
    //if (conv_ret != UVC_SUCCESS) {
    //  uvc_perror(conv_ret, "Couldn't convert frame to RGB");
    //  return;
    //}
    //image->encoding = "rgb8";
    //memcpy(&(image->data[0]), rgb_frame_->data, rgb_frame_->data_bytes);
  } else {
    uvc_error_t conv_ret = uvc_any2bgr(frame, rgb_frame_);
    if (conv_ret != UVC_SUCCESS) {
      uvc_perror(conv_ret, "Couldn't convert frame to RGB");
      return;
    }
    image_.encoding = "bgr8";
	output_frame = rgb_frame_;
  }

  if (!is_stereo_) {
	  memcpy(&(image_.data[0]), output_frame->data, output_frame->data_bytes);
  } else {
	uint8_t *ip = (uint8_t *)output_frame->data;
	for (int i = 0; i < image_.height; ++i) {
		// Swap the left and right frames because of reasons...
		memcpy(&image_right_.data[i * image_right_.step], ip += image_right_.step, image_right_.step);
		memcpy(&image_.data[i * image_.step], ip += image_.step, image_.step);
	}

	image_right_.encoding = image_.encoding;
  }

  sensor_msgs::CameraInfo::Ptr cinfo(
    new sensor_msgs::CameraInfo(cinfo_manager_.getCameraInfo()));

  image_.header.frame_id = config_.frame_id;
  image_.header.stamp = timestamp;
  cinfo->header.frame_id = config_.frame_id;
  cinfo->header.stamp = timestamp;

  cam_pub_.publish(image_, *cinfo);
  if (is_stereo_) {
	sensor_msgs::CameraInfo::Ptr cinfo_right(
	  new sensor_msgs::CameraInfo(cinfo_manager_right_.getCameraInfo()));

	image_right_.header.frame_id = config_.frame_id;
	image_right_.header.stamp = timestamp;
	cinfo_right->header.frame_id = config_.frame_id;
	cinfo_right->header.stamp = timestamp;
	
	cam_pub_right_.publish(image_right_, *cinfo_right);
  }

  if (config_changed_) {
    config_server_.updateConfig(config_);
    config_changed_ = false;
  }
}

/* static */ void CameraDriver::ImageCallbackAdapter(uvc_frame_t *frame, void *ptr) {
  CameraDriver *driver = static_cast<CameraDriver*>(ptr);

  driver->ImageCallback(frame);
}

void CameraDriver::AutoControlsCallback(
  enum uvc_status_class status_class,
  int event,
  int selector,
  enum uvc_status_attribute status_attribute,
  void *data, size_t data_len) {
  boost::recursive_mutex::scoped_lock(mutex_);

  printf("Controls callback. class: %d, event: %d, selector: %d, attr: %d, data_len: %lu\n",
         status_class, event, selector, status_attribute, data_len);

  if (status_attribute == UVC_STATUS_ATTRIBUTE_VALUE_CHANGE) {
    switch (status_class) {
    case UVC_STATUS_CLASS_CONTROL_CAMERA: {
      switch (selector) {
      case UVC_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL:
        uint8_t *data_char = (uint8_t*) data;
        uint32_t exposure_int = ((data_char[0]) | (data_char[1] << 8) |
                                 (data_char[2] << 16) | (data_char[3] << 24));
        config_.exposure_absolute = exposure_int * 0.0001;
        config_changed_ = true;
        break;
      }
      break;
    }
    case UVC_STATUS_CLASS_CONTROL_PROCESSING: {
      switch (selector) {
      case UVC_PU_WHITE_BALANCE_TEMPERATURE_CONTROL:
        uint8_t *data_char = (uint8_t*) data;
        config_.white_balance_temperature = 
          data_char[0] | (data_char[1] << 8);
        config_changed_ = true;
        break;
      }
      break;
    }
    }

    // config_server_.updateConfig(config_);
  }
}

/* static */ void CameraDriver::AutoControlsCallbackAdapter(
  enum uvc_status_class status_class,
  int event,
  int selector,
  enum uvc_status_attribute status_attribute,
  void *data, size_t data_len,
  void *ptr) {
  CameraDriver *driver = static_cast<CameraDriver*>(ptr);

  driver->AutoControlsCallback(status_class, event, selector,
                               status_attribute, data, data_len);
}

enum uvc_frame_format CameraDriver::GetVideoMode(std::string vmode){
  if(vmode == "uncompressed") {
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  } else if (vmode == "compressed") {
    return UVC_COLOR_FORMAT_COMPRESSED;
  } else if (vmode == "yuyv") {
    return UVC_COLOR_FORMAT_YUYV;
  } else if (vmode == "uyvy") {
    return UVC_COLOR_FORMAT_UYVY;
  } else if (vmode == "rgb") {
    return UVC_COLOR_FORMAT_RGB;
  } else if (vmode == "bgr") {
    return UVC_COLOR_FORMAT_BGR;
  } else if (vmode == "mjpeg") {
    return UVC_COLOR_FORMAT_MJPEG;
  } else if (vmode == "gray8") {
    return UVC_COLOR_FORMAT_GRAY8;
  } else {
    ROS_ERROR_STREAM("Invalid Video Mode: " << vmode);
    ROS_WARN_STREAM("Continue using video mode: uncompressed");
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  }
};

void CameraDriver::OpenCamera(UVCCameraConfig &new_config) {
  assert(state_ == kStopped);

  int vendor_id = strtol(new_config.vendor.c_str(), NULL, 0);
  int product_id = strtol(new_config.product.c_str(), NULL, 0);

  ROS_INFO("Opening camera with vendor=0x%x, product=0x%x, serial=\"%s\", index=%d",
           vendor_id, product_id, new_config.serial.c_str(), new_config.index);

  uvc_device_t **devs;

  uvc_error_t find_err = uvc_find_devices(
    ctx_, &devs,
    vendor_id,
    product_id,
    new_config.serial.empty() ? NULL : new_config.serial.c_str());

  if (find_err != UVC_SUCCESS) {
    uvc_perror(find_err, "uvc_find_device");
    return;
  }

  // select device by index
  dev_ = NULL;
  int dev_idx = 0;
  while (devs[dev_idx] != NULL) {
    if(dev_idx == new_config.index) {
      dev_ = devs[dev_idx];
    }
    else {
      uvc_unref_device(devs[dev_idx]);
    }

    dev_idx++;
  }

  if(dev_ == NULL) {
    ROS_ERROR("Unable to find device at index %d", new_config.index);
    return;
  }

  uvc_error_t open_err = uvc_open(dev_, &devh_);

  if (open_err != UVC_SUCCESS) {
    switch (open_err) {
    case UVC_ERROR_ACCESS:
#ifdef __linux__
      ROS_ERROR("Permission denied opening /dev/bus/usb/%03d/%03d",
                uvc_get_bus_number(dev_), uvc_get_device_address(dev_));
#else
      ROS_ERROR("Permission denied opening device %d on bus %d",
                uvc_get_device_address(dev_), uvc_get_bus_number(dev_));
#endif
      break;
    default:
#ifdef __linux__
      ROS_ERROR("Can't open /dev/bus/usb/%03d/%03d: %s (%d)",
                uvc_get_bus_number(dev_), uvc_get_device_address(dev_),
                uvc_strerror(open_err), open_err);
#else
      ROS_ERROR("Can't open device %d on bus %d: %s (%d)",
                uvc_get_device_address(dev_), uvc_get_bus_number(dev_),
                uvc_strerror(open_err), open_err);
#endif
      break;
    }

    uvc_unref_device(dev_);
    return;
  }

  if (vendor_id == 0x2b03 && product_id == 0xf580) {
	  is_zed_camera_ = true;
	  is_stereo_ = true;
  } else {
	  is_zed_camera_ = false;
	  is_stereo_ = false;
  }
  
  if (is_stereo_) {
	cam_pub_ = it_.advertiseCamera("left/image_raw", 1, false);
	cam_pub_right_ = it_.advertiseCamera("right/image_raw", 1, false);
  } else {
	cam_pub_ = it_.advertiseCamera("image_raw", 1, false);
  }

  uvc_set_status_callback(devh_, &CameraDriver::AutoControlsCallbackAdapter, this);

  uvc_stream_ctrl_t ctrl;
  uvc_error_t mode_err = uvc_get_stream_ctrl_format_size(
    devh_, &ctrl,
    GetVideoMode(new_config.video_mode),
    new_config.width, new_config.height,
    new_config.frame_rate);

  if (mode_err != UVC_SUCCESS) {
    uvc_perror(mode_err, "uvc_get_stream_ctrl_format_size");
    uvc_close(devh_);
    uvc_unref_device(dev_);
    ROS_ERROR("check video_mode/width/height/frame_rate are available");
    uvc_print_diag(devh_, NULL);
    return;
  }
  
  // Setup image message sizes
  if (is_stereo_) {
	  image_.width = new_config.width / 2;
	  image_.height = new_config.height;
	  image_.step = image_.width * 3;
	  image_.data.resize(image_.step * image_.height);
	  image_right_.width = new_config.width / 2;
	  image_right_.height = new_config.height;
	  image_right_.step = image_.width * 3;
	  image_right_.data.resize(image_.step * image_.height);
  } else {
	  image_.width = new_config.width;
	  image_.height = new_config.height;
	  image_.step = image_.width * 3;
	  image_.data.resize(image_.step * image_.height);
  }
  ROS_INFO_STREAM("image width " << image_.width << " height " << image_.height << " step " << image_.step);

  uvc_error_t stream_err = uvc_start_streaming(devh_, &ctrl, &CameraDriver::ImageCallbackAdapter, this, 0);

  if (stream_err != UVC_SUCCESS) {
    uvc_perror(stream_err, "uvc_start_streaming");
    uvc_close(devh_);
    uvc_unref_device(dev_);
    return;
  }

  if (rgb_frame_)
    uvc_free_frame(rgb_frame_);

  rgb_frame_ = uvc_allocate_frame(new_config.width * new_config.height * 3);
  ROS_INFO("Allocated %d bytes for frame", new_config.width * new_config.height * 3);
  assert(rgb_frame_);

  state_ = kRunning;
}

void CameraDriver::CloseCamera() {
  assert(state_ == kRunning);

  uvc_close(devh_);
  devh_ = NULL;

  uvc_unref_device(dev_);
  dev_ = NULL;

  state_ = kStopped;
}

};
