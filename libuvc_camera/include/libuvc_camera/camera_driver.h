#pragma once

#include <libuvc/libuvc.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/thread/mutex.hpp>

#include <libuvc_camera/UVCCameraConfig.h>

namespace libuvc_camera {

class CameraDriver {
public:
  CameraDriver(ros::NodeHandle nh, ros::NodeHandle priv_nh);
  ~CameraDriver();

  bool Start();
  void Stop();

private:
  enum State {
    kInitial = 0,
    kStopped = 1,
    kRunning = 2,
  };

  // Flags controlling whether the sensor needs to be stopped (or reopened) when changing settings
  static const int kReconfigureClose = 3; // Need to close and reopen sensor to change this setting
  static const int kReconfigureStop = 1; // Need to stop the stream before changing this setting
  static const int kReconfigureRunning = 0; // We can change this setting without stopping the stream

  void OpenCamera(UVCCameraConfig &new_config);
  void CloseCamera();

  // Accept a reconfigure request from a client
  void ReconfigureCallback(UVCCameraConfig &config, uint32_t level);
  enum uvc_frame_format GetVideoMode(std::string vmode);
  // Accept changes in values of automatically updated controls
  void AutoControlsCallback(enum uvc_status_class status_class,
                            int event,
                            int selector,
                            enum uvc_status_attribute status_attribute,
                            void *data, size_t data_len);
  static void AutoControlsCallbackAdapter(enum uvc_status_class status_class,
                                          int event,
                                          int selector,
                                          enum uvc_status_attribute status_attribute,
                                          void *data, size_t data_len,
                                          void *ptr);
  // Accept a new image frame from the camera
  void ImageCallback(uvc_frame_t *frame);
  static void ImageCallbackAdapter(uvc_frame_t *frame, void *ptr);
  void setupCameraInfo(UVCCameraConfig &new_config);


  int zedSetGeneric(uint8_t item, uint8_t camera, uint8_t *data);
  int zedSetAutoExp(bool autoexp);
  int zedSetGain(uint16_t gain);
  int zedSetExposure(uint16_t exposure);

  ros::NodeHandle nh_, priv_nh_;

  State state_;
  boost::recursive_mutex mutex_;

  uvc_context_t *ctx_;
  uvc_device_t *dev_;
  uvc_device_handle_t *devh_;
  uvc_frame_t *rgb_frame_;

  image_transport::ImageTransport it_;
  image_transport::CameraPublisher cam_pub_;
  image_transport::CameraPublisher cam_pub_right_;

  dynamic_reconfigure::Server<UVCCameraConfig> config_server_;
  UVCCameraConfig config_;
  bool config_changed_;

  camera_info_manager::CameraInfoManager cinfo_manager_;
  camera_info_manager::CameraInfoManager cinfo_manager_right_;

  bool is_zed_camera_;
  bool is_stereo_;

  sensor_msgs::Image image_;
  sensor_msgs::Image image_right_;
};

};
