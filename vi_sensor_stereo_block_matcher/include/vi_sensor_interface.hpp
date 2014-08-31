/*
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
 *
 * All rights reserved.
 *
 * Redistribution and non-commercial use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of the {organization} nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <vector>
#include <iostream>
#include <ctime>
#include <fstream>      
#include <chrono>

#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/filesystem.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <visensor/visensor.hpp>

class ViSensorInterface {
 public:
  ViSensorInterface();
  ViSensorInterface(uint32_t image_rate, uint32_t imu_rate);
  ~ViSensorInterface();
  void run(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  void StartIntegratedSensor(uint32_t image_rate, uint32_t imu_rate);
  void ImageCallback(visensor::ViFrame::Ptr frame_ptr);
  void ImuCallback(boost::shared_ptr<visensor::ViImuMsg> imu_ptr);
  void process_data();
  bool computeRectificationMaps(void);

  visensor::ViSensorDriver drv_;
  typedef std::deque<visensor::ViFrame::Ptr> ViFrameQueue;
  static const visensor::SensorId::SensorId idxCam0_ = visensor::SensorId::CAM0;
  static const visensor::SensorId::SensorId idxCam1_ = visensor::SensorId::CAM1;

  ViFrameQueue frameQueue[4];

  boost::mutex io_mutex_;
  bool computed_rectification_map_;
  cv::Mat map00_, map01_, map10_, map11_;
  cv::StereoBM bm_;
};
