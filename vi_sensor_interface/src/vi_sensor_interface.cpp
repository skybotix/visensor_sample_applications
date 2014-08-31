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
#include <vi_sensor_interface.hpp>
#include <unistd.h>
ViSensorInterface::ViSensorInterface(uint32_t image_rate, uint32_t imu_rate)
    : vi_sensor_connected_(false) {
  StartIntegratedSensor(image_rate, imu_rate);
}

ViSensorInterface::ViSensorInterface()
    : vi_sensor_connected_(false) {
  StartIntegratedSensor(20, 200);
}

ViSensorInterface::~ViSensorInterface() {

}

void ViSensorInterface::StartIntegratedSensor(uint32_t image_rate, uint32_t imu_rate) {

  if (image_rate > 30) {
    image_rate = 30;
    std::cout << "Desired image rate is too hight, setting it to 30 Hz." << std::endl;
  }
  if (imu_rate > 800) {
    imu_rate = 800;
    std::cout << "Desired imu rate is too hight, setting it to 800 Hz." << std::endl;
  }

  try {
    drv_.init();
  } catch (visensor::exceptions::ConnectionException const &ex) {
    std::cout << ex.what() << "\n";
    return;
  }

  // set callback for image messages
  drv_.setCameraCallback(boost::bind(&ViSensorInterface::ImageCallback, this, _1));

  // set callback for IMU messages
  drv_.setImuCallback(boost::bind(&ViSensorInterface::ImuCallback, this, _1));

  drv_.startAllCameras(image_rate);
  drv_.startAllImus(imu_rate);
  vi_sensor_connected_ = true;

  boost::thread drawWorker_0(boost::bind(&ViSensorInterface::worker, this, 0));
  boost::thread drawWorker_1(boost::bind(&ViSensorInterface::worker, this, 1));
  boost::thread drawWorker_2(boost::bind(&ViSensorInterface::worker, this, 2));
  boost::thread drawWorker_3(boost::bind(&ViSensorInterface::worker, this, 3));

  drawWorker_0.join();
  drawWorker_1.join();
  drawWorker_2.join();
  drawWorker_3.join();
}

void ViSensorInterface::ImageCallback(visensor::ViFrame::Ptr frame_ptr) {
  uint32_t camera_id = frame_ptr->camera_id;
	//push image on queue
  frameQueue[camera_id].push(frame_ptr);
}

void ViSensorInterface::ImuCallback(boost::shared_ptr<visensor::ViImuMsg> imu_ptr) {
  uint32_t timeNSec = imu_ptr->timestamp;
  Eigen::Vector3d gyro(imu_ptr->gyro[0], imu_ptr->gyro[1], imu_ptr->gyro[2]);
  Eigen::Vector3d acc(imu_ptr->acc[0], imu_ptr->acc[1], imu_ptr->acc[2]);
	//Do stuff with IMU...
}

void ViSensorInterface::worker(unsigned int cam_id) {
  while (1) {
		//Popping image from queue. If no image available, we perform a blocking wait
    visensor::ViFrame::Ptr frame = frameQueue[cam_id].pop();
    uint32_t camera_id = frame->camera_id;
    cv::Mat image;
    image.create(frame->height, frame->width, CV_8UC1);
    memcpy(image.data, frame->getImageRawPtr(), frame->height * frame->width);
    //update window with image
    char winName[255];
    boost::mutex::scoped_lock lock(io_mutex_);  //lock thread as opencv does seem to have problems with multithreading
    sprintf(winName, "Camera %u", camera_id);
    cv::namedWindow(winName, CV_WINDOW_AUTOSIZE);
    cv::imshow(winName, image);
    cv::waitKey(1);
  }
}
