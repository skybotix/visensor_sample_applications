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

ViSensorInterface::ViSensorInterface(uint32_t image_rate, uint32_t imu_rate)
    : computed_rectification_map_(false) {
  StartIntegratedSensor(image_rate, imu_rate);
}

ViSensorInterface::ViSensorInterface()
    : computed_rectification_map_(false) {

  StartIntegratedSensor(20, 200);
}

ViSensorInterface::~ViSensorInterface() {

}

void ViSensorInterface::run(void) {
  // set callback for completed frames
  drv_.setCameraCallback(boost::bind(&ViSensorInterface::ImageCallback, this, _1));

  // set callback for completed IMU messages
  drv_.setImuCallback(boost::bind(&ViSensorInterface::ImuCallback, this, _1));

  //Lets wait for the user to end program.
  //This is ok as program is callback-driven anyways
  boost::mutex m;
  boost::mutex::scoped_lock l(m);
  boost::condition_variable c;
  c.wait(l);
}

void ViSensorInterface::StartIntegratedSensor(uint32_t image_rate, uint32_t imu_rate) {

  if (image_rate > 30) {
    image_rate = 30;
    std::cout << "Desired image rate is too high, setting it to 30 Hz." << std::endl;
  }
  if (imu_rate > 800) {
    imu_rate = 800;
    std::cout << "Desired imu rate is too high, setting it to 800 Hz." << std::endl;
  }

  try {
    drv_.init();
  } catch (visensor::exceptions::ConnectionException const &ex) {
    std::cout << ex.what() << "\n";
    return;
  }

  std::cout << "Libvisensor version is " << visensor::LIBRARY_VERSION_MAJOR << "."
      << visensor::LIBRARY_VERSION_MINOR << "." << visensor::LIBRARY_VERSION_PATCH << std::endl;

  // load available calibration and configure sensors corresponding to it
  drv_.selectCameraCalibration();

  drv_.startAllCameras(image_rate);
  drv_.startAllImus(imu_rate);
}

void ViSensorInterface::ImageCallback(visensor::ViFrame::Ptr frame_ptr) {
  boost::mutex::scoped_lock lock(io_mutex_);  //lock thread as opencv does seem to have problems with multithreading
  uint32_t camera_id = frame_ptr->camera_id;
  frameQueue[camera_id].emplace_back(frame_ptr);
  process_data();
}

void ViSensorInterface::ImuCallback(boost::shared_ptr<visensor::ViImuMsg> imu_ptr) {
  Eigen::Vector3d gyro(imu_ptr->gyro[0], imu_ptr->gyro[1], imu_ptr->gyro[2]);
  Eigen::Vector3d acc(imu_ptr->acc[0], imu_ptr->acc[1], imu_ptr->acc[2]);
}

void ViSensorInterface::process_data() {

  //Lets sync image streams from cam0 & cam1
  if (frameQueue[idxCam0_].empty() || frameQueue[idxCam1_].empty())
    return;
  while (frameQueue[idxCam0_].front()->timestamp > frameQueue[idxCam1_].front()->timestamp) {
    frameQueue[idxCam1_].pop_front();
    if (frameQueue[idxCam0_].empty() || frameQueue[idxCam1_].empty())
      return;
  }
  while (frameQueue[idxCam0_].front()->timestamp < frameQueue[idxCam1_].front()->timestamp) {
    frameQueue[idxCam0_].pop_front();
    if (frameQueue[idxCam0_].empty() || frameQueue[idxCam1_].empty())
      return;
  }

  //If we arrive here, both queues have synced timestamps
  visensor::ViFrame::Ptr frame0 = frameQueue[idxCam0_].front();
  visensor::ViFrame::Ptr frame1 = frameQueue[idxCam1_].front();

  if (!computed_rectification_map_)
    computeRectificationMaps();

  cv::Mat img0, img1;
  img0.create(frame0->height, frame0->width, CV_8UC1);
  img0.data = frame0->getImageRawPtr();

  img1.create(frame1->height, frame1->width, CV_8UC1);
  img1.data = frame1->getImageRawPtr();

  cv::Mat img0rect, img1rect;
  cv::remap(img0, img0rect, map00_, map01_, cv::INTER_LINEAR);
  cv::remap(img1, img1rect, map10_, map11_, cv::INTER_LINEAR);

  cv::namedWindow("img_rect_cam0", CV_WINDOW_AUTOSIZE);
  cv::imshow("img_rect_cam0", img0rect);

  cv::Mat disp, disp8;
  bm_(img0rect, img1rect, disp);
  disp.convertTo(disp8, CV_8U);
  cv::namedWindow("disparity", CV_WINDOW_AUTOSIZE);
  cv::imshow("disparity", disp8);
  cv::waitKey(1);

  //We dont need the frames anymore, lets drop them
  frameQueue[idxCam0_].pop_front();
  frameQueue[idxCam1_].pop_front();
}

bool ViSensorInterface::computeRectificationMaps(void) {
  visensor::ViCameraCalibration camera_calibration_0, camera_calibration_1;
  try {
    visensor::ViCameraCalibration tmp;
    std::vector<visensor::ViCameraCalibration> calibrations;
    drv_.getSelectedCameraCalibration(&tmp, idxCam0_);
    if (tmp.lens_model_->type_ != visensor::ViCameraLensModel::LensModelTypes::RADTAN ||
        tmp.projection_model_->type_ != visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE) {
      std::cerr << "No radtan and pinhole calibration specified. Therefore the factory "
          "calibration is choosen for the ROS sensor message." << std::endl;
      calibrations = drv_.getCameraCalibrations(
          idxCam0_, 0, tmp.is_flipped_,
          visensor::ViCameraLensModel::LensModelTypes::RADTAN,
          visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE);
      if (calibrations.size() == 0) {
        std::cerr << "No corresponding Factory Calibration for cam left found" << std::endl;
        return false;
      }
      camera_calibration_0 = calibrations.front();
    }
    else {
      camera_calibration_0 = tmp;
    }
    drv_.getSelectedCameraCalibration(&tmp, idxCam1_);
    if (tmp.lens_model_->type_ != visensor::ViCameraLensModel::LensModelTypes::RADTAN ||
        tmp.projection_model_->type_ != visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE) {
      std::cerr << "No radtan and pinhole calibration specified. "
          "Therefore the factory calibration is choosen for the ROS sensor message." << std::endl;
      calibrations = drv_.getCameraCalibrations(
          idxCam1_, 0, tmp.is_flipped_,
          visensor::ViCameraLensModel::LensModelTypes::RADTAN,
          visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE);
      if (calibrations.size() == 0) {
        std::cerr << "No corresponding Factory Calibration for cam right found" << std::endl;
        return false;
      }
      camera_calibration_1 = calibrations.front();
    }
    else {
      camera_calibration_1 = tmp;
    }
  }
  catch (visensor::exceptions const &ex) {
    std::cerr << "Could not read the camera configuration of cameras "<< idxCam0_ << " and "
        << idxCam1_ << ". Exception: " <<  ex.what() << std::endl;
    return false;
  }

  int image_width = camera_calibration_0.resolution_[0];
  int image_height = camera_calibration_0.resolution_[1];;

  double c0[9];
  double d0[5];
  double r0[9];
  double p0[12];
  double rot0[9];
  double t0[3];

  double c1[9];
  double d1[5];
  double r1[9];
  double p1[12];
  double rot1[9];
  double t1[3];

  double r[9];
  double t[3];

  std::vector<double> lens_coefficients = camera_calibration_0.lens_model_->getCoefficients();
  std::fill(d0, d0 + 5, 0);
  for ( unsigned int i = 0; i < lens_coefficients.size(); ++i ) {
    d0[i] = lens_coefficients.at(i);
  }
  visensor::ViCameraProjectionModelPinhole::Ptr cam0_projection_model =
      camera_calibration_0.getProjectionModel<visensor::ViCameraProjectionModelPinhole>();
  c0[0] = cam0_projection_model->focal_length_u_;
  c0[1] = 0.0;
  c0[2] = cam0_projection_model->principal_point_u_;
  c0[3] = 0.0;
  c0[4] = cam0_projection_model->focal_length_v_;
  c0[5] = cam0_projection_model->principal_point_v_;
  c0[6] = 0.0;
  c0[7] = 0.0;
  c0[8] = 1.0;

  lens_coefficients = camera_calibration_1.lens_model_->getCoefficients();
  std::fill(d1, d1 + 5, 0);
  for ( unsigned int i = 0; i < lens_coefficients.size(); ++i ) {
    d1[i] = lens_coefficients.at(i);
  }
  visensor::ViCameraProjectionModelPinhole::Ptr cam1_projection_model =
      camera_calibration_1.getProjectionModel<visensor::ViCameraProjectionModelPinhole>();
  c1[0] = cam1_projection_model->focal_length_u_;
  c1[1] = 0.0;
  c1[2] = cam1_projection_model->principal_point_u_;
  c1[3] = 0.0;
  c1[4] = cam1_projection_model->focal_length_v_;
  c1[5] = cam1_projection_model->principal_point_v_;
  c1[6] = 0.0;
  c1[7] = 0.0;
  c1[8] = 1.0;

  for (int i = 0; i < 9; ++i) {
    rot0[i] = camera_calibration_0.R_[i];
    rot1[i] = camera_calibration_1.R_[i];
  }
  for (int i = 0; i < 3; ++i) {
    t0[i] = camera_calibration_0.t_[i];
    t1[i] = camera_calibration_1.t_[i];
  }

  Eigen::Map < Eigen::Matrix3d > RR0(rot0);
  Eigen::Map < Eigen::Vector3d > tt0(t0);
  Eigen::Map < Eigen::Matrix3d > RR1(rot1);
  Eigen::Map < Eigen::Vector3d > tt1(t1);

  Eigen::Matrix4d T0 = Eigen::Matrix4d::Zero();
  Eigen::Matrix4d T1 = Eigen::Matrix4d::Zero();

  T0.block<3, 3>(0, 0) = RR0;
  T0.block<3, 1>(0, 3) = tt0;
  T0(3, 3) = 1.0;
  T1.block<3, 3>(0, 0) = RR1;
  T1.block<3, 1>(0, 3) = tt1;
  T1(3, 3) = 1.0;

  Eigen::Matrix4d T_rel = Eigen::Matrix4d::Zero();
  T_rel = T1 * T0.inverse();

  Eigen::Map < Eigen::Matrix3d > R_rel(r);
  Eigen::Map < Eigen::Vector3d > t_rel(t);

  R_rel = T_rel.block<3, 3>(0, 0);
  t_rel << T_rel(0, 3), T_rel(1, 3), T_rel(2, 3);

  double r_temp[9];
  r_temp[0] = R_rel(0, 0);
  r_temp[1] = R_rel(0, 1);
  r_temp[2] = R_rel(0, 2);
  r_temp[3] = R_rel(1, 0);
  r_temp[4] = R_rel(1, 1);
  r_temp[5] = R_rel(1, 2);
  r_temp[6] = R_rel(2, 0);
  r_temp[7] = R_rel(2, 1);
  r_temp[8] = R_rel(2, 2);

  //cv::Mat wrapped(rows, cols, CV_32FC1, external_mem, CV_AUTOSTEP);
  cv::Mat C0(3, 3, CV_64FC1, c0, 3 * sizeof(double));
  cv::Mat D0(5, 1, CV_64FC1, d0, 1 * sizeof(double));
  cv::Mat R0(3, 3, CV_64FC1, r0, 3 * sizeof(double));
  cv::Mat P0(3, 4, CV_64FC1, p0, 4 * sizeof(double));

  cv::Mat C1(3, 3, CV_64FC1, c1, 3 * sizeof(double));
  cv::Mat D1(5, 1, CV_64FC1, d1, 1 * sizeof(double));
  cv::Mat R1(3, 3, CV_64FC1, r1, 3 * sizeof(double));
  cv::Mat P1(3, 4, CV_64FC1, p1, 4 * sizeof(double));

  cv::Mat R(3, 3, CV_64FC1, r_temp, 3 * sizeof(double));

  cv::Mat T(3, 1, CV_64FC1, t, 1 * sizeof(double));

  cv::Size img_size(image_width, image_height);

  cv::Rect roi1, roi2;
  cv::Mat Q;

  cv::stereoRectify(C0, D0, C1, D1, img_size, R, T, R0, R1, P0, P1, Q, cv::CALIB_ZERO_DISPARITY, 0,
                    img_size, &roi1, &roi2);

  cv::initUndistortRectifyMap(C0, D0, R0, P0, img_size, CV_16SC2, map00_, map01_);
  cv::initUndistortRectifyMap(C1, D1, R1, P1, img_size, CV_16SC2, map10_, map11_);

  computed_rectification_map_ = true;
  return true;
}
