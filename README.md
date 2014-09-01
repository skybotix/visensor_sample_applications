visensor_sample_applications
===================

This repo contains a few sample standalone applications for the VI-Sensor

* **vi_sensor_interface**: A very basic interface to the VI-Sensor. Receives images and IMU messages and displays it using openCV.
* **vi_sensor_stereo_block_matcher**: Applies rectification to Vi-Sensor images and subsequently computes the disparity image using standard openCV block matcher. Intrinsic & extrinsic calibration parameters for rectification are downloaded from VI-Sensor.
