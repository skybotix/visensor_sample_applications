vi_sensor_interface
===================

Simple example on how to interface the VI-Sensor and display the images using openCV

Build instructions
==================
Download libvisensor and install it if you have not done that before

<pre><code>
git clone https://github.com/ethz-asl/libvisensor.git
cd libvisensor
./install_libvisensor.sh
</code></pre>

Now, were building the interface

<pre><code>
cd vi_sensor_interface
mkdir build
cd build
cmake ..
make
cd ../bin
./vi_sensor_interface
</code></pre>

