vi_sensor_stereo_block_matcher
===================

Simple example on how to compute a stereo disparity image using the VI-Sensor

Build instructions
==================
Download libvisensor and install it if you have not done that before

<pre><code>
git clone https://github.com/ethz-asl/libvisensor.git
cd libvisensor
./install_libvisensor.sh
</code></pre>

Now, were building the stereo block matcher

<pre><code>
cd vi_sensor_stereo_block_matcher
mkdir build
cd build
cmake ..
make
cd ../bin
./vi_sensor_stereo_block_matcher
</code></pre>
