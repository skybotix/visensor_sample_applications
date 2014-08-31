vi_sensor_interface
===================

OpenCV interface for VI sensor

Build instructions
==================
Download libvisensor. If you do not have access, ask sammy.omari@skybotix.com .

<pre><code>cd libvisensor
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
</code></pre>

Now, were building the opencv interface to the vi sensor driver

<pre><code>cd vi_sensor_interface
mkdir build
cd build
cmake ..
make
cd ../bin
./vi_sensor_interface
</code></pre>
