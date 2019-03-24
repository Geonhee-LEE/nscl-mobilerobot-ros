# Structure

# Jetson installation

## ROS
https://www.jetsonhacks.com/2018/04/27/robot-operating-system-ros-on-nvidia-jetson-tx-development-kits/

## ZED
https://github.com/stereolabs/zed-ros-wrapper


# Requirement

<pre><code>
sudo apt-get install ros-kinetic-move-base -y 
sudo apt-get install ros-kinetic-gmapping -y 
</pre></code>

If you include the packages about navigation-kinetic-devel, you don't need you install below components

<pre><code>
sudo apt-get install ros-kinetic-amcl -y 
sudo apt-get install ros-kinetic-map-server -y 
sudo apt-get install ros-kinetic-dwa-local-planner -y 
sudo apt-get install ros-kinetic-tf2 -y 
sudo apt-get install ros-kinetic-tf2-ros -y
sudo apt-get install ros-kinetic-global-planner -y 
sudo apt-get install ros-kinetic-sensor-msgs -y 
sudo apt-get install ros-kinetic-rosserial-python -y
sudo apt-get install ros-kinetic-range-sensor-layer -y
sudo apt-get install ros-kinetic-diagnostic-updater -y
sudo apt-get install ros-kinetic-interactive-marker -y
sudo apt-get install ros-kinetic-gazebo-ros -y
</code></pre>

## SSH Setting

<pre><code>

sudo apt-get install ssh -y 

ssh remote_id@remote_ip

</code></pre>



