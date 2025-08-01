# Cartographer Launcher for ROS 2
A simple tool can launch Cartographer SLAM and save the pgm format map
## How to use it?
1. Install Cartographer under ROS 2    
   ```
   sudo apt install ros-humble-cartographer
   sudo apt install ros-humble-cartographer-ros
   ```
2. Tools to save pgm format map  
   ```
   pip install opencv-python numpy
   ```
3. Clone this repo:
   ```
   git clone https://github.com/Bob-YsPan/cartographer_launcher_ros2.git cartographer_launcher
   ```
3. Turning the map resolution  
    **Note: some cheap lidar like YDLidar-X4 need to reduce the resolution to hither value like 0.08, to prevent the scan result too light to make map_saver ignort it!**  
   `src/cartographer_launcher/launch/cartographer.launch.py`  
   ```
   resolution = LaunchConfiguration('resolution', default='0.08')
   ```
   This part also needs to be adjusted to ensure the obstacle's darkness on the map
   `src/cartographer_launcher/config/cartographer_2d.lua`  
   ```
   TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.08
   ```
     
   You can turning others parameters like `src/cartographer_launcher/config/cartographer_2d.lua`, if you know how to fine-tuning it!  
4. Build package and source it
   ```
   cd ~/<your_ws>/
   colcon build --packages-select cartographer_launcher
   source ~/<your_ws>/install/setup.bash
   ```
## Scripts you can use
1. Launch the SLAM function:
    ```
    ros2 launch cartographer_launcher cartographer.launch.py
    ```
2. Launch the SLAM without default map resolution  
    (If you only change this resolution, the signal of obstacles will little lighter, also tuning the config in the Lua is recommended)
    ```
    ros2 launch cartographer_launcher cartographer.launch.py resolution:=0.01
    ```
3. Save the map when SLAM finished (Don't stop the `cartographer.launch.py`, save the map first!)
    ```
    cd ~/<your_ws's map location>
    ros2 run cartographer_launcher map_saver --ros-args -p filename:="map_name"
    ```
    The map file simillar like the version saved by `map_server`, but with the better details that can be use by navigation stack
## Refrences
* A tutorial to launch Cartographer on ROS 2  
  https://www.waveshare.com/wiki/Cartographer_Map_Building 
* A Chinese site provide some parameter can use  
  https://zhuanlan.zhihu.com/p/675717750
* Map saver for Cartographer under ROS 1  
  https://github.com/HaoQChen/map_server
* Some power for ChatGPT 🤖