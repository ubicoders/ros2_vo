 #colcon build --packages-select my_stereo_vo && source install/setup.bash 


## colcon build --packages-select stereo_visual_slam 

colcon build --packages-select ubicoders_svo --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source install/setup.bash 

#colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON  --symlink-install