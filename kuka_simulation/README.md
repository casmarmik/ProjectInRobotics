# Kuka simulation 

For this to work you need the [kuka_experimental repository](https://github.com/ros-industrial/kuka_experimental), which stores the kuka robot. So you should clone that into your ros workspace. 

All the workcell configuration for this project are set up in the kuka_workcell.xacro file. This is located in the config folder. 

To try it out you can launch the rviz demo like this, in your ros workspace
```bash
  roslaunch kuka_simulation demo.launch 
```
