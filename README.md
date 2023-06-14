# VR_Assignment
## Installing 
First, there is the need of installing and building `AirSim` on Windows by running in your Windows prompt:
```
git clone https://github.com/Microsoft/AirSim.git
cd AirSim
build.cmd
```
Once it is done, go into the `AirSim` folder and modify the `setting.json` file so that it looks like this:  
```
{
    "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "ViewMode": "FlyWithMe",

    "Vehicles": {
      "Drone1": {
        "VehicleType": "SimpleFlight",
        "DefaultVehicleState": "Disarmed",
        "X": 94050.0,"Y": 73410.0,"Z": 612.0, "Yaw": 0
      },
      "Drone2": {
        "VehicleType": "SimpleFlight",
        "DefaultVehicleState": "Disarmed",
        "X": 94050.0,"Y": 73610.0,"Z": 612.0, "Yaw": 0
      }

    }
}
```
Since ROS works on Linux, open an Ubuntu terminal and follow these instructions to install [AirSim ROS Wrapper](https://microsoft.github.io/AirSim/airsim_ros_pkgs/).  
Then, to set the environment for the project, run on Ubuntu terminal:
```
echo "export WSL_HOST_IP=(your WSL IP address)" >> ⁓/.bashrc
sudo apt install python-is-python3
python3 -m pip install pymap3d
sudo pip install msgpack-rpc-python (SERVE???)
```
Finally, in your ROS workspace clone this [repository](https://github.com/mmatteo-hub/VR4R_Assignment).  
To correctly visualize the map, make sure that `Sun Position Controller` plugin is enabled in UE. For more details, read UE DOcumentation [here](https://docs.unrealengine.com/5.1/en-US/geographically-accurate-sun-positioning-tool-in-unreal-engine/).  
Now everything is set to run the simulation!
## Running
To run the simulation, type in the terminal:
```
roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=$WSL_HOST_IP
python3 ./run.py
rosservice call /graph_knowledge/load_graph         (press Tab and complete with path to your .jason graph)
rosservice call /graph_knowledge/compute_path       (press Tab and complete with start and goal nodes)
```
rosrun pollution_monitoring test.py

# TO DO
Creare grafico json (file già presente nella folder *graphs*) con posizione base parametro  
Creare script per arm/disarm dei droni (prendere comando da input in terminale da utente)
