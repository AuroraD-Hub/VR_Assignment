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
pip install airsim
sudo pip install msgpack-rpc-python 
```
Finally, in your ROS workspace clone this [repository](https://github.com/mmatteo-hub/VR4R_Assignment).  
To correctly visualize the map, make sure that `Sun Position Controller` plugin is enabled in UE. For more details, read UE DOcumentation [here](https://docs.unrealengine.com/5.1/en-US/geographically-accurate-sun-positioning-tool-in-unreal-engine/).  
Now everything is set to run the simulation!
## Running
To run the simulation, type in the terminal:
```
roslaunch VR_Assignment pollution_moniotring.launch
||
roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=$WSL_HOST_IP
python3 ./run.py
rosservice call /graph_knowledge/load_graph         (press Tab and complete with path to your .json graph)
rosservice call /graph_knowledge/compute_path       (press Tab and complete with start and goal nodes)
||
```
## Troubleshooting
If you need to temporarly disable the WSL, you can run the following commands to save all your file:  
Open the Windows prompt and run:
```
wsl --export <Distribution Name> <FileName>
```
Note that with this line you get a `.tar` file in which all your latest files and installations are saved. This file will be stored in the folder in which your terminal was open.

Whenever you need to enable it again, do the following steps:
1. Open the Windows prompt and run:
   ```
   wsl --import <Distribution Name> <InstallLocation> <FileName>
   ```
2. Open your WSL ditribution terminal and note if you enter as `root`. If this is the case, but your command are in your user `home` folder, type:
   ```
   gedit .bashrc
   ```
   and type `source /path_to_your_user_home_folder/.bashrc` at the end of the file. Now close the terminal.  
2. Reopen your WSL ditribution terminal and run:
   ```
   sudo apt update
   sudo apt upgrade
   sudo apt --fix-broken install
   ```  
4. Go to your workspace and type:
   ```
   source devel/setup.bash
   rospack profile
   rospack list
   ```
   Do these steps to be sure your ROS environment is properly sourced and all your packages are available.
5. To re-establish connection with your GitHub repository type:
    ```
    git config --global --add safe.directory
    ```
Now your WSL installation should be up and running smooth again.

# TO DO
Creare grafico json (file già presente nella folder *graphs*) con posizione base parametro  
Creare script per arm/disarm dei droni (prendere comando da input in terminale da utente)
