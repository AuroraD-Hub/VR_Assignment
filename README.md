# VR_Assignment
## Installing 
First, there is the need of installing and building `AirSim` on Windows by running in your Windows prompt:
```
git clone https://github.com/Microsoft/AirSim.git
cd AirSim
build.cmd
```
Once it is done, go into the `AirSim` folder in `Documents` and modify the `setting.json` file so that it looks like this:  
```
{
    "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
    "SettingsVersion": 1.2,
    "SimMode": "Car",
    "LocalHostIp": "172.23.32.1",
    "ApiServerPort": 41451,
    "Vehicles": {
      "Car": {
        "VehicleType": "PhysXCar"
      },
      "Multirotor": {
        "VehicleType": "SimpleFlight",
        "DefaultVehicleState": "Disarmed"
      }
    }
}
```
Since ROS works on Linux, open an Ubuntu terminal previously installed on a [WSL](https://learn.microsoft.com/en-us/windows/wsl/install) and follow these instructions to install [AirSim ROS Wrapper](https://microsoft.github.io/AirSim/airsim_ros_pkgs/).  
Then, to set the environment for the project, run on Ubuntu terminal:
```
echo "export WSL_HOST_IP=(your WSL IP address)" >> ⁓/.bashrc
sudo apt install python-is-python3
python3 -m pip install pymap3d
sudo pip install msgpack-rpc-python
sudo apt-get install gnome-terminal
pip install airsim
```
Finally, in your ROS workspace clone this [drone controller package](https://github.com/mmatteo-hub/VR4R_Assignment) and this repository typing respectively:
```
git clone https://github.com/mmatteo-hub/VR4R_Assignment
git clone https://github.com/AuroraD-Hub/VR_Assignment
```
Make sure to build your workspace and source it.  
To correctly visualize the map on Unreal Engine, make sure that `Sun Position Controller` plugin is enabled. For more details, read UE DOcumentation [here](https://docs.unrealengine.com/5.1/en-US/geographically-accurate-sun-positioning-tool-in-unreal-engine/).  
Now everything is set to run the simulation!
## Running
To run the simulation, type in terminal and follow the instructions given:
```
rosrun VR_Assignment interface_sm.py
```
## Simulation
This simulation relies on a Finite State Machine (FMS) and the AirSim API to spawn and delite vehicles based on the specific scope of the vehicle. There are two states: `CarMoving` and `DroneFlying`.  
It starts in `CarMoving` since AirSim is set up to `Car` SimMode and allows the user to control the vehicle with keyboard around the city to the chosen location in which the drone should sample the air. Once the location is reached, the user has to follow the instruction prompted on the terminal for the FMS to change state. In this way, the FMS saves current position of the car, delete from simulation the vehicle and make the transition to `DroneFlying`.  
In this second state a drone is spawned in the car last postion saved and two gnome terminals are opened: one for AirSim Wrapper and the other for the drone controller package. Follow the instruction on this last terminal to arm the drone and start the air sampling procedure. Once it is done, these gnome terminals are closed and the user just have to follow the instruction on the main one to make the transition to `CarMoving` again.  
## Advantages
1. Combining the FSM with AirSim API allows to use AirSim with multiple types of vehicles in the same simulation without the need to re-run it, which is actually not supported by AirSim originally. This is a good achievement since multi-type vehicles is a mode frequently requested from developpers that want to use AirSim to control both cars and multirotors.  
2. Another advantage is that since the algorithm relies on a FSM, it is possible to use this simulation ideally infinite times by moving around the city with the car and sampling the air with the drone continuosly. In this way, the simulation can also be considered more realistic.  



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
