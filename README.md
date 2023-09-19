# AMaRa Group
Here the final assignment for Virtual Reality for Robotics is illustrated.  
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
    "SimMode": "Multirotor",
    "LocalHostIp": "insert_your_ip_here",
    "ApiServerPort": 41451,
    "Vehicles": {
      "Drone": {
        "VehicleType": "SimpleFlight"
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
E-mail [us](Contacts) to get the UE environment already ready to use with AirSim Plugin and our _Pollution BP block_ installed. To correctly visualize the map on Unreal Engine, make sure that `Sun Position Controller` plugin is enabled. For more details, read UE Documentation [here](https://docs.unrealengine.com/5.1/en-US/geographically-accurate-sun-positioning-tool-in-unreal-engine/).  
Now everything is set to run the simulation!
## Running
To run this solution, first play the simulation on UE and then type in terminal:
```
roslaunch VR_Assignment pollution_monitoring.launch host:=$WSL_HOST_IP
```
Note: the first time this project is run, the user will likely get an error about a wrong path of json file in which the _Pollution_ Blueprint block write the sampled data (read the report for more details). This depends on the user workspace and operating system so the user should change it with his/her own path to `SavedData/PollutionData.json` automatically generated from the block inside the UE project folder.  

Now run the solution again, follow the instruction given and enjoy your Smart City!

---------------------------------------------------------------------------
# Simulation
This simulation relies on a Finite State Machine (FMS) and the AirSim API to spawn and delite vehicles based on the specific scope of the vehicle. There are two states: `CarMoving` and `DroneFlying`.  
It starts in `CarMoving` since AirSim is set up to `Car` SimMode and allows the user to control the vehicle with keyboard around the city to the chosen location in which the drone should sample the air. Once the location is reached, the user has to follow the instruction prompted on the terminal for the FMS to change state. In this way, the FMS saves current position of the car, delete from simulation the vehicle and make the transition to `DroneFlying`.  
In this second state a drone is spawned in the car last postion saved and two gnome terminals are opened: one for AirSim Wrapper and the other for the drone controller package. Follow the instruction on this last terminal to arm the drone and start the air sampling procedure. Once it is done, these gnome terminals are closed and the user just have to follow the instruction on the main one to make the transition to `CarMoving` again.  
## Advantages and limitations
This project has some advantages with respect to how AirSim is used in literature:
1. Combining the FSM with AirSim API allows to use AirSim with multiple types of vehicles in the same simulation without the need to re-run it, which is actually not supported by AirSim originally. This is a good achievement since multi-type vehicles is a mode frequently requested from developpers that want to use AirSim to control both cars and multirotors.  
2. Another advantage is that since the algorithm relies on a FSM, it is possible to use this simulation ideally infinite times by moving around the city with the car and sampling the air with the drone continuosly. In this way, the simulation can also be considered more realistic.

On the other hand, there are also some limitation:
1. This approach works well whenever vehicles control is mutual, which means that it works specifically when car and multirotor don't have to be controlled at the same time. In other cases, e.g. Squadron of UAV and Ground Vehicles with SWARM technique, this approach cannot be used.
   
--------------------------------------------------------------------------------------------  
# Problems and solution adopted
Because of AirSim setting and Dronati's package issue explained in the report present in this repository, we re-adapted our project to our own controller with minimum changes in the code.  
For this reason, when the simulation starts by following the same instruction in section [Running](Running), a drone is spawned and given instruction to fly towards given waypoints and perform the air pollution monitoring procedure of our scope. Then, the user just have to follow the instruction in main terminal to get the information needed.

--------------------------------------------------------------------------------------------  
# Contacts
Aurora Durante (aurora.durante@coservizi.it)  
Martina Germani (martinella711@gmail.com)  

Both developpers are Master Students in [Robotics Engineering](https://corsi.unige.it/en/corsi/10635) in UNIGE, Genoa.
