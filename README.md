# VR_Assignment
Clone this [repository](https://github.com/mmatteo-hub/VR4R_Assignment).  
To set the environment, run:
```
sudo apt install python-is-python3
python3 -m pip install pymap3d
```
## Running
Then, type in the terminal: (per runnare la soluzione della repository clonata!)
```
roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=$WSL_HOST_IP
python3 ./run.py
rosservice call /graph_knowledge/load_graph         (press Tab)
rosservice call /graph_knowledge/compute_path       (press Tab)
```
rosrun pollution_monitoring test.py

# TO DO
Creare grafico json (file già presente nella folder *graphs*) con posizione base parametro
