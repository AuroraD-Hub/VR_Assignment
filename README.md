# VR_Assignment
Clone this [repository](https://github.com/mmatteo-hub/VR4R_Assignment).  
Then, type in the terminal: (per runnare la soluzione della repository clonata!)
```
roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=172.23.40.28
python3 ./run.py
rosservice call /graph_loader/load_graph "location: '/path_to_workspace/src/graph_loader/graphs/mountain_graph.json'"
rosservice call /graph_loader/compute_path "node_start: 'p0' node_goal: 'p2'"
rosrun pollution_monitoring test.py
```

# TO DO
Creare grafico json (file già presente nella folder *graphs*) con posizione base parametro
