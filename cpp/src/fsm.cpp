#include <iostream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <drone_coverage_msgs/LoadCoverageGraph.h>
#include <drone_coverage_msgs/ComputeCoveragePath.h>

// Dichiarazione degli stati
enum class State {
  CAR_MOVING,
  DRONE_FLYING
};

// Dichiarazione della classe CarMoving
class CarMoving {
public:
    CarMoving()
    {
        // Qui viene inizializzato lo stato CarMoving
        ROS_INFO("Car Actor State");
        pose_ = geometry_msgs::Pose();
    }


  std::string execute() {
    // Logica dello stato CarMoving
    std::cout << "Car Actor State\n";

    // Connessione al simulatore AirSim per il veicolo Car
    airsim::CarClient client("172.23.32.1", 41451);
    client.confirmConnection();
    client.enableApiControl(true);

    std::cout << "There should be a car\n";

    // Carica il file JSON del percorso
    std::string path = ros::package::getPath("VR_Assignment") + "/graphs/drone_graph.json";
    std::ifstream file(path);
    json data;
    file>>data;

    // Simulazione dell'interfaccia e dell'aggiornamento dello stato
    std::string input;
    std::cout << "Whenever you reach your desired position, press 'D' to arm the drone: ";
    std::cin >> input;
    if (input == "D" || input == "d") {
      // Aggiornamento del percorso e cambio veicolo a drone
      pose_ = client.simGetVehiclePose("Car");
      updatePath(path, client, data, pose_);
      client.simDestroyObject("Car");

      std::cout << "There should not be anything: "<<client.listVehicles()<<std::endl;
      client.simAddVehicle("Multirotor", "SimpleFlight", pose_);
      std::cout << "There should be a drone: "<<client.listVehicles()<<std::endl;
      return "goal_reached";
    } else {
      // Salvataggio del percorso
      std::ofstream outfile(path);
      outfile<<std::setw(4)<<data<<std::endl;
      return "sampling_done";
    }
  }
private:
    geometry_msgs::Pose pose_;

    void updatePath(const std::string& path, airsim::CarClient& client, const json& data, const geometry_msgs::Pose& pose)
    {
        std::cout << "Car reached position (" << pose.position.x << ", " << pose.position.y << ")." << std::endl;
        for (int i = 0; i < 4; i++)
        {
            data["nodes"]["p" + std::to_string(i)]["position"][0] = pose.position.x;
            data["nodes"]["p" + std::to_string(i)]["position"][1] = pose.position.y;
            data["nodes"]["p" + std::to_string(i)]["position"][2] = pose.position.z + i * 10;
        }

        std::ofstream outfile(path);
        outfile << std::setw(4) << data << std::endl;
    }


};

// Dichiarazione della classe DroneFlying
class DroneFlying {
public:
    DroneFlying()
    {
        // Qui viene inizializzato lo stato DroneFlying
        ROS_INFO("Drone Actor State");
        pose_ = geometry_msgs::Pose();
    }


  std::string execute() {
    // Logica dello stato DroneFlying
    std::cout << "Drone Actor State\n";

    // Connessione al simulatore AirSim per il veicolo Drone
    airsim::MultirotorClient client("172.23.32.1", 41451);
    client.confirmConnection();
    client.enableApiControl(true);

    std::cout << "There should be a drone\n";

    // Simulazione dell'interfaccia e dell'aggiornamento dello stato
    std::cout << "Execute './run.py' from VR4R_Assignment\n";
    std::cout << "Done? If so, press any key to continue.\n";
    std::cin.ignore();

    // Carica il file JSON del grafo e del percorso da seguire
    std::string graph = ros::package::getPath("VR_Assignment") + "/graphs/drone_graph.json";
    drone_coverage_msgs::LoadCoverageGraph srv_load_graph;
    srv_load_graph.request.graph_path = graph;
    ros::service::call("/graph_knowledge/load_graph", srv_load_graph);
    drone_coverage_msgs::ComputeCoveragePath srv_compute_path;
    srv_compute_path.request.start_node_id = "p0";
    srv_compute_path.request.end_node_id = "p0";
    ros::service::call("/graph_knowledge/compute_path", srv_compute_path);

    std::string input;
    std::cout << "When the drone is landed, press 'C' to move to another position for sampling: ";
    std::cin >> input;
    if (input == "C" || input == "c") {
      // Cambio veicolo a car
      pose_ = client.simGetVehiclePose("Multirotor");
      std::cout << "There should not be anything\n";
      client.simDestroyObject("Multirotor");
      client.simAddVehicle("Car", "PhysXCar", pose_);
      std::cout << "There should be a car\n";
      return "sampling_done";
    } else {
      return "goal_reached";
    }
  }
private:
    geometry_msgs::Pose pose_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "interface_sm");

  // Creazione della macchina a stati finiti
  State current_state = State::CAR_MOVING;
  std::string outcome;

  // Esecuzione della macchina a stati finiti
  while (true) {
    if (current_state == State::CAR_MOVING) {
      CarMoving car_moving_state;
      outcome = car_moving_state.execute();

      if (outcome == "goal_reached") {
        current_state = State::DRONE_FLYING;
      }
    } else if (current_state == State::DRONE_FLYING) {
      DroneFlying drone_flying_state;
      outcome = drone_flying_state.execute();

      if (outcome == "sampling_done") {
        current_state = State::CAR_MOVING;
      }
    }
  }

  ros::spin();
  return 0;
}
