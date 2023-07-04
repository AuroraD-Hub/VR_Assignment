#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <smach/sm.h>
#include <smach_ros/introspection_server.h>
#include <drone_coverage_msgs/LoadCoverageGraph.h>
#include <drone_coverage_msgs/ComputeCoveragePath.h>

// CAR_MOVING State
class CarMoving : public smach::State
{
public:
    CarMoving()
    {
        // Qui viene inizializzato lo stato CarMoving
        ROS_INFO("Car Actor State");
        pose_ = geometry_msgs::Pose();
    }

    virtual std::string execute()
    {
        // Connettiti al simulatore AirSim
        airsim::CarClient client("172.23.32.1", 41451);
        client.confirmConnection();
        client.enableApiControl(true);

        ROS_INFO("There should be a car: %s", client.listVehicles().c_str());

        // Carica il file JSON del percorso
        std::string path = ros::package::getPath("VR_Assignment") + "/graphs/drone_graph.json";
        std::ifstream file(path);
        json data;
        file >> data;

        // Avvia l'interfaccia
        char i;
        std::cout << "Whenever you reach your desired position, press 'D' to arm the drone: ";
        std::cin >> i;
        if (i == 'D' || i == 'd')
        {
            pose_ = client.simGetVehiclePose("Car");
            updatePath(path, client, data, pose_);
            client.simDestroyObject("Car");
            std::cout << "There should not be anything: " << client.listVehicles() << std::endl;
            client.simAddVehicle("Multirotor", "SimpleFlight", pose_);
            std::cout << "There should be a drone: " << client.listVehicles() << std::endl;
            return "goal_reached";
        }
        else
        {
            std::ofstream outfile(path);
            outfile << std::setw(4) << data << std::endl;
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

// DRONE_FLYING State
class DroneFlying : public smach::State
{
public:
    DroneFlying()
    {
        // Qui viene inizializzato lo stato DroneFlying
        ROS_INFO("Drone Actor State");
        pose_ = geometry_msgs::Pose();
    }

    virtual std::string execute()
    {
        // Connettiti al simulatore AirSim
        airsim::MultirotorClient client("172.23.32.1", 41451);
        client.confirmConnection();
        client.enableApiControl(true);

        ROS_INFO("There should be a drone: %s", client.listVehicles().c_str());

        // Avvia l'interfaccia
        std::cout << "Execute './run.py' from VR4R_Assignment" << std::endl;
        std::cout << "Done? If so, press any key to continue." << std::endl;
        std::cin.ignore();

        // Carica il grafo JSON e il percorso da seguire
        std::string graph = ros::package::getPath("VR_Assignment") + "/graphs/drone_graph.json";
        drone_coverage_msgs::LoadCoverageGraph srv_load_graph;
        srv_load_graph.request.graph_path = graph;
        ros::service::call("/graph_knowledge/load_graph", srv_load_graph);
        drone_coverage_msgs::ComputeCoveragePath srv_compute_path;
        srv_compute_path.request.start_node_id = "p0";
        srv_compute_path.request.end_node_id = "p0";
        ros::service::call("/graph_knowledge/compute_path", srv_compute_path);

        // Avvia l'interfaccia
        char i;
        std::cout << "When the drone is landed, press 'C' to move to another position for sampling: ";
        std::cin >> i;
        if (i == 'C' || i == 'c')
        {
            pose_ = client.simGetVehiclePose("Multirotor");
            std::cout << "There should not be anything: " << client.listVehicles() << std::endl;
            client.simDestroyObject("Multirotor");
            client.simAddVehicle("Car", "PhysXCar", pose_);
            std::cout << "There should be a car: " << client.listVehicles() << std::endl;
            return "sampling_done";
        }
        else
        {
            return "goal_reached";
        }
    }

private:
    geometry_msgs::Pose pose_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "interface_sm");

    // Crea una macchina a stati SMACH
    smach::StateMachine sm;

    // Apri il contenitore
    smach::StateMachine* container = new smach::StateMachine;
    container->add("CAR_MOVING", new CarMoving());
    container->add("DRONE_FLYING", new DroneFlying());

    // Aggiungi lo stato del contenitore alla macchina a stati principale
    sm.setInitialState(container);

    // Crea e avvia il server di introspezione per la visualizzazione
    smach_ros::IntrospectionServer introspection("server_name", sm, "/SM_ASSIGNMENT");
    introspection.start();

    // Esegui la macchina a stati
    sm.execute();

    // Aspetta la terminazione dell'applicazione
    ros::spin();
    introspection.stop();

    return 0;
}
