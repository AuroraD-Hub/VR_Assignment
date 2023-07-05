#include <iostream>
#include <string>

// Dichiarazione degli stati
enum class State {
  CAR_MOVING,
  DRONE_FLYING
};

// Dichiarazione della classe CarMoving
class CarMoving {
public:
  std::string execute() {
    // Logica dello stato CarMoving
    std::cout << "Car Actor State\n";

    // Connessione al simulatore AirSim per il veicolo Car
    // ...

    std::cout << "There should be a car\n";

    // Carica il file JSON del percorso
    std::string path = "path/to/graphs/drone_graph.json";
    // ...

    // Simulazione dell'interfaccia e dell'aggiornamento dello stato
    std::string input;
    std::cout << "Whenever you reach your desired position, press 'D' to arm the drone: ";
    std::cin >> input;
    if (input == "D" || input == "d") {
      // Aggiornamento del percorso e cambio veicolo a drone
      // ...

      std::cout << "There should not be anything\n";
      std::cout << "There should be a drone\n";
      return "goal_reached";
    } else {
      // Salvataggio del percorso
      // ...

      return "sampling_done";
    }
  }
};

// Dichiarazione della classe DroneFlying
class DroneFlying {
public:
  std::string execute() {
    // Logica dello stato DroneFlying
    std::cout << "Drone Actor State\n";

    // Connessione al simulatore AirSim per il veicolo Drone
    // ...

    std::cout << "There should be a drone\n";

    // Simulazione dell'interfaccia e dell'aggiornamento dello stato
    std::cout << "Execute './run.py' from VR4R_Assignment\n";
    std::cout << "Done? If so, press any key to continue.\n";
    std::cin.ignore();

    // Carica il file JSON del grafo e del percorso da seguire
    std::string graph = "path/to/graphs/drone_graph.json";
    // ...

    std::string input;
    std::cout << "When the drone is landed, press 'C' to move to another position for sampling: ";
    std::cin >> input;
    if (input == "C" || input == "c") {
      // Cambio veicolo a car
      // ...

      std::cout << "There should not be anything\n";
      std::cout << "There should be a car\n";
      return "sampling_done";
    } else {
      return "goal_reached";
    }
  }
};

int main() {
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

  return 0;
}
