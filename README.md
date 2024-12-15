# Network Control Simulation in ROS-1
- A Docker Containerized-ROS1 Package to simulate the network conditions and its effect on perception,planning , Control on Autonmous Vehicles

- steps to run the simulation
1. Clone the repository.
2. From the root directory of the repository, update the absolute path in `scripts/deploy/base.sh`.
3. Build the Docker image by running the following command:  
   ```bash
   ./scripts/build/project_docker.sh
4. to run the docker image , execute : 
    ```bash 
    ./scripts/deploy/devel.sh
5. Once inside the Docker container, navigate to /root/workspace 
- build the workspace 
- run  (this will run , network interface node , vehicle-sim node , vehicle-dynamics nod)
    ```bash 
    ./client.sh
- roslaunch vehicle_interface vehicle_interface.launch ( low level controller interface )
- roslaunch central_node central_node.launch ( cloud computing interface to run high level trajectory planning like : model predictive control )
