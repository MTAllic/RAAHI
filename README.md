# RAAHI
Autonomous Navigation System with Camera and LiDAR
Welcome to the Autonomous Navigation System project repository! This system leverages camera and LiDAR inputs to understand its surroundings semantically and employs Large Language Models (LLMs) to reason and make navigation decisions in complex environments. Designed for simulation and real-world applications, the project emphasizes efficiency, adaptability, and robustness in autonomous navigation.

Overview
This project integrates multiple advanced technologies to create an autonomous navigation system capable of operating without prior map data. By combining camera and LiDAR inputs with LLM-based reasoning, the system can:

Capture Semantic Understanding: Use camera and LiDAR data for object detection and scene analysis.
Generate Contextual Prompts: Design prompts based on sensor data to communicate with the LLM.
Enable LLM Reasoning: Utilize LLMs to make high-level decisions and guide navigation tasks.
Navigate Dynamically: Adapt to new environments in real time, leveraging semantic understanding.
Features
Multi-Sensor Integration: Combines camera and LiDAR for real-time semantic mapping.
LLM-Based Decision Making: Employs LLMs for high-level navigation reasoning.
Simulation and Real-World Ready: Fully compatible with simulation platforms and designed for real-world implementation.
Modular Design: Separate components for simulation, object recognition, LLM integration, and navigation.
Getting Started
Prerequisites
ROS 2 (Humble or later): Install ROS 2 for running the robotic framework.
Simulation Tools: Install Gazebo and RViz for testing in simulated environments.
SLAM Toolbox: Set up SLAM Toolbox for real-time mapping and localization.
Object Detection: Use YOLOv8 for recognizing objects in the environment.
LLM Integration: Install OLLAMA to enable LLM processing.
Installation
Follow these steps to set up the environment:

Clone this repository:

bash
Copy code
git clone https://github.com/your-username/autonomous-navigation-system.git
cd autonomous-navigation-system
Install dependencies:

bash
Copy code
curl -fsSL https://ollama.com/install.sh | sh
ollama pull llama3
ollama pull llava-llama3
pip install ollama
Running the System
Launch the Simulation:

bash
Copy code
ros2 launch helios_one launch_sim.launch.py world:=./src/helios_one/worlds/office_small.world
Start SLAM Toolbox:

bash
Copy code
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/helios_one/config/mapper_params_online_async.yaml use_sim_time:=true
Object Recognition with YOLOv8:

bash
Copy code
ros2 launch helios_recognition launch_yolov8.launch.py
Run YOLO-to-LLM Bridge:

bash
Copy code
ros2 launch yolo_to_llm launch_yolo_to_llm.launch.py
Visualize with RViz:

bash
Copy code
rviz2 -d main.rviz
Enable Navigation Stack:

bash
Copy code
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
Start LLM Node:

bash
Copy code
ros2 launch ros2_llm ros2_llm.launch.py
Send LLM Prompts:

bash
Copy code
ros2 service call /query_llm ros2_llm_interfaces/srv/InferenceService "{prompt: 'your prompt here', images: []}"
Automate Prompts:

bash
Copy code
ros2 launch prompt_sender prompt_sender_sim.launch.py
Project Structure
bash
Copy code
.
├── src/
│   ├── helios_one/         # Simulation environment and SLAM configuration
│   ├── helios_recognition/ # Object detection with YOLOv8
│   ├── yolo_to_llm/        # YOLO-to-LLM integration
│   ├── ros2_llm/           # LLM processing scripts
│   └── prompt_sender/      # Automating LLM prompts
├── main.rviz               # RViz configuration file
├── README.md               # Project documentation
Contributing
Contributions are welcome! If you have suggestions or improvements, feel free to fork the repository and submit a pull request. For major changes, open an issue to discuss your ideas.

License
This project is licensed under the MIT License. See the LICENSE file for details.

Contact
For any questions or feedback, please contact [your-email@example.com].
