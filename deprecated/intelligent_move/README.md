# Nova Carter3 Robot Control with ChatGPT Integration

This project implements a comprehensive system to control the nova_carter3 robot in Isaac Sim using voice commands, object detection, and ChatGPT-generated ROS 2 code.

## System Architecture

The system consists of four main components:

1. **Speech Recognition Module** (`src/speech_recognition_module.py`)
   - Converts voice commands to text using Google Speech Recognition
   - Supports continuous listening and timeout handling

2. **Object Detection Module** (`src/object_detection.py`)
   - Uses YOLOv8 (YOLOv13 alternative) for object detection
   - Specialized red block detection using HSV color filtering
   - Provides bounding box and center coordinates

3. **ChatGPT Integration** (`src/chatgpt_integration.py`)
   - Generates ROS 2 Python code from natural language commands
   - Specialized prompts for robot navigation and object approach
   - Code validation and safety checks

4. **Robot Controller** (`src/robot_controller.py`)
   - ROS 2 node for nova_carter3 control
   - Handles camera data from stereo cameras
   - Publishes velocity commands to `/cmd_vel`
   - Provides movement primitives and visual feedback control

## Available Topics

- `/cmd_vel` (geometry_msgs/Twist) - Robot movement commands
- `/front_stereo_camera/left/image_raw` (sensor_msgs/Image) - Left camera feed
- `/front_stereo_camera/right/image_raw` (sensor_msgs/Image) - Right camera feed

## Installation

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Set up OpenAI API key:
```bash
export OPENAI_API_KEY="your-api-key-here"
```

3. Ensure ROS 2 is installed and sourced:
```bash
source /opt/ros/humble/setup.bash  # or your ROS 2 distro
```

## Usage

Run the main orchestration script:

```bash
python main.py
```

### Operation Modes

1. **Voice Control Mode**: 
   - Uses speech recognition to interpret commands
   - Processes commands through ChatGPT
   - Executes generated robot control code

2. **Manual Command Mode**:
   - Text-based command input
   - Same ChatGPT processing pipeline
   - Useful for testing and debugging

3. **Autonomous Red Block Detection Mode**:
   - Automatically searches for red blocks
   - Approaches detected blocks using visual feedback
   - Stops when close enough to target

## Example Commands

- "Move forward to the red block"
- "Turn left and find the red object"
- "Stop the robot"
- "Approach the detected red block slowly"

## System Workflow

1. **Voice Input**: User speaks command → Speech recognition converts to text
2. **Object Detection**: Camera feed analyzed for red blocks
3. **Code Generation**: ChatGPT generates ROS 2 control code based on command and detection
4. **Execution**: Generated code executed to control robot movement
5. **Feedback**: Visual feedback used for precise positioning

## Safety Features

- Maximum speed limits
- Code validation before execution
- Fallback movement commands
- Emergency stop capability
- Minimum distance thresholds

## File Structure

```
chat-with-isaacsim/
├── main.py                          # Main orchestration script
├── requirements.txt                 # Python dependencies
├── README.md                       # This file
└── src/
    ├── speech_recognition_module.py # Voice command processing
    ├── object_detection.py         # Red block detection
    ├── chatgpt_integration.py      # LLM code generation
    └── robot_controller.py         # ROS 2 robot control
```

## Troubleshooting

1. **No audio input**: Check microphone permissions and PyAudio installation
2. **Camera not found**: Ensure Isaac Sim is running with nova_carter3 robot
3. **ROS 2 connection issues**: Verify ROS 2 environment and topic availability
4. **ChatGPT errors**: Check OpenAI API key and internet connection

## Notes

- The system uses YOLOv8 instead of YOLOv13 (which is not officially released)
- Red block detection uses HSV color space for better color filtering
- Generated code is validated before execution for safety
- The system includes fallback mechanisms when ChatGPT fails
- Continuous monitoring ensures real-time object detection and response