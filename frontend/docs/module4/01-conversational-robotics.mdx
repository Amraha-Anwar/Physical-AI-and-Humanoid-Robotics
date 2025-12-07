---
sidebar_position: 1
title: Conversational Robotics
---

# Conversational Robotics: Voice-to-Action

## Learning Objectives

Upon completing this chapter, you will be able to:

-   Understand the concept of Voice-to-Action in robotics and its importance for natural human-robot interaction.
-   Identify key components of a speech recognition pipeline for robotic control.
-   Learn how to integrate speech recognition technology (e.g., OpenAI Whisper or similar) to process natural language.
-   Translate transcribed voice commands into actionable ROS 2 messages for robot execution.
-   Develop a basic ROS 2 node to listen for voice commands and publish to a downstream topic.

## Introduction to Conversational Robotics

Conversational robotics aims to enable robots to understand and respond to human speech, making interaction more intuitive and accessible. The goal is to move beyond rigid command structures to natural language interfaces, allowing users to direct robots with spoken instructions. This **Voice-to-Action** paradigm involves several stages: speech capture, speech-to-text conversion, natural language understanding (NLU), and finally, mapping NLU output to robot actions.

For a humanoid robot, this capability opens up possibilities for:
-   Direct task assignment (e.g., "Walk to the door," "Pick up the cup").
-   Querying robot status (e.g., "What are you doing?").
-   Emergency commands (e.g., "Stop!").

## 1. Speech Recognition (Speech-to-Text)

The first step in Voice-to-Action is converting spoken words into written text. This is handled by **Speech-to-Text (STT)** engines. While many options exist (Google Cloud Speech-to-Text, Amazon Transcribe, Mozilla DeepSpeech), **OpenAI Whisper** has emerged as a highly capable, multilingual, and robust open-source STT model.

### Using OpenAI Whisper (Conceptual)

Whisper can be run locally or via API. For an edge device like the Jetson Orin Nano, running a smaller Whisper model locally is feasible for real-time applications.

**Conceptual Workflow:**

1.  **Audio Capture**: The robot's microphone captures audio input from the user.
2.  **Preprocessing**: Audio is often preprocessed (e.g., noise reduction, normalization) before being fed to the STT model.
3.  **Transcription**: The STT model converts the audio into a text transcript.

**Pseudo-code Example (Python with a conceptual Whisper library):**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# from your_audio_pkg.msg import AudioData # Assuming an AudioData message type
import numpy as np
# import whisper # conceptual import

class VoiceCommandTranscriber(Node):
    def __init__(self):
        super().__init__('voice_command_transcriber')
        # Publisher for transcribed text
        self.transcribed_text_pub = self.create_publisher(String, 'voice_commands/text', 10)
        # Subscriber for raw audio data (conceptual)
        # self.audio_sub = self.create_subscription(AudioData, 'audio_in', self.audio_callback, 10)
        
        # Simulate audio input for demonstration
        self.timer = self.create_timer(5.0, self.simulate_audio_input)
        self.get_logger().info('Voice command transcriber node started.')

    def audio_callback(self, msg):
        # Process raw audio data (e.g., convert to format Whisper expects)
        # audio_np = np.frombuffer(msg.data, dtype=np.int16) # Example conversion

        # Perform speech-to-text (conceptual)
        # model = whisper.load_model("base") # Load a Whisper model
        # result = model.transcribe(audio_np)
        # transcribed_text = result["text"]

        # For simulation, we'll use a hardcoded command
        transcribed_text = "robot walk forward" # This would come from Whisper

        if transcribed_text:
            self.get_logger().info(f'Transcribed: "{transcribed_text}"')
            msg_text = String()
            msg_text.data = transcribed_text.lower() # Normalize to lowercase
            self.transcribed_text_pub.publish(msg_text)

    def simulate_audio_input(self):
        # This function simulates receiving audio input
        # In a real system, this would be an actual audio callback
        self.get_logger().info('Simulating audio input...')
        self.audio_callback(None) # Call the callback with dummy data

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandTranscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Pseudo-code: `voice_command_transcriber.py`*

## 2. Linking Voice Commands to ROS 2 Topics

Once we have a text transcript, the next step is to translate this natural language into a structured, actionable command that the robot's control system can understand. This often involves a simple form of **Natural Language Understanding (NLU)**. For basic commands, rule-based parsing can be sufficient. For more complex interactions, an LLM might be employed (covered in the next chapter).

We'll create a simple "command parser" node that subscribes to the transcribed text and publishes a specific command message to a downstream ROS 2 topic.

**Conceptual Workflow:**

1.  **Subscribe to Transcribed Text**: The command parser node listens to the `voice_commands/text` topic.
2.  **Parse Command**: It analyzes the text to identify keywords or patterns that correspond to known robot actions.
3.  **Publish Action Message**: Based on the parsed command, it publishes a specific message to a robot action topic (e.g., `/cmd_vel` for movement, or a custom action goal).

**Pseudo-code Example (Python):**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist # For velocity commands

class CommandParser(Node):
    def __init__(self):
        super().__init__('command_parser')
        # Subscriber for transcribed text commands
        self.text_sub = self.create_subscription(String, 'voice_commands/text', self.text_command_callback, 10)
        # Publisher for robot movement commands (e.g., cmd_vel)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info('Command parser node started. Waiting for voice commands...')

    def text_command_callback(self, msg):
        command_text = msg.data
        self.get_logger().info(f'Received command: "{command_text}"')

        twist_msg = Twist()
        
        # Simple rule-based parsing
        if "walk forward" in command_text:
            self.get_logger().info("Executing: Walk Forward")
            twist_msg.linear.x = 0.2 # Move forward at 0.2 m/s
        elif "turn left" in command_text:
            self.get_logger().info("Executing: Turn Left")
            twist_msg.angular.z = 0.5 # Turn left at 0.5 rad/s
        elif "stop" in command_text:
            self.get_logger().info("Executing: Stop")
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        else:
            self.get_logger().warn(f"Unknown command: '{command_text}'")
            return # Don't publish if command is not recognized

        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CommandParser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*Pseudo-code: `command_parser.py`*

### Integrating with `setup.py`

Remember to add these new nodes as entry points in your `setup.py` within your ROS 2 Python package (e.g., `my_py_pkg` or a new `humanoid_vla` package):

```python
# ... inside setup() function ...
    entry_points={
        'console_scripts': [
            'voice_transcriber = my_py_pkg.voice_command_transcriber:main',
            'command_parser = my_py_pkg.command_parser:main',
        ],
    },
```

## Conclusion

Conversational robotics empowers humanoids with a more natural interaction paradigm. By integrating speech recognition to convert voice to text, and then parsing these text commands into ROS 2 messages, we can enable intuitive control over our robot. This forms the foundation for more advanced cognitive abilities where Large Language Models can interpret complex instructions.

---
**Next Steps**: Building upon speech recognition, the next chapter will delve into **Cognitive Planning** using Large Language Models (LLMs), allowing the robot to understand more abstract goals, reason about its environment, and generate complex action sequences.
