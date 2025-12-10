---
sidebar_position: 2
title: Cognitive Planning with LLMs
---

# Cognitive Planning with LLMs

## Learning Objectives

Upon completing this chapter, you will be able to:

-   Understand the role of Large Language Models (LLMs) in abstract cognitive planning for robots.
-   Learn how to use an LLM (with OpenAI API syntax) to translate natural language goals into sequences of robotic actions.
-   Implement the LLM-to-ROS translation logic, ensuring structured output for reliable execution.
-   Integrate LLM-generated plans into a ROS 2 system.

## Introduction to Cognitive Planning

Traditional robot planning often relies on predefined state machines, rule-based systems, or complex symbolic AI techniques. While effective for well-defined problems, these approaches struggle with ambiguity, novel situations, and the open-ended nature of human commands. **Cognitive planning**, particularly with the advent of Large Language Models (LLMs), offers a powerful paradigm shift.

LLMs can bridge the gap between high-level, abstract natural language goals (e.g., "Make me coffee," "Clean the room") and the low-level, concrete actions a robot can perform. They can:
-   **Interpret ambiguity**: Understand nuanced human instructions.
-   **Reason about context**: Infer implicit steps based on common sense knowledge.
-   **Generate sequences**: Decompose a complex goal into a series of simpler, executable steps.
-   **Handle exceptions**: Potentially suggest recovery strategies for failures.

## 1. LLMs for Task Decomposition

The core idea is to prompt an LLM with a high-level goal and a list of available robot actions, asking it to return a structured sequence of these actions to achieve the goal.

### OpenAI API Syntax for Structured Output

To ensure reliable execution, the LLM's output must be structured and parseable. Modern LLMs support JSON output, which is ideal for this. We'll use a `system` prompt to define the robot's capabilities and the desired output format, and a `user` prompt for the specific task.

Let's assume our humanoid robot has the following basic actions:
-   `walk(direction)`: `direction` can be "forward", "backward", "left", "right".
-   `turn(direction)`: `direction` can be "left", "right".
-   `look(object)`: `object` can be "table", "door", "person".
-   `pick_up(object)`: `object` can be "cup", "ball".
-   `place_down(object)`: `object` can be "cup", "ball".

**LLM Prompt Design:**

````python
system_prompt = """
You are a robotic task planner. Your goal is to break down high-level natural language instructions into a sequence of low-level robot actions.
The robot can perform the following actions:
- walk(direction: str) - Moves the robot. Directions: "forward", "backward", "left", "right".
- turn(direction: str) - Rotates the robot. Directions: "left", "right".
- look(object: str) - Orients the robot's head/camera towards an object. Objects: "table", "door", "person", "cup", "ball".
- pick_up(object: str) - Picks up a specified object. Objects: "cup", "ball".
- place_down(object: str) - Places down a specified object. Objects: "cup", "ball".

Return the plan as a JSON array of action objects. Each action object must have a 'name' (string) and 'args' (object mapping argument names to values).
"""
````


Example:
User: "Go get the cup from the table."
Output:
```json
[
  {"name": "walk", "args": {"direction": "forward"}},
  {"name": "look", "args": {"object": "table"}},
  {"name": "walk", "args": {"direction": "forward"}},
  {"name": "pick_up", "args": {"object": "cup"}},
  {"name": "walk", "args": {"direction": "backward"}},
  {"name": "place_down", "args": {"object": "cup"}}
]
```

If an instruction is unclear or an object/action is not supported, return an empty array or indicate impossibility.

user_instruction = "Please go to the door, then pick up the red ball and bring it here." # Example user input


## 2. LLM-to-ROS Translation Logic (Pseudo-code)

This logic involves:
1.  Sending the prompt to the LLM (e.g., OpenAI API).
2.  Parsing the JSON response.
3.  Translating each LLM-generated action into a corresponding ROS 2 message or action goal.
4.  Executing the sequence of ROS 2 actions.

**Pseudo-code Example (Python with `rclpy` and conceptual OpenAI API):**

````python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
# from custom_action_msgs.action import PickUp, PlaceDown # Conceptual custom action types
import json
import requests # For conceptual OpenAI API call

# --- ROS 2 Message Definitions for Actions (conceptual) ---
# For simplicity, we'll use Twist for walk/turn, and String for look.
# For pick_up/place_down, you would typically use ROS 2 Actions.

# Assuming OpenAI API key is set as an environment variable
OPENAI_API_KEY = "YOUR_OPENAI_API_KEY" # In a real app, use env vars or secrets

class CognitivePlanner(Node):
    def __init__(self):
        super().__init__('cognitive_planner')
        self.declare_parameter('openai_api_endpoint', 'https://api.openai.com/v1/chat/completions')
        self.openai_api_endpoint = self.get_parameter('openai_api_endpoint').get_parameter_value().string_value

        self.get_logger().info("Cognitive Planner node started.")
        self.get_logger().info(f"OpenAI API Endpoint: {self.openai_api_endpoint}")

        # Subscribe to high-level natural language goals (from voice command, GUI, etc.)
        self.goal_sub = self.create_subscription(String, 'human_goals', self.goal_callback, 10)

        # Publishers for low-level robot actions
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.look_at_pub = self.create_publisher(String, 'look_at_object', 10) # Custom topic for look action
        # self.pick_up_client = ActionClient(self, PickUp, 'pick_up_action') # Conceptual action client
        # self.place_down_client = ActionClient(self, PlaceDown, 'place_down_action') # Conceptual action client

        self.robot_actions = {
            "walk": self.execute_walk_action,
            "turn": self.execute_turn_action,
            "look": self.execute_look_action,
            "pick_up": self.execute_pick_up_action, # Placeholder
            "place_down": self.execute_place_down_action, # Placeholder
        }

        self.system_prompt = """
        You are a robotic task planner. Your goal is to break down high-level natural language instructions into a sequence of low-level robot actions.
        The robot can perform the following actions:
        - walk(direction: str) - Moves the robot. Directions: "forward", "backward", "left", "right".
        - turn(direction: str) - Rotates the robot. Directions: "left", "right".
        - look(object: str) - Orients the robot's head/camera towards an object. Objects: "table", "door", "person", "cup", "ball".
        - pick_up(object: str) - Picks up a specified object. Objects: "cup", "ball".
        - place_down(object: str) - Places down a specified object. Objects: "cup", "ball".

        Return the plan as a JSON array of action objects. Each action object must have a 'name' (string) and 'args' (object mapping argument names to values).

        Example:
        User: "Go get the cup from the table."
        Output:
        ```json
        [
          {"name": "walk", "args": {"direction": "forward"}},
          {"name": "look", "args": {"object": "table"}},
          {"name": "walk", "args": {"direction": "forward"}},
          {"name": "pick_up", "args": {"object": "cup"}},
          {"name": "walk", "args": {"direction": "backward"}},
          {"name": "place_down", "args": {"object": "cup"}}
        ]
        ```
        If an instruction is unclear or an object/action is not supported, return an empty array or indicate impossibility.
        """

    def call_llm_api(self, user_instruction):
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {OPENAI_API_KEY}"
        }
        payload = {
            "model": "gpt-4-turbo", # Or another suitable model
            "messages": [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": user_instruction}
            ],
            "response_format": {"type": "json_object"} # Crucial for structured output
        }
        
        try:
            response = requests.post(self.openai_api_endpoint, headers=headers, json=payload)
            response.raise_for_status() # Raise an exception for HTTP errors
            response_json = response.json()
            
            # Extract content from the response
            content = response_json['choices'][0]['message']['content']
            return json.loads(content) # Parse the JSON string from the LLM
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error calling OpenAI API: {e}")
            return []
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding JSON from LLM: {e}. Raw content: {content}")
            return []
        except KeyError as e:
            self.get_logger().error(f"Unexpected LLM response structure: {e}. Response: {response_json}")
            return []

    def goal_callback(self, msg):
        human_goal = msg.data
        self.get_logger().info(f"Received high-level goal: '{human_goal}'")

        # 1. Get plan from LLM
        action_plan = self.call_llm_api(human_goal)
        if not action_plan:
            self.get_logger().warn("LLM failed to generate a valid plan or returned an empty plan.")
            return

        self.get_logger().info(f"LLM-generated plan: {action_plan}")

        # 2. Execute plan
        for action in action_plan:
            action_name = action.get("name")
            action_args = action.get("args", {})

            if action_name in self.robot_actions:
                self.get_logger().info(f"Executing action: {action_name} with args: {action_args}")
                self.robot_actions[action_name](**action_args)
                # In a real system, you'd wait for action completion before proceeding
                rclpy.spin_once(self, timeout_sec=1.0) # Allow ROS to process
            else:
                self.get_logger().error(f"Unknown action '{action_name}' in plan. Skipping.")
                return # Stop execution if an unknown action is encountered
        
        self.get_logger().info("Plan execution completed.")

    # --- Low-level ROS 2 Action Execution Methods ---
    def execute_walk_action(self, direction):
        twist_msg = Twist()
        if direction == "forward":
            twist_msg.linear.x = 0.2
        elif direction == "backward":
            twist_msg.linear.x = -0.1
        elif direction == "left":
            twist_msg.linear.y = 0.1
        elif direction == "right":
            twist_msg.linear.y = -0.1
        else:
            self.get_logger().warn(f"Invalid walk direction: {direction}")
            return
        self.cmd_vel_pub.publish(twist_msg)
        # Simulate action duration
        self.get_logger().info(f"Robot walking {direction}...")
        # In a real robot, this would be a feedback loop or a blocking action client call

    def execute_turn_action(self, direction):
        twist_msg = Twist()
        if direction == "left":
            twist_msg.angular.z = 0.5
        elif direction == "right":
            twist_msg.angular.z = -0.5
        else:
            self.get_logger().warn(f"Invalid turn direction: {direction}")
            return
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info(f"Robot turning {direction}...")

    def execute_look_action(self, object):
        look_msg = String()
        look_msg.data = object
        self.look_at_pub.publish(look_msg)
        self.get_logger().info(f"Robot looking at {object}...")

    def execute_pick_up_action(self, object):
        self.get_logger().info(f"Robot attempting to pick up {object}...")
        # In a real system, this would involve calling a pick_up_action client goal
        # For example: self.pick_up_client.wait_for_server(); goal = PickUp.Goal(object_name=object); future = self.pick_up_client.send_goal_async(goal)

    def execute_place_down_action(self, object):
        self.get_logger().info(f"Robot attempting to place down {object}...")
        # In a real system, this would involve calling a place_down_action client goal


def main(args=None):
    rclpy.init(args=args)
    cognitive_planner_node = CognitivePlanner()
    
    # Simulate a human goal
    # In a real scenario, this would come from the voice command node or a GUI
    human_goal_publisher = cognitive_planner_node.create_publisher(String, 'human_goals', 10)
    
    # Give some time for publishers/subscribers to connect
    import time
    time.sleep(2) 
    
    simulated_goal = String()
    simulated_goal.data = "Go to the door, then pick up the cup from the table and place it down."
    cognitive_planner_node.get_logger().info(f"Simulating human goal: {simulated_goal.data}")
    human_goal_publisher.publish(simulated_goal)

    rclpy.spin(cognitive_planner_node)

    cognitive_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
````
*Pseudo-code: `cognitive_planner.py`*

### Explanation of Pseudo-code:

-   **`CognitivePlanner` Node**: Subscribes to `human_goals` (natural language input) and publishes to low-level action topics (`cmd_vel`, `look_at_object`). It also conceptually uses `ActionClient` for `pick_up`/`place_down` actions.
-   **`call_llm_api`**: A helper function to interact with the OpenAI API. It constructs the prompt using the `system_prompt` (defining robot capabilities and output format) and the `user_instruction`. Crucially, it sets `response_format={"type": "json_object"}` to enforce structured JSON output.
-   **`goal_callback`**: When a new high-level goal is received, it calls the LLM, parses the JSON plan, and then iterates through the actions, calling the appropriate ROS 2 execution method.
-   **Low-level Action Methods**: `execute_walk_action`, `execute_turn_action`, `execute_look_action` translate the LLM's structured actions into ROS 2 `Twist` messages or custom messages. For more complex actions like `pick_up` or `place_down`, these would typically involve ROS 2 Action Servers/Clients for robust execution with feedback and preemption.

## 3. Integrating into ROS 2 Ecosystem

To make `cognitive_planner.py` executable within your ROS 2 environment, ensure it's placed in a package (e.g., `humanoid_vla_pkg`) and its entry point is defined in the `setup.py` of that package:

````python
# ... inside setup() function ...
    entry_points={
        'console_scripts': [
            'cognitive_planner = humanoid_vla_pkg.cognitive_planner:main',
        ],
    },
````

After building your package (`colcon build --packages-select humanoid_vla_pkg`) and sourcing your workspace, you can run the node:

```bash
ros2 run humanoid_vla_pkg cognitive_planner
```

This node would then listen for goals on the `/human_goals` topic. In a complete system, the `VoiceCommandTranscriber` node from the previous chapter would publish to this topic.

## Conclusion

By leveraging LLMs for cognitive planning, robots can move beyond rigid, pre-programmed behaviors to truly understand and execute complex, high-level natural language commands. The ability to generate structured action plans from LLMs and translate them into executable ROS 2 messages is a significant step towards more autonomous and human-friendly robotic systems.

---
**Next Steps**: With the cognitive brain in place, the final chapter will synthesize all learned concepts into a Capstone project, focusing on the crucial aspects of testing, integration, and the Sim-to-Real transfer of these complex humanoid capabilities.
