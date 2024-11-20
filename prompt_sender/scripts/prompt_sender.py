#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ros2_llm_interfaces.srv import InferenceService

class AutoPromptLLMNode(Node):
    def __init__(self):
        super().__init__('auto_prompt_llm_node')
        
        # Subscriber for the /prompt topic
        self.subscription = self.create_subscription(
            String,
            '/prompt',
            self.prompt_callback,
            10
        )
        
        # Publisher for the /response topic
        self.response_publisher = self.create_publisher(
            String,
            '/response',
            10
        )
        
        # Client for the LLM service
        self.llm_client = self.create_client(InferenceService, '/chat_llm')
        self.get_logger().info('Waiting for LLM service to become available...')
        
        # Wait for the service to be available before proceeding
        while not self.llm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Waiting for LLM service to become available...')
        
        self.get_logger().info('LLM service is available.')

    def prompt_callback(self, msg):
        # Log the received prompt data
        prompt = 'one word asnwer please, cat or dog?'#msg.data
        self.get_logger().info(f'Received prompt: {prompt}')
        
        # Call the LLM service synchronously with the prompt
        self.send_prompt_to_llm(prompt)

    def send_prompt_to_llm(self, prompt):
        # Create a service request
        request = InferenceService.Request()
        request.prompt = prompt
        request.images = []  # Provide an empty list for images as per the service definition

        # Call the service synchronously and wait for the response
        self.get_logger().info(f'Sending prompt to LLM service: {prompt}')
        try:
            response = self.llm_client.call(request)
            if response:
                # Log and publish the response
                self.get_logger().info(f'LLM response: {response.response}')
                response_msg = String()
                response_msg.data = response.response
                self.response_publisher.publish(response_msg)
                self.get_logger().info('Published LLM response to /response topic.')
            else:
                self.get_logger().warn('Received no response from LLM service.')
        except Exception as e:
            self.get_logger().error(f'Exception during LLM service call: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = AutoPromptLLMNode()
    try:
        # Keep the node active to listen for prompts and process service calls
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle clean shutdown on Ctrl+C
        pass
    finally:
        # Clean up resources and shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
