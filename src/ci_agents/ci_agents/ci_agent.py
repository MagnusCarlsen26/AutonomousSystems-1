import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
import random

class CIAgent(Node):
    def __init__(self,ciAgentNo):
        super().__init__('ci_agent')
        self.meeting_publisher = self.create_publisher(String, 'visitor_response', 10)
        self.meeting_subscriber = self.create_subscription(
            String,
            'meeting_request',
            self.meeting_request_callback,
            10
        )
        
        self.authorization_publisher = self.create_publisher(String, 'check_authorization',10)
        self.authorization_subscriber = self.create_subscription(
            String,
            'check_authorization_response',
            self.check_authorization_callback,
            10
        )

        self.navigation_publisher = self.create_publisher(String, 'navigation_request', 10)
        self.navigation_subscriber = self.create_subscription(
            String,
            'navigation_response',
            self.navigation_response_callback,
            10
        )

        self.visiter_navigation_publisher = self.create_publisher(String, 'visiter_navigation_response', 10)
        self.visiter_navigation_subscriber = self.create_subscription(
            String,
            'visiter_navigation_request',
            self.visiter_navigation_response_callback,
            10
        )

        self.ciAgentNo = ciAgentNo
        self.number_of_visitors_guided = 0
        self.entairtained = 0
        self.violation = 0

    def meeting_request_callback(self, msg):
        request = json.loads(msg.data)
        if request["ciAgentNo."] != self.ciAgentNo :
            return

        self.entairtained += 1
        self.get_logger().info(f'Meeting request received: Visitor - {request["visiterNo"]}, Host - {request["hostNo"]}, Time - {request["meeting_duration"]}')

        number = request["hostNo"][7:]
        request["biAgentNo."] = f"biAgentNo.{number}"
        response_msg = String()
        response_msg.data = json.dumps(request)
        self.authorization_publisher.publish(response_msg)
        self.get_logger().info("Waiting for confirmation from BI")

    def check_authorization_callback(self,msg):
        request = json.loads(msg.data)

        if request["ciAgentNo."] != self.ciAgentNo:
            return

        if random.random()<0.1:
            print("CI not returning to escort visitor")
            self.violation += 1
            return

        self.entairtained += 1
        response_msg = String()
        response_msg.data = json.dumps(request)
        self.meeting_publisher.publish(response_msg)
        self.get_logger().info(f"Message from BI - {response_msg.data}")
        self.get_logger().info('Meeting confirmation sent: ' + response_msg.data)

    def visiter_navigation_response_callback(self,msg):
        request = json.loads(msg.data)

        if request["ciAgentNo."] != self.ciAgentNo:
            return
        
        self.entairtained += 1
        number = request["hostNo"][7:]
        request["biAgentNo."] = f"biAgentNo.{number}"
        self.send_navigation_request(request)

    def send_navigation_request(self, payload):

        msg = String()
        msg.data = json.dumps(payload)  
        self.get_logger().info('Navigation request sent: ' + msg.data)
        self.navigation_publisher.publish(msg)

    def navigation_response_callback(self, msg):
        
        response = json.loads(msg.data)
        if response["ciAgentNo."] != self.ciAgentNo:
            return
        
        if not response["isOOS"] :
            if response["next_move"] == "You have reached !!" :
                self.number_of_visitors_guided += 1
        self.entairtained += 1
        response_msg = String()
        response_msg.data = json.dumps(response)
        self.visiter_navigation_publisher.publish(response_msg)
        self.get_logger().info(f"Navigation confirmation sent: {response_msg.data}" )



def main(args=None):
    rclpy.init(args=args)

    noOfciAgents = 10
    agents = []
    
    for i in range(noOfciAgents):
        agent_name = f"ciAgentNo.{i+1}"
        print(agent_name)
        ci_agent = CIAgent(agent_name)
        agents.append(ci_agent)

    executors = []
    for agent in agents:
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(agent)
        executors.append(executor)

    import threading
    threads = [threading.Thread(target=executor.spin) for executor in executors]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    for agent in agents:
        agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
