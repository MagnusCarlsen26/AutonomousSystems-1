# visitor_agents/visitor_agents/visitor_agent.py
import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
import random
import time
class VisitorAgent(Node):
    def __init__(self,visitorNo,host,ciAgentNo,meeting_duration):
        super().__init__('visitor_agent')

        self.publisher = self.create_publisher(String, 'meeting_request', 10)
        self.subscriber = self.create_subscription(
            String,
            'visitor_response',
            self.visitor_response_callback,
            10
        )

        self.navigation_publisher = self.create_publisher( String, 'visiter_navigation_request',10 )
        self.navigation_subscriber = self.create_subscription(
            String,
            'visiter_navigation_response',
            self.navigation_response_callback,
            10  
        )

        self.visiterNo = visitorNo
        self.host = host
        self.ciAgentNo = ciAgentNo
        self.meeting_duration = meeting_duration    
        self.navigation_history = []
        
    def send_meeting_request(self):

        request = {
            "visiterNo" : self.visiterNo,
            "hostNo": self.host,
            "ciAgentNo.": self.ciAgentNo,
            "meeting_duration" : self.meeting_duration
        }
        msg = String()
        msg.data = json.dumps(request)
        self.publisher.publish(msg)
        self.get_logger().info('Meeting request sent: ' + msg.data)

    def navigation_response_callback(self,msg):

        response = json.loads(msg.data)

        if response["visiterNo"] != self.visiterNo:
            return  
        
        if response["isOOS"] :
            print("host out of service")
            time.sleep(7)
        else :
            print(f"Next move = ",response["next_move"])
            self.navigation_history.append(response["next_move"])
            time.sleep(3)
        self.send_navigation_request()

    def send_navigation_request(self):

        request = {
            "visiterNo" : self.visiterNo,
            "hostNo": self.host,
            "ciAgentNo.": self.ciAgentNo,
            "meeting_duration" : self.meeting_duration,
            "navigation_history" : self.navigation_history
        }
        
        msg = String()
        msg.data = json.dumps(request)
        self.navigation_publisher.publish(msg)
        self.get_logger().info('Navigation request sent: ' + msg.data)

    def visitor_response_callback(self, msg):
        response = json.loads(msg.data)

        if response["visiterNo"] != self.visiterNo:
            return  
        if response["isOOS"] :
            print("Host is Out of service")
            self.send_meeting_request()
            return


        self.get_logger().info(f'Visitor response received: Message - {response["authorization"]}')
        if response['authorization'] == 'Access Granted':
            self.send_navigation_request()

def main(args=None):
    rclpy.init(args=args)

    noOfVisitors = 2
    noOfciAgents = 10

    for i in range(1,noOfVisitors):
        visitor_agent = VisitorAgent(
            f"visiterNo.{i}",
            f"hostNo.{random.randint(1,noOfVisitors)}",
            f"ciAgentNo.{random.randint(1,noOfciAgents)}",
            f"{random.randint(1,30)} Minutes"
        )
        visitor_agent.send_meeting_request()

        rclpy.spin(visitor_agent)
        visitor_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
