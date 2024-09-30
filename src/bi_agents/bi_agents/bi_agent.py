# bi_agents/bi_agents/bi_agent.py
import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
import random
import threading

class BIAgent(Node):
    def __init__(self,biAgentNo):
        super().__init__('bi_agent')

        self.authorization_publisher = self.create_publisher(String, 'check_authorization_response',10)
        self.authorization_subscriber = self.create_subscription(
            String,
            'check_authorization',
            self.check_authorization_callback,
            10
        )

        self.navigation_publisher = self.create_publisher(String, 'navigation_response',10)
        self.navigation_subscriber = self.create_subscription(
            String,
            'navigation_request',
            self.navigation_request_callback,
            10
        )

        self.biAgentNo = biAgentNo
        self.isOOS = False
        self.OOS_duration = 0
        self.OOS_duration = 0
        self.number_of_ci_guided = 0
        self.entartained = 0
        self.penalty = 0
        
    def getOOS(self):
        if random.random()<0.01 :
            self.OOS_duration = random.randint(1,10)
            self.isOOS = True

    def check_authorization_callback(self,msg):

        
        request = json.loads(msg.data)
        
        if request["biAgentNo."] != self.biAgentNo :
            return
        
        if not self.isOOS  :
            self.entartained += 1
            self.isOOS = False
            self.OOS_duration = 0
            if random.random() < 0.3 :
                request["authorization"] = "Access Denied"
            else :
                request["authorization"] = "Access Granted"
            request["isOOS"] = False
        else :
            request["isOOS"] = True
            if random.random() < 0.5 :
                self.OOS = False
                self.penalty += self.OOS_duration
                self.OOS_duration = 0
            else :
                self.OOS_duration -= 1
            
        response_msg = String()
        response_msg.data = json.dumps(request)
        self.authorization_publisher.publish(response_msg)
        self.get_logger().info('Meeting confirmation sent: ' + response_msg.data)

        self.getOOS()

    def navigation_request_callback(self,msg):

        request = json.loads(msg.data)

        if request["biAgentNo."] != self.biAgentNo :
            return
        
        if not self.isOOS  :
            self.entartained += 1
            self.isOOS = False
            self.OOS_duration = 0
            request["next_move"] = ["L","R","U","D"][random.randint(0,3)]
            if random.random()<0.1:
                request["next_move"] = "You have reached !!"
                self.number_of_ci_guided += 1
            request["isOOS"] = False
        else :
            request["isOOS"] = True
            if random.random() < 0.5 :
                self.OOS = False
                self.penalty += self.OOS_duration
                self.OOS_duration = 0
            else :
                self.OOS_duration -= 1
        response_msg = String()

        response_msg.data = json.dumps(request)
        self.get_logger().info('Navigation sent' + response_msg.data)

        self.navigation_publisher.publish(response_msg)
        self.getOOS()

def main(args=None):

    rclpy.init(args=args)


    agents = []
    noOfbiAgents = 10

    for i in range(noOfbiAgents):
        agent_name = f"biAgentNo.{i+1}"
        print(agent_name)
        bi_agent = BIAgent(agent_name)
        agents.append(bi_agent)
    
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