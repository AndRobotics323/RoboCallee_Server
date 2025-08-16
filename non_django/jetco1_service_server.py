import rclpy
from rclpy.node import Node

from robocallee_fms.srv import RobotArmRequest 
from robocallee_fms.srv import CustomerRequest 
from robocallee_fms.srv import EmployeeRequest



import time 



service_name = 'arm1_service'

#service_name = 'customer_service'


class JetcoService(Node):
    def __init__(self):
        super().__init__('arm1_service_node')
 
        
        #self.srv = self.create_service(CustomerRequest, service_name, self.handle_request)
        self.srv = self.create_service(RobotArmRequest, service_name, self.handle_request)

    def handle_request(self, request, response):
        self.get_logger().info(f"선반 번호: { str(request.shelf_num) }, 핑키 번호: {request.pinky_num}") 

                
                
                
        response.success = True

        # response.estimated_mins = 3
        # response.message = f"{request.shoe_name} 준비 완료"

        time.sleep(3)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = JetcoService()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
    main()
