import rclpy
from rclpy.node import Node
# from server_to_fms.srv import TryOnRequest
from robocallee_fms.srv import RobotArmRequest 
from http_request import ask_django_ocr


import time 

django_url = 'http://192.168.0.189:8000/gwanje/ocr_from_flask_stream/'


service_name = 'arm2_service'

#service_name = 'employee_service'

class JetcoService(Node):
    def __init__(self):
        super().__init__('arm2_service_node')


        self.srv = self.create_service(RobotArmRequest, service_name, self.handle_request)        
        #self.srv = self.create_service(EmployeeRequest, service_name, self.handle_request)
                
                

    def handle_request(self, request, response):
        self.get_logger().info(f"선반 번호: { str(request.shelf_num) }, 핑키 번호: {request.pinky_num}") 
        
        response.action = request.action
        response.shelf_num = request.shelf_num
        response.success = True

        if request.action == 'buffer_to_shelf':
            shoe_info = ask_django_ocr(django_url, 'get_shoe_info')[0]  # 받을 때 [tmp_json] 형태로 받기 때문
            response.model = shoe_info['model']
            response.size = shoe_info['size']
            response.color = shoe_info['color']



        # time.sleep(3)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = JetcoService()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
    main()
