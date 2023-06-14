from drone_interfaces.srv import Move
from std_srvs.srv import Empty
from drone_interfaces.srv import Battery
from drone_interfaces.srv import Height

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
import time


def main(args=None):
    rclpy.init(args=args)

    cb_group = ReentrantCallbackGroup()
    height_limit = False
    node = rclpy.create_node('drone_service_client')




#                                   ----
    cli = node.create_client(Empty, 'streamon')
    req = Empty.Request()
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    try:
        result = future.result()
    except Exception as e:
        node.get_logger().info('Service call failed %r' % (e,))
    else:
        node.get_logger().info("streamon successful")
        


#                                   ----
    async def call_service():


#                                  not working

        cli = node.create_client(Height,'height', callback_group=cb_group)
        
        req = Height.Request()
        res = Height.Response()  #int

        while not cli.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('service not available, waiting again...')

        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node,future)


        
        print(future.result().height)
       


        time.sleep(2)

        #                                   ----

        
    timer = node.create_timer(5, call_service, callback_group= cb_group)    #every 3 seconds
    while  height_limit != True:
        rclpy.spin_once(node)
    if height_limit:
        timer.cancel()
  

        

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
