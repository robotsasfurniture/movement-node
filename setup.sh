mkdir -p ros2_ws/src/movement_controller
cd ros2_ws/src/movement_controller
ros2 pkg create --build-type ament_python movement_controller --dependencies rclpy geometry_msgs websockets 
mkdir -p movement_controller/resource
touch movement_controller/resource/movement_controller 