cmake_minimum_required(VERSION 3.8)
project(dsr_description2)

find_package(ament_cmake REQUIRED)

# package://dsr_description2/meshes/... 를 해결하기 위한 설치.
# RViz/robot_state_publisher 는 ament index 로 share 디렉터리를 찾는다.
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})

ament_package()
