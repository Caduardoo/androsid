cmake_minimum_required(VERSION 3.8)
project(android_interfaces)

find_package(ament_cmake REQUIRED)

# When adding the first message/service/action, uncomment the block below,
# add the file(s) under msg/, srv/, or action/, and list them here:
#
# find_package(rosidl_default_generators REQUIRED)
#
# rosidl_generate_interfaces(${PROJECT_NAME}
#   # "msg/Example.msg"
#   # "srv/Example.srv"
#   # "action/Example.action"
# )

ament_package()
