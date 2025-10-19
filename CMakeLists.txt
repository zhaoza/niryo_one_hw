cmake_minimum_required(VERSION 3.0.2)
project(niryo_one_hw)

# C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  std_srvs
  rosparam_shortcuts
  dynamixel_sdk
)

# If the package exports variables/targets, prefer those
set(HAVE_DXL_IMPORTED_TARGET OFF)
if (TARGET dynamixel_sdk::dynamixel_sdk)
  set(HAVE_DXL_IMPORTED_TARGET ON)
endif()

# Fallback: look for the library by common names (for some system installs)
if (NOT HAVE_DXL_IMPORTED_TARGET AND NOT dynamixel_sdk_LIBRARIES)
  find_library(DXL_LIB
    NAMES dynamixel_sdk dxl_x64_cpp dxl_x86_cpp dxl_sbc_cpp
    HINTS ${catkin_LIBRARY_DIRS} /opt/ros/noetic/lib /usr/lib /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu
  )
  if (NOT DXL_LIB)
    message(FATAL_ERROR "Cannot find Dynamixel SDK library. Install ros-noetic-dynamixel-sdk or vendor the 'dynamixel_sdk' catkin pkg.")
  endif()
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp hardware_interface controller_manager std_srvs rosparam_shortcuts dynamixel_sdk
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamixel_sdk_INCLUDE_DIRS}
)

add_executable(niryo_one_hw_node
  src/niryo_one_hw_node.cpp
  src/niryo_one_hw_interface.cpp
)

# Ensure correct build order with catkin
add_dependencies(niryo_one_hw_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Link order: catkin first
target_link_libraries(niryo_one_hw_node ${catkin_LIBRARIES})

if (HAVE_DXL_IMPORTED_TARGET)
  target_link_libraries(niryo_one_hw_node dynamixel_sdk::dynamixel_sdk)
elseif (dynamixel_sdk_LIBRARIES)
  target_link_libraries(niryo_one_hw_node ${dynamixel_sdk_LIBRARIES})
else()
  target_link_libraries(niryo_one_hw_node ${DXL_LIB})
endif()

# Install rules
install(TARGETS niryo_one_hw_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
