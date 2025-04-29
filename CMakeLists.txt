cmake_minimum_required(VERSION 3.8)
project(rviz2_tf_transform_lookup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/rviz2_tf_transform_lookup/tf_lookup.hpp
)
set(CMAKE_AUTORCC ON)
qt5_add_resources(RESOURCE_FILES resources.qrc)
add_library(tf_lookup src/tf_lookup.cpp ${MOC_FILES} ${RESOURCE_FILES})
target_include_directories(tf_lookup PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tf_lookup
  pluginlib
  rviz_common
  std_msgs
  tf2
  tf2_ros
)
install(TARGETS tf_lookup
        EXPORT export_rviz2_tf_transform_lookup
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_export_targets(export_rviz2_tf_transform_lookup)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()
