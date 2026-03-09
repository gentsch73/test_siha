cmake_minimum_required(VERSION 3.14)
project(siha_autonomy)

# C++17 zorunlu
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimizasyon (Release modda)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# ─────────────────────────────────────────────────
#  Bağımlılıklar
# ─────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# ─────────────────────────────────────────────────
#  Header Dizini
# ─────────────────────────────────────────────────
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# ─────────────────────────────────────────────────
#  Kütüphane: siha_core (tüm modüller)
# ─────────────────────────────────────────────────
add_library(siha_core SHARED
  # Core
  src/core/mission_controller.cpp

  # Vision
  src/vision/yolo_detector.cpp
  src/vision/target_tracker.cpp
  src/vision/lockon_manager.cpp
  src/vision/video_pipeline.cpp

  # Flight
  src/flight/pid_controller.cpp
  src/flight/guidance_system.cpp
  src/flight/flight_controller.cpp

  # Mission
  src/mission/kamikaze_module.cpp
  src/mission/qr_detector.cpp
  src/mission/search_pattern.cpp
  src/mission/evasion_module.cpp

  # Decision (YENİ)
  src/decision/decision_engine.cpp

  # Communication
  src/comm/server_comm.cpp
  src/comm/telemetry_manager.cpp

  # Safety
  src/safety/safety_monitor.cpp

  # Recording
  src/recording/video_recorder.cpp
)

target_link_libraries(siha_core
  ${OpenCV_LIBS}
)

ament_target_dependencies(siha_core
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
)

# ─────────────────────────────────────────────────
#  Ana Çalıştırılabilir Dosya
# ─────────────────────────────────────────────────
add_executable(main_node src/main_node.cpp)

target_link_libraries(main_node
  siha_core
  ${OpenCV_LIBS}
)

ament_target_dependencies(main_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

# ─────────────────────────────────────────────────
#  Kurulum
# ─────────────────────────────────────────────────
install(TARGETS main_node siha_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
