# PHD-FRI-RECORDER: project-owned recording control interfaces.
cmake_minimum_required(VERSION 3.22)
project(lbr_recording_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PrepareRecording.srv"
  "srv/RecordingCommand.srv"
  "srv/RecordingStatus.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
