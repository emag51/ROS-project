cmake_minimum_required(VERSION 3.5)
project(robot_urdf)

# Ensure we use ament_cmake
find_package(ament_cmake REQUIRED)

# Install URDF, meshes, launch, and RViz files
install(
  DIRECTORY meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files explicitly
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.py" PERMISSIONS OWNER_EXECUTE OWNER_READ OWNER_WRITE GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)

# Add package.xml install rules (optional but good practice)
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Declare this as an ament package
ament_package()
