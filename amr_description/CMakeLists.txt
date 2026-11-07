cmake_minimum_required(VERSION 3.8)
project(amr_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Las pruebas procesan el xacro desde el árbol de instalación (find amr_description).
  ament_add_pytest_test(test_urdf test/test_urdf.py TIMEOUT 60)
  ament_add_pytest_test(test_arm_fk test/test_arm_fk.py TIMEOUT 60)
endif()

ament_package()
