cmake_minimum_required(VERSION 3.8)
project(dual_fr3_maniskill)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

# Optional at configure time so geometry/MPM-only installations still build.
# The Rope-Actor finger-contact path reports a clear error if it is missing.
find_package(Python3 COMPONENTS Interpreter Development.Module QUIET)
find_path(DUAL_FR3_SAPIEN_INCLUDE_DIR sapien/sapien_actor_base.h
  HINTS
    "$ENV{VIRTUAL_ENV}/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/sapien/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../.venv/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/sapien/include")
if(Python3_Development.Module_FOUND AND DUAL_FR3_SAPIEN_INCLUDE_DIR)
  Python3_add_library(_rope_physx MODULE src/rope_physx.cpp)
  target_compile_features(_rope_physx PRIVATE cxx_std_14)
  target_compile_definitions(_rope_physx PRIVATE NDEBUG)
  target_include_directories(_rope_physx PRIVATE
    "${DUAL_FR3_SAPIEN_INCLUDE_DIR}"
    "${DUAL_FR3_SAPIEN_INCLUDE_DIR}/physx"
    "${DUAL_FR3_SAPIEN_INCLUDE_DIR}/pxshared")
  ament_get_python_install_dir(PYTHON_INSTALL_DIR)
  install(TARGETS _rope_physx LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")
endif()
install(PROGRAMS
  scripts/maniskill_bridge.py
  scripts/usb_cable_bridge.py
  scripts/trunking_cable_bridge.py
  scripts/check_simulation.py
  scripts/check_cable.py
  scripts/check_rope_finger_contact.py
  scripts/check_initial_grasp_geometry.py
  scripts/check_insertion_geometry.py
  scripts/watch_usb_grasp.py
  scripts/watch_usb_insertion.py
  scripts/check_mpm.py
  scripts/profile_trunking_cable.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config meshes docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md requirements-maniskill2.txt DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  # Python package moves retain old imports without eagerly loading the engine.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_perception_camera test/test_perception_camera.py)
  ament_add_pytest_test(test_insertion test/test_insertion.py)
  ament_add_pytest_test(test_insertion_services test/test_insertion_services.py)
  ament_add_pytest_test(test_module_layout test/test_module_layout.py)
  ament_add_pytest_test(test_trajectory test/test_trajectory.py)
  ament_add_pytest_test(test_forces test/test_forces.py)
  ament_add_pytest_test(test_usb_grasp test/test_usb_grasp.py)
  ament_add_pytest_test(test_usb_grasp_services test/test_usb_grasp_services.py)
  ament_add_pytest_test(test_usb_only test/test_usb_only.py)
  ament_add_pytest_test(test_assets test/test_assets.py)
  ament_add_pytest_test(test_cable_model test/test_cable_model.py)
  ament_add_pytest_test(test_cable_backends test/test_cable_backends.py)
  ament_add_pytest_test(test_rope_finger_contact test/test_rope_finger_contact.py)
  ament_add_pytest_test(test_rope_timestep test/test_rope_timestep.py)
  ament_add_pytest_test(test_threading test/test_threading.py)
  ament_add_pytest_test(test_initial_grasp_geometry test/test_initial_grasp_geometry.py)
  ament_add_pytest_test(test_sapien_compat test/test_sapien_compat.py)
  ament_add_pytest_test(test_warp_setup test/test_warp_setup.py)
  ament_add_pytest_test(test_cable_profile test/test_cable_profile.py)
endif()
ament_package()
