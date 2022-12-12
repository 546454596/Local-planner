# Install script for directory: /home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/action" TYPE FILE FILES
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/action/LocalPlanner.action"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/action/GlobalPlanner.action"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/action/ArmorDetection.action"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/msg" TYPE FILE FILES
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/LocalPlannerAction.msg"
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/LocalPlannerActionGoal.msg"
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/LocalPlannerActionResult.msg"
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/LocalPlannerActionFeedback.msg"
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/LocalPlannerGoal.msg"
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/LocalPlannerResult.msg"
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/LocalPlannerFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/msg" TYPE FILE FILES
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/GlobalPlannerAction.msg"
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/GlobalPlannerActionGoal.msg"
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/GlobalPlannerActionResult.msg"
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/GlobalPlannerActionFeedback.msg"
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/GlobalPlannerGoal.msg"
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/GlobalPlannerResult.msg"
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/GlobalPlannerFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/msg" TYPE FILE FILES
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/ArmorDetectionAction.msg"
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/ArmorDetectionActionGoal.msg"
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/ArmorDetectionActionResult.msg"
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/ArmorDetectionActionFeedback.msg"
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/ArmorDetectionGoal.msg"
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/ArmorDetectionResult.msg"
    "/home/cx/brainnavi/testRobot/devel/share/roborts_msgs/msg/ArmorDetectionFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/msg" TYPE FILE FILES
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/msg/TwistAccel.msg"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/msg/GimbalAngle.msg"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/msg/GimbalRate.msg"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/msg/ObstacleMsg.msg"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/msg/ShootInfo.msg"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/msg/ShootState.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/msg/referee_system" TYPE FILE FILES
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/msg/referee_system/BonusStatus.msg"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/msg/referee_system/GameResult.msg"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/msg/referee_system/GameStatus.msg"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/msg/referee_system/GameSurvivor.msg"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/msg/referee_system/ProjectileSupply.msg"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/msg/referee_system/RobotBonus.msg"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/msg/referee_system/RobotDamage.msg"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/msg/referee_system/RobotHeat.msg"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/msg/referee_system/RobotShoot.msg"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/msg/referee_system/RobotStatus.msg"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/msg/referee_system/SupplierStatus.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/srv" TYPE FILE FILES
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/srv/GimbalMode.srv"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/srv/FricWhl.srv"
    "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/srv/ShootCmd.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/cmake" TYPE FILE FILES "/home/cx/brainnavi/testRobot/src/cmake-build-debug/RoboRTS/roborts_msgs/catkin_generated/installspace/roborts_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/cx/brainnavi/testRobot/devel/include/roborts_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/cx/brainnavi/testRobot/devel/share/roseus/ros/roborts_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/cx/brainnavi/testRobot/devel/share/common-lisp/ros/roborts_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/cx/brainnavi/testRobot/devel/share/gennodejs/ros/roborts_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/cx/brainnavi/testRobot/devel/lib/python2.7/dist-packages/roborts_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/cx/brainnavi/testRobot/devel/lib/python2.7/dist-packages/roborts_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cx/brainnavi/testRobot/src/cmake-build-debug/RoboRTS/roborts_msgs/catkin_generated/installspace/roborts_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/cmake" TYPE FILE FILES "/home/cx/brainnavi/testRobot/src/cmake-build-debug/RoboRTS/roborts_msgs/catkin_generated/installspace/roborts_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs/cmake" TYPE FILE FILES
    "/home/cx/brainnavi/testRobot/src/cmake-build-debug/RoboRTS/roborts_msgs/catkin_generated/installspace/roborts_msgsConfig.cmake"
    "/home/cx/brainnavi/testRobot/src/cmake-build-debug/RoboRTS/roborts_msgs/catkin_generated/installspace/roborts_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roborts_msgs" TYPE FILE FILES "/home/cx/brainnavi/testRobot/src/RoboRTS/roborts_msgs/package.xml")
endif()

