project( HKVision_driver )

add_library(HKVision_driver
  SHARED
  HKVision_driver.cpp
)

INCLUDE_DIRECTORIES(/home/tdt/catkin_ws/src/RoboRTS-ros/roborts_camera/HKVision/include)
link_directories(/home/tdt/catkin_ws/src/RoboRTS-ros/roborts_camera/HKVision/include/64)

target_link_libraries( HKVision_driver 
        PRIVATE
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        libGCBase_gcc421_v3_0.so
        libGenApi_gcc421_v3_0.so
        libImageProcess.so
        liblog4cpp_gcc421_v3_0.so
        libLog_gcc421_v3_0.so
        libMathParser_gcc421_v3_0.so
        libMediaProcess.so
        libMvCameraControl.so
        libMVGigEVisionSDK.so
        libMVRender.so
        libMvUsb3vTL.so
        libNodeMapData_gcc421_v3_0.so
        libXmlParser_gcc421_v3_0.so
        )

target_include_directories(HKVision_driver
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
)

add_library(driver::HKVision_driver ALIAS HKVision_driver)


