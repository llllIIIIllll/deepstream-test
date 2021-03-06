cmake_minimum_required(VERSION 3.5)
project(ros2_videostreamer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -O2) #-Wpedantic
endif()

# add host cpu option
configure_file(
	"${PROJECT_SOURCE_DIR}/host_cpu.h.in"
	"${PROJECT_BINARY_DIR}/host_cpu.h"
)

#####################
# find dependencies #
#####################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(perception_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
#find_package(GSTREAMER REQUIRED gstreamer-1.0)

#################
# include files #
#################
include_directories(
	include
	${GSTREAMER_INCLUDE_DIRS}
	${PROJECT_BINARY_DIR}
	/usr/include/gstreamer-1.0/
)
set(DS_LIB /opt/nvidia/deepstream/deepstream-4.0/lib)
link_libraries(
		/opt/nvidia/deepstream/deepstream-4.0/lib/libnvdsgst_meta.so
		/opt/nvidia/deepstream/deepstream-4.0/lib/libnvds_meta.so
)

###########################
# compile target function #
###########################
function(custome_executable subfolder target addition)
	add_executable(${target} src/${subfolder}/${target}.cpp 
							 ${ARGV2}
							 ${ARGV3}
							 ${ARGV4}
							 #${ARGV5}
	)
	target_link_libraries(${target}
		${GSTREAMER_LIBRARIES} 
		${OpenCV_LIBS}
		pthread
		gstapp-1.0
	)
	ament_target_dependencies(${target}
		"rclcpp"
		"rcl"
		"rclcpp"
		# "rclcpp_action"
		"std_msgs"
		"std_srvs"
		"sensor_msgs"
		"geometry_msgs"  
		"perception_msgs"
	)
	install(TARGETS ${target}
	DESTINATION lib/${PROJECT_NAME})
endfunction()

######################
# set addition files #
######################
SET(USB_ADDITION 
	 src/Receiver.cpp
	 src/usb_receiver/UsbReceiver.cpp
	 src/usb_receiver_component.cpp
)
SET(RTSP_ADDITION 
	 src/Receiver.cpp
	 src/rtsp_receiver/RtspReceiver.cpp
	 src/rtsp_receiver_component.cpp
)
######################
# call exec function #
######################
custome_executable(receiver usb_node ${USB_ADDITION})
custome_executable(receiver rtsp_node ${RTSP_ADDITION})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
  
############
# set test #
############
if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# remove the line when a copyright and license is present in all source files
	set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# remove the line when this package is a git repo
	set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
