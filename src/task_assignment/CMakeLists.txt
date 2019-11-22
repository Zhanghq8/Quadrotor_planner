# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(PythonLibs 2.7)
find_package(Eigen3 3.1.2)

# Add libraries
set(TASKASSIGNMENT_LIB_SRC
    cbba.hpp
    cbba_impl.hpp  
)

add_library(taskassignment INTERFACE)
target_sources(taskassignment INTERFACE ${TASKASSIGNMENT_LIB_SRC})
target_include_directories(taskassignment INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>)


