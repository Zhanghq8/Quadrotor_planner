# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(PythonLibs 2.7)
find_package(Eigen3 3.1.2)

# Add libraries
set(AUTOTEAM_LIB_SRC
    auto_team.hpp
    auto_team_impl.hpp
)

add_library(auto_team INTERFACE)
target_sources(auto_team INTERFACE ${VEHICLE_LIB_SRC})
target_include_directories(vehicle INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>)