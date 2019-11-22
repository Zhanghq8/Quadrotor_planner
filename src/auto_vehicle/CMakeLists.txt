# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(PythonLibs 2.7)
find_package(Eigen3 3.1.2)

# Add libraries
set(AUTOVEHICLE_LIB_SRC
    src/auto_vehicle.cpp  
    src/tasks.cpp
)

add_library(auto_vehicle STATIC ${AUTOVEHICLE_LIB_SRC})
target_link_libraries(auto_vehicle map)

target_include_directories(auto_vehicle PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

add_executable(test_cbba test/test_cbba.cpp)
target_link_libraries(test_cbba auto_vehicle map)

add_executable(test_ipas test/test_ipas.cpp)
target_link_libraries(test_ipas auto_vehicle map)


