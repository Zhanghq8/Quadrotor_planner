# Dependency libraries
find_package(OpenCV REQUIRED)
find_package(PythonLibs 2.7)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Add libraries
set(LTL_LIB_SRC
    src/buchi_automaton.cpp
    src/spot_hoa_interpreter.cpp
    src/product_automaton.cpp
)

add_library(ltl STATIC ${LTL_LIB_SRC})
target_link_libraries(ltl map spot bddx auto_vehicle)
target_include_directories(ltl PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)


add_executable(test_buchi test/test_buchi.cpp)
target_link_libraries(test_buchi ltl ${catkin_LIBRARIES})

add_executable(test_product test/test_product.cpp)
target_link_libraries(test_product ltl map ${catkin_LIBRARIES})



