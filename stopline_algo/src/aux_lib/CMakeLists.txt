cmake_minimum_required(VERSION 2.8.3)
project(aux_lib)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  traffic_msgs
  roscpp
  prediction_ct_vel
)

find_package(Eigen3 REQUIRED)

find_package(PythonLibs 3.8 REQUIRED)

# Instruct CMake to run moc automatically when needed.
find_package(Qt5Widgets REQUIRED)
find_library(QCUSTOMPLOT_LIB_DEBUG NAMES qcustomplotd
    HINTS /usr/local/lib/QCustomPlot)
find_library(QCUSTOMPLOT_LIB_RELEASE NAMES qcustomplot
    HINTS /usr/local/lib/QCustomPlot)
set(QCUSTOMPLOT_INCLUDE_DIRS /usr/local/include)

catkin_package(LIBRARIES aux_lib plot_lib
    INCLUDE_DIRS include
    CATKIN_DEPENDS std_msgs
    traffic_msgs
    roscpp
    prediction_ct_vel
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${QCUSTOMPLOT_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(aux_lib
    src/aux_functions.cpp
    src/clipper.cpp
    src/st_cell_planner.cpp
)

add_dependencies(aux_lib
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(aux_lib
    PUBLIC ${catkin_LIBRARIES}
)

# Declare a C++ plot library
add_library(plot_lib
    src/plot_functions.cpp
)

target_compile_definitions(plot_lib
    PUBLIC -DQCUSTOMPLOT_USE_LIBRARY
)

add_dependencies(plot_lib
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(plot_lib
    ${catkin_LIBRARIES}
    ${PYTHON_LIBRARIES}
    Qt5::Widgets
    debug ${QCUSTOMPLOT_LIB_DEBUG}
    optimized ${QCUSTOMPLOT_LIB_RELEASE}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
)

# Unit testing
catkin_add_gtest(${PROJECT_NAME}-test test/aux_funcs_unittests.cpp)

target_link_libraries(${PROJECT_NAME}-test
    aux_lib
    plot_lib
)
