cmake_minimum_required(VERSION 3.0.0)
project(gmapping VERSION 0.1.0)

# # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -O0 -Wall")
add_compile_options(-g3 -O0 -Wall)

# # set(PROJECT_BINARY_DIR ${PROJECT_BINARY_DIR}/build)
# link_libraries(opencv_core opencv_imgproc opencv_highgui)

# add_executable(gmapping main.cpp)

# ==============================correct above===================================== #

include_directories( ${PROJECT_SOURCE_DIR}/include )
message(STATUS "${PROJECT_SOURCE_DIR}/include")
# specific opencv cmake configuration file
# use OpenCVConfig.cmake by default, cause /usr/bin/ld: can not found -lopencv_dep_cudart ...nppc...
# in newer opencv version, use modules.cmake instead
set(OpenCV_DIR /usr/local/share/OpenCV/)
# set(OpenCV_DIR /home/shangjun/下载/opencv-2.4.13.6/build_laptop)
# set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
# set(CUDA_USE_SHARED_CUDA_RUNTIME OFF)
find_package(OpenCV REQUIRED)
message(STATUS "opencv libs: ${OpenCV_LIBS}")

# link_directories(/usr/local/cuda/targets/x86_64-linux/lib/)

# set include path and link path manually
# set(opencv_dir /usr/local/include)
# set(opencv_lib /usr/local/lib)
# set(boost_dir /usr/local/include)
# include_directories(${opencv_dir})
# link_directories(${opencv_lib})

# find library use name or likeli-name with sepcific path, path is noneed if lib in system directories
# find_library(opencv_lib opencv_core /usr/local/lib)
# find_library(opencv_lib1 opencv_highgui)
# find_library(opencv_lib2 opencv_imgproc)
# find_library(glib_lib glib-2.0 /usr/lib/x86_64-linux-gnu)
# message(STATUS " > opencv lib: ${opencv_lib}")
# message(STATUS " > glib-2.0 lib: ${glib_lib}")

aux_source_directory(. SOURCES)
aux_source_directory(src/gridfastslam GRIDSOURCES)
aux_source_directory(src/motionmodel MODELSOURCES)
# aux_source_directory(src/particlefilter FILTERSOURCES)
# add_library(filter SHARED ${FILTERSOURCES} )
aux_source_directory(src/scanmatcher MATCHERSOURCES)
aux_source_directory(src/sensor SENSORSOURCES)
aux_source_directory(src/utils UTILSOURCES)

link_libraries(${OpenCV_LIBS})
list(APPEND SOURCES ${GRIDSOURCES} ${MODELSOURCES} ${MATCHERSOURCES} ${SENSORSOURCES} ${UTILSOURCES})
message(STATUS "source files: ${SOURCES}")
add_executable(gmapping ${SOURCES})

# target_link_libraries(gmapping utils gridfastslam motionmodel matcher sensor ${OpenCV_LIBS})


include(CTest)
enable_testing()

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)