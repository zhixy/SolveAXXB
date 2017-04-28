cmake_minimum_required(VERSION 2.8)
project(SolveAXXB)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)


file(GLOB SRC
    axxb/*
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Eigen
set(MIN_EIGEN_VERSION 3.2.0)
find_package(Eigen ${MIN_EIGEN_VERSION} REQUIRED)
if (EIGEN_FOUND)
  message("-- Found Eigen version ${EIGEN_VERSION}: ${EIGEN_INCLUDE_DIRS}")
endif (EIGEN_FOUND)


include_directories(
  ${EIGEN_INCLUDE_DIRS}
)

add_library(SolveAXXB SHARED ${SRC})
## Specify libraries to link a library or executable target against
target_link_libraries(SolveAXXB
  ${EIGEN_LIBRARIES}
)

install(TARGETS SolveAXXB
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
  
#install(FILES axxb/*.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
install(DIRECTORY axxb/ DESTINATION include/SolveAXXB FILES_MATCHING PATTERN "*.h")
