find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(show show.cpp)
target_link_libraries(show ${OpenCV_LIBS})