set (CMAKE_CXX_FLAGS "--std=gnu++11 -O0 -g3 -Wall -fmessage-length=0 ${CMAKE_CXX_FLAGS}")

include_directories( CMAKE_CURRENT_SOURCE_DIR $ENV{WINHOME}/wpilib/cpp/current/include)
link_directories($ENV{WINHOME}/wpilib/user/cpp/lib $ENV{WINHOME}/wpilib/cpp/current/lib)
set (SOURCE_FILES Robot.cpp RateEncoder.cpp)

add_executable( FRCUserProgram ${SOURCE_FILES})
target_link_libraries(FRCUserProgram wpi)
