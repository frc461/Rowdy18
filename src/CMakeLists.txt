include_directories( CMAKE_CURRENT_SOURCE_DIR $ENV{WINHOME}/wpilib/cpp/current/include)
set (SOURCE_FILES Robot.cpp RateEncoder.cpp)

add_executable( FRCUserProgram ${SOURCE_FILES})
