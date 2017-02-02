all: Robot.cpp Robot.h XboxJoystickMap.h RateEncoder.h RateEncoder.cpp
	arm-frc-linux-gnueabi-g++ -std=c++1y "-I$(WINHOME)/wpilib/cpp/current/include" "-I$(WINHOME)/workspace/DriveTrainBot/src" "-I$(WINHOME)/wpilib/user/cpp/include" -O0 -g3 -Wall -c -fmessage-length=0 -o "../Debug/src/RateEncoder.o" "RateEncoder.cpp"
	arm-frc-linux-gnueabi-g++ -std=c++1y "-I$(WINHOME)/wpilib/cpp/current/include" "-I$(WINHOME)/workspace/DriveTrainBot/src" "-I$(WINHOME)/wpilib/user/cpp/include" -O0 -g3 -Wall -c -fmessage-length=0 -o "../Debug/src/Robot.o" "Robot.cpp"

	arm-frc-linux-gnueabi-g++ "-L$(WINHOME)/wpilib/cpp/current/lib" "-L$(WINHOME)/wpilib/user/cpp/lib" -Wl,-rpath,/opt/GenICam_v3_0_NI/bin/Linux32_ARM -o "../Debug/src/FRCUserProgram" "../Debug/src/RateEncoder.o" "../Debug/src/Robot.o" -lwpi 

