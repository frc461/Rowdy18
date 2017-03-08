/*
 * Logger.h
 *
 *  Created on: Feb 18, 2017
 *      Author: WBI
 */

#ifndef SRC_LOGGER_H_
#define SRC_LOGGER_H_

#include "LogTypes.h"

#include <iostream>
#include "WPILib.h"
#include "BetterPIDController.h"
#include <sys/time.h>

class Logger {
public:
	static void OpenNewLog(const char *suffix = "", const char *extension = ".txt");
	static void CloseLog();
	static void Log(LOG_TYPE logType, const char* s, ...);
	static void LogPID(LOG_TYPE logtype, BetterPIDController *pid);
	static void LogRunTime();

private:
	static void GetCurrentDateTime(char *buf, int size);
	static FILE *currentLogFile;
	static timeval openTime;
};

#endif /* SRC_LOGGER_H_ */
