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

class Logger {
public:
	Logger();
	void OpenNewLog(const char *suffix = "", const char *extension = ".txt");
	void CloseLog();
	void Log(LOG_TYPE logType, const char* s, ...);
	virtual ~Logger();

private:
	void GetCurrentDateTime(char *buf, int size);
	FILE *currentLogFile;
};

#endif /* SRC_LOGGER_H_ */
