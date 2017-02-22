/*
 * Logger.cpp
 *
 *  Created on: Feb 18, 2017
 *      Author: WBI
 */

#include "Logger.h"
#include <sys/stat.h>
#include <string.h>
#include <stdarg.h>

#define DATETIME_FORMAT "%m-%d-%Y_%I:%M:%S"

Logger::Logger() {
	// TODO Auto-generated constructor stub
	currentLogFile = NULL;
}

void Logger::GetCurrentDateTime(char *buffer, int size) {
	time_t rawtime;
	struct tm * timeinfo;

	time (&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, size, DATETIME_FORMAT ".txt",timeinfo);
}

void Logger::OpenNewLog(const char *suffix, const char *extension) {
	const char *path = "/home/lvuser/Logs/";
	mkdir(path, S_IRWXU | S_IRWXG | S_IRWXO);

	int timeBufSize = 80;
	char timeBuf[timeBufSize];
	GetCurrentDateTime(timeBuf, timeBufSize);
	char logFileName[strlen(path) + strlen(timeBuf) + strlen(suffix) + strlen(extension) + 1];
	sprintf(logFileName, "%s%s%s%s", path, timeBuf, suffix, extension);
	printf("Writing to log file: %s\n", logFileName);
	currentLogFile = fopen(logFileName, "w");
}

void Logger::Log(LOG_TYPE logType, const char* s, ...) {
	va_list args;
	va_start(args, s);
	fprintf(currentLogFile, "%s> ", LOG_PREFIXES[logType]);
	fprintf(currentLogFile, s, args);
	va_end(args);
}

void Logger::CloseLog() {
	if (currentLogFile != NULL) {
		fclose(currentLogFile);
		currentLogFile = NULL;
	}
}

Logger::~Logger() {
	// TODO Auto-generated destructor stub
	CloseLog();
}

