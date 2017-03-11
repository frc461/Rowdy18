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
#include <sys/time.h>

#define DATETIME_FORMAT "%m-%d-%Y_%I:%M:%S"

FILE *Logger::currentLogFile = NULL;
timeval Logger::openTime;

void Logger::GetCurrentDateTime(char *buffer, int size) {
	time_t rawtime;
	struct tm * timeinfo;

	time (&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, size, DATETIME_FORMAT, timeinfo);
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
	Logger::currentLogFile = fopen(logFileName, "w");
	fprintf(currentLogFile, "%s\n", logFileName);
	gettimeofday(&openTime, 0);
}

void Logger::Log(LOG_TYPE logType, const char* s, ...) {
	va_list args;
	va_start(args, s);
	fprintf(currentLogFile, "%s> ", LOG_PREFIXES[logType]);
	vfprintf(currentLogFile, s, args);
	va_end(args);
}

void Logger::LogPID(LOG_TYPE logType, const char* name, BetterPIDController* pid) {
	Log(logType, "%s: P: %lf, I: %'lf, D: %lf, Input: %lf, Setpoint: %lf, Error: %lf, Output: %lf\n",
			name,
	    pid->GetP(),
			pid->GetI(),
			pid->GetD(),
			pid->GetInput(),
			pid->GetSetpoint(),
			pid->GetError(),
			pid->Get());
}

void Logger::LogRunTime() {
	timeval tv;
	gettimeofday(&tv, 0);
	fprintf(currentLogFile, "%ld.%ld\n", tv.tv_sec - openTime.tv_sec, tv.tv_usec - openTime.tv_usec);
}

void Logger::CloseLog() {
	if (currentLogFile != NULL) {
		fclose(currentLogFile);
		currentLogFile = NULL;
	}
}
