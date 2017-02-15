#ifndef __LOGGER_H
#define __LOGGER_H

#include <ros/ros.h>

#define LOGGER_BUFFER_SIZE 1024

class Logger {

public:

	static void init(const std::string &rover);
	static void log(const char *format, ...);

	~Logger();

private:
	static Logger *_instance;

	Logger(const std::string &rover);

	ros::Publisher _chat;
	char _buffer[LOGGER_BUFFER_SIZE];
	char _lastmsg[LOGGER_BUFFER_SIZE];
};

#endif
