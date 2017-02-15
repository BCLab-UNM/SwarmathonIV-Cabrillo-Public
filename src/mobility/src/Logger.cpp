#include "Logger.h"

#include <std_msgs/String.h>

#include <cstdio>

Logger *Logger::_instance = NULL;

Logger::Logger(const std::string &rover) {
	ros::NodeHandle mNH;
	_chat = mNH.advertise<std_msgs::String>(rover + "/infoLog", 1, true);
}

Logger::~Logger() {
}

void Logger::init(const std::string &rover) {
	if (_instance == NULL) {
		_instance = new Logger(rover);
	}
}

void Logger::log(const char *format, ...) {
	if (_instance == NULL)
		return;

	va_list args;
	va_start (args, format);
	int got = vsnprintf (_instance->_buffer, LOGGER_BUFFER_SIZE, format, args);
	va_end(args);
	if (got > 0 && strncmp(_instance->_buffer, _instance->_lastmsg, LOGGER_BUFFER_SIZE)) {
		strncpy(_instance->_lastmsg, _instance->_buffer, LOGGER_BUFFER_SIZE);
		std_msgs::String m;
		m.data = _instance->_buffer;
		_instance->_chat.publish(m);
	}
}
