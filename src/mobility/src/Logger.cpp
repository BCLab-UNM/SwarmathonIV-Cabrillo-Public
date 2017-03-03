#include "Logger.h"

#include <std_msgs/String.h>

#include <cstdio>

Logger *Logger::_instance = NULL;

Logger::Logger(const std::string &rover) {
	ros::NodeHandle mNH;
	_log = mNH.advertise<std_msgs::String>(rover + "/infoLog", 1, true);
	_chat = mNH.advertise<std_msgs::String>(rover + "/state_machine", 1, true);
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
	int got = vsnprintf (_instance->_logbuf, LOGGER_BUFFER_SIZE, format, args);
	va_end(args);
	if (got > 0 && strncmp(_instance->_logbuf, _instance->_loglast, LOGGER_BUFFER_SIZE)) {
		strncpy(_instance->_loglast, _instance->_logbuf, LOGGER_BUFFER_SIZE);
		std_msgs::String m;
		m.data = _instance->_logbuf;
		_instance->_log.publish(m);
	}
}

void Logger::chat(const char *format, ...) {
	if (_instance == NULL)
		return;

	va_list args;
	va_start (args, format);
	int got = vsnprintf (_instance->_chatbuf, LOGGER_BUFFER_SIZE, format, args);
	va_end(args);
	if (got > 0 && strncmp(_instance->_chatbuf, _instance->_chatlast, LOGGER_BUFFER_SIZE)) {
		strncpy(_instance->_chatlast, _instance->_chatbuf, LOGGER_BUFFER_SIZE);
		std_msgs::String m;
		m.data = _instance->_chatbuf;
		_instance->_chat.publish(m);
	}
}
