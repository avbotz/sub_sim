/** @file commandParser.hpp
 *  @brief Helper struct to send messages between Porpoise and simulation controller.
 *
 *  @author Craig Wang
 */
#ifndef SIMULATOR_COMMANDPARSER_HPP
#define SIMULATOR_COMMANDPARSER_HPP

#include <iostream>
#include <string>
#include <vector>
#include <fstream>

/* Emulating Nautical's command parsing */
struct commandParser
{
	commandParser() {}

	FILE* in;
	std::string command;
	std::vector<std::string> elements;

	void init(const char *);
	void split();
	int available();
	char read();
	int parseInt();
	float parseFloat();
};

#endif