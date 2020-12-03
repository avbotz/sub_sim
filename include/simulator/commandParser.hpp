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

	std::string command;
	std::string fname;
	std::vector<std::string> elements;
	std::ofstream out;

	void init(std::string);
	void split();
	int available();
	char read();
	int parseInt();
	float parseFloat();
};

#endif