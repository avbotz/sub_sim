/** @file commandParser.cpp
 *  @brief Struct that helps the simulation controller communicate with Porpoise.
 *
 *  @author Craig Wang
 */

#include "simulator/commandParser.hpp"

void commandParser::init(const char* filename)
{
	this->in = fopen(filename, "r+");
}

void commandParser::split()
{
	/* Split string commands by space into a vector */
	char sep = ' ';
	std::size_t start = 0, end = 0;
	while ((end = this->command.find(sep, start)) != std::string::npos) 
	{
		this->elements.push_back(this->command.substr(start, end - start));
		start = end + 1;
	}
	this->elements.push_back(this->command.substr(start));
}

char commandParser::read()
{
	/* Return first character of string command */
	if (this->elements.size() > 0)
	{
		char c = this->elements[0][0];
		this->elements.erase(this->elements.begin());
		return c;
	}
	return '0';
}

int commandParser::parseInt()
{
	/* Takes string command and returns first number as int */
	if (this->elements.size() > 0)
	{
		int result = std::stoi(this->elements[0]);
		this->elements.erase(this->elements.begin());
		return result;
	}
	return 0;
}

float commandParser::parseFloat()
{
	/* Takes string command and returns first number as float */
	if (this->elements.size() > 0)
	{
		float result = std::stof(this->elements[0]);
		this->elements.erase(this->elements.begin());
		return result;
	}
	return 0.;
}

int commandParser::available()
{
	/* Reads command from file, returns number of nonspace characters in the command */
	char line[200];
	if (fgets(line, 200, this->in) != NULL)
	{
		std::string str(line);
		this->command = str;
	}
	else this->command = "";
	this->split();
	return this->elements.size();
}