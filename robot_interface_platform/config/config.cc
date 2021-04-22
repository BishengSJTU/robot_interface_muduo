#include "config.h"

void Config::setParameterFile(const std::string& file_name)
{
	file_name_ = file_name;
}
Config::Config()
{

}
Config::Config(const std::string& file_name)
{
    file_name_ = file_name;
}
Config::~Config()
{

}