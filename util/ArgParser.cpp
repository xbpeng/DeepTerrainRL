#include "ArgParser.h"

#include <assert.h>
#include <fstream>
#include <iostream>
#include <sstream>

#include "FileUtil.h"

const int gInvalidIndex = -1;
const char gKeyStart = '-';
const char gKeyEnd = '=';

cArgParser::cArgParser()
{
}

cArgParser::cArgParser(char **args, int num_args)
{
	AppendArgs(args, num_args);
}

cArgParser::cArgParser(const std::string& file)
{
	AppendArgs(file);
}

void cArgParser::AppendArgs(char **args, int num_args)
{
	for (int i = 0; i < num_args; ++i)
	{
		std::string curr_str = std::string(args[i]);
		mArgs.push_back(curr_str);
	}
}

void cArgParser::Clear()
{
	mArgs.clear();
}

void cArgParser::AppendArgs(const std::string& file)
{
	bool object_found = false;
	bool is_comment = false;
	char buffer;
	std::string str_buffer = "";
	int slash_count = 0;

	FILE* file_ptr = cFileUtil::OpenFile(file.c_str(), "r");

	if (file_ptr != NULL)
	{
		int items_read = static_cast<int>(fread(&buffer, sizeof(char), 1, file_ptr));

		while (items_read > 0)
		{
			{
				if (!is_comment)
				{
					if ((buffer != '/') && (slash_count == 1))
					{
						str_buffer += '/';
						slash_count = 0;
					}

					if ((buffer == ' ') || (buffer == '\t')
						|| (buffer == '\n') || (buffer == '\r'))
					{
						if (str_buffer != "")
						{
							mArgs.push_back(str_buffer);
							str_buffer = "";
						}
					}
					else if (buffer == '/')
					{
						++slash_count;
						if (slash_count >= 2)
						{
							is_comment = true;
						}
					}
					else
					{
						str_buffer += buffer;
					}
				}

				if (buffer == '\n')
				{
					is_comment = false;
					slash_count = 0;
				}
			}

			items_read = static_cast<int>(fread(&buffer, sizeof(char), 1, file_ptr));
		}

		// dump everything else out
		if (str_buffer != "")
		{
			mArgs.push_back(str_buffer);
		}
	}

	cFileUtil::CloseFile(file_ptr);
}

bool cArgParser::ParseString(const std::string& key, std::string& out) const
{
	int key_idx = FindKeyIndex(key);
	if (IsValidKeyIndex(key_idx))
	{
		int val_idx = key_idx + 1;
		const std::string& val = mArgs[val_idx];
		if (!IsKey(val))
		{
			out = val;
			return true;
		}
	}

	return false;
}

bool cArgParser::ParseStringArray(const std::string& key, std::vector<std::string>& out) const
{
	out.clear();
	int key_idx = FindKeyIndex(key);
	if (IsValidKeyIndex(key_idx))
	{
		for (size_t i = key_idx + 1; i < mArgs.size(); ++i)
		{
			const std::string& val = mArgs[i];
			if (!IsKey(val))
			{
				out.push_back(val);
			}
			else
			{
				break;
			}
		}
		return true;
	}

	return false;
}

bool cArgParser::ParseInt(const std::string& key, int& out) const
{
	int key_idx = FindKeyIndex(key);
	if (IsValidKeyIndex(key_idx))
	{
		int val_idx = key_idx + 1;
		const std::string& val = mArgs[val_idx];
		if (!IsKey(val))
		{
			out = std::atoi(val.c_str());
			return true;
		}
	}

	return false;
}

bool cArgParser::ParseIntArray(const std::string& key, std::vector<int>& out) const
{
	out.clear();
	int key_idx = FindKeyIndex(key);
	if (IsValidKeyIndex(key_idx))
	{
		for (size_t i = key_idx + 1; i < mArgs.size(); ++i)
		{
			const std::string& val = mArgs[i];
			if (!IsKey(val))
			{
				out.push_back(std::atoi(val.c_str()));
			}
			else
			{
				break;
			}
		}
		return true;
	}

	return false;
}

bool cArgParser::ParseDouble(const std::string& key, double& out) const
{
	int key_idx = FindKeyIndex(key);
	if (IsValidKeyIndex(key_idx))
	{
		int val_idx = key_idx + 1;
		const std::string& val = mArgs[val_idx];
		if (!IsKey(val))
		{
			out = static_cast<double>(std::atof(val.c_str()));
			return true;
		}
	}

	return false;
}

bool cArgParser::ParseDoubleArray(const std::string& key, std::vector<double>& out) const
{
	out.clear();
	int key_idx = FindKeyIndex(key);
	if (IsValidKeyIndex(key_idx))
	{
		for (size_t i = key_idx + 1; i < mArgs.size(); ++i)
		{
			const std::string& val = mArgs[i];
			if (!IsKey(val))
			{
				out.push_back(static_cast<double>(std::atof(val.c_str())));
			}
			else
			{
				break;
			}
		}
		return true;
	}

	return false;
}

bool cArgParser::ParseBool(const std::string& key, bool& out) const
{
	int key_idx = FindKeyIndex(key);
	if (IsValidKeyIndex(key_idx))
	{
		int val_idx = key_idx + 1;
		const std::string& val = mArgs[val_idx];
		if (!IsKey(val))
		{
			if (val == "true" || val == "1"
				|| val == "True" || val == "T"
				|| val == "t")
			{
				out = true;
				return true;
			}
			else if (val == "false" || val == "0"
				|| val == "False" || val == "F"
				|| val == "f")
			{
				out = false;
				return true;
			}
			else
			{
				return false;
			}
		}
	}

	return false;
}

std::string cArgParser::FormatKey(const std::string& key) const
{
	std::string format_key = key;
	size_t key_size = key.size();
	if (key_size < 1)
	{
		return "";
	}
	else
	{
		if (key[0] != gKeyStart)
		{
			format_key = gKeyStart + key;
		}

		if (key[key.size() - 1] != gKeyEnd)
		{
			format_key = format_key + gKeyEnd;
		}
	}

	return format_key;
}

bool cArgParser::IsKey(const std::string& str) const
{
	size_t len = str.size();
	if (len < 3)
	{
		return false;
	}
	else
	{
		if (str[0] == gKeyStart
			&& str[len - 1] == gKeyEnd)
		{
			return true;
		}
	}
	return false;
}

int cArgParser::GetNumArgs() const
{
	return static_cast<int>(mArgs.size());
}

bool cArgParser::IsValidKeyIndex(int idx) const
{
	bool valid = (idx != gInvalidIndex) && (idx >= 0);
	return valid;
}

int cArgParser::FindKeyIndex(const std::string& key) const
{
	std::string format_key = FormatKey(key);
	for (int i = 0; i < GetNumArgs(); ++i)
	{
		const std::string& curr_str = mArgs[i];
		bool found = curr_str == format_key;
		if (found)
		{
			return i;
		}
	}
	return gInvalidIndex;
}

cArgParser::~cArgParser()
{
}