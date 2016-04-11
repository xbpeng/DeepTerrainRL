#pragma once

#include <vector>
#include <string>

class cArgParser
{
public:
	cArgParser();
	cArgParser(char **args, int num_args);
	cArgParser(const std::string& file);
	virtual ~cArgParser();

	virtual void Clear();
	virtual void AppendArgs(char **args, int num_args);
	virtual void AppendArgs(const std::string& file);

	virtual int GetNumArgs() const;
	virtual bool ParseString(const std::string& key, std::string& out) const;
	virtual bool ParseStringArray(const std::string& key, std::vector<std::string>& out) const;
	virtual bool ParseInt(const std::string& key, int& out) const;
	virtual bool ParseIntArray(const std::string& key, std::vector<int>& out) const;
	virtual bool ParseDouble(const std::string& key, double& out) const;
	virtual bool ParseDoubleArray(const std::string& key, std::vector<double>& out) const;
	virtual bool ParseBool(const std::string& key, bool& out) const;
	virtual std::string FormatKey(const std::string& key) const;
	virtual bool IsKey(const std::string& str) const;

protected:
	std::vector<std::string> mArgs;

	virtual bool IsValidKeyIndex(int idx) const;
	virtual int FindKeyIndex(const std::string& key) const;
};