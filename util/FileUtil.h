#pragma once

#include <vector>
#include <string>
#include <fstream>

class cFileUtil
{
public:
	static FILE* OpenFile(const std::string& file_name, const char* mode);
	static FILE* OpenFile(const char* file_name, const char* mode);
	static void CloseFile(FILE*& f);
	static void ClearFile(const std::string& file_name);
	static void DeleteFile(const char* file_name);
	static std::string RemoveExtension(const std::string& filename);
	static void DeleteFile(const std::string& filename);
	static long int GetFileSize(const std::string& filename);
	static std::string GetExtension(const std::string& filename);
	static void FilterFilesByExtension(std::vector<std::string>& files, const std::string& ext);

	static void FindLine(std::ifstream& f_stream, int line);
	static std::string ReadTextFile(const std::string& path);

	// static bool ReadArray(FILE* f, const std::string& tag_beg, const std::string& tag_end, std::vector<double>& out_buffer);
	// static bool ReadTable(FILE* f, const std::string& tag_beg, const std::string& tag_end, std::vector<std::vector<double>>& out_buffer);

	static bool AppendText(const std::string& str, const std::string& out_filename);

private:
	static std::string ReadTextFile(FILE* f);
};
