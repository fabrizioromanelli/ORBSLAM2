/// Created by Felix Holz on 2018-08-17

#pragma once

#include <iostream>
class FileUtils
{
public:
	static std::string read_file(const char* path)
	{
		//Flags r -> read, t -> text file
		FILE* file = fopen(path, "rt");
		//Go to the end of the File
		fseek(file, 0, SEEK_END);
		//Get the length in bytes
		unsigned long length = ftell(file);
		char* data = new char[length + 1];
		//set whole array to 0
		memset(data, 0, length + 1);
		//reset seek pos to beginning
		fseek(file, 0, SEEK_SET);
		//read
		fread(data, 1, length, file);
		fclose(file);

		std::string result(data);
		delete[] data;

		return result;
	}
};

