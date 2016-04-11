#pragma once
#include <ctime>

#define TIMER_PRINT_BEG(NAME) \
	static int time_count_ ## NAME = 0; \
	static double avg_time_ ## NAME = 0; \
	std::clock_t total_time_beg_ ## NAME = std::clock();

#define TIMER_PRINT_END(NAME) \
	std::clock_t total_time_end_ ## NAME = std::clock(); \
	double time_elapsed_ ## NAME = static_cast<double>(total_time_end_ ## NAME - total_time_beg_ ## NAME) / CLOCKS_PER_SEC; \
	avg_time_ ## NAME = (avg_time_ ## NAME * time_count_ ## NAME + time_elapsed_ ## NAME) / (time_count_ ## NAME + 1); \
	++time_count_ ## NAME; \
	printf("Timer(%s): %.8fs, samples: %i\n", #NAME, avg_time_ ## NAME, time_count_ ## NAME);


#define TIMER_RECORD_BEG(NAME) \
	static int time_count_ ## NAME = 0; \
	static double avg_time_ ## NAME = 0; \
	std::clock_t total_time_beg_ ## NAME = std::clock();

#define TIMER_RECORD_END(NAME, TIME_REC, COUNT_REC) \
	std::clock_t total_time_end_ ## NAME = std::clock(); \
	double time_elapsed_ ## NAME = static_cast<double>(total_time_end_ ## NAME - total_time_beg_ ## NAME) / CLOCKS_PER_SEC; \
	TIME_REC = (TIME_REC * COUNT_REC + time_elapsed_ ## NAME) / (COUNT_REC + 1); \
	++COUNT_REC;