#include "Rand.h"
#include <time.h>
#include <assert.h>
#include <algorithm>

cRand::cRand()
{
	mRandGen = std::default_random_engine(static_cast<unsigned long int>(time(NULL)));
	mRandDoubleDist = std::uniform_real_distribution<double>(0, 1);
	mRandDoubleDistNorm = std::normal_distribution<double>(0, 1);
	mRandIntDist = std::uniform_int_distribution<int>(0, std::numeric_limits<int>::max());
}

cRand::~cRand()
{
}

double cRand::RandDouble()
{
	return mRandDoubleDist(mRandGen);
}

double cRand::RandDouble(double min, double max)
{
	if (min == max)
	{
		return min;
	}

	// generate random double in [min, max]
	double rand_double = mRandDoubleDist(mRandGen);
	rand_double = min + (rand_double * (max - min));
	return rand_double;
}

double cRand::RandDoubleNorm(double mean, double stdev)
{
	double rand_double = mRandDoubleDistNorm(mRandGen);
	rand_double = mean + stdev * rand_double;
	return rand_double;
}

int cRand::RandInt()
{
	return mRandIntDist(mRandGen);
}

int cRand::RandInt(int min, int max)
{
	if (min == max)
	{
		return min;
	}

	// generate random double in [min, max)
	int delta = max - min;
	int rand_int = mRandIntDist(mRandGen);
	rand_int = min + rand_int % delta;

	return rand_int;
}

int cRand::RandIntExclude(int min, int max, int exc)
{
	int rand_int = 0;
	if (exc < min || exc >= max)
	{
		rand_int = RandInt(min, max);
	}
	else
	{
		int new_max = max - 1;
		if (new_max <= min)
		{
			rand_int = min;
		}
		else
		{
			rand_int = RandInt(min, new_max);
			if (rand_int >= exc)
			{
				++rand_int;
			}
		}
	}
	return rand_int;
}

void cRand::Seed(unsigned long int seed)
{
	mRandGen.seed(seed);
}

int cRand::RandSign()
{
	return FlipCoin() ? -1 : 1;
}

bool cRand::FlipCoin(double p)
{
	return (RandDouble(0, 1) < p);
}
