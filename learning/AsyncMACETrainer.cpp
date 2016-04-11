#include "AsyncMACETrainer.h"
#include "MACETrainer.h"

cAsyncMACETrainer::cAsyncMACETrainer()
{
	mNumActionFrags = 1;
	mActionFragSize = 1;
}

cAsyncMACETrainer::~cAsyncMACETrainer()
{
}

void cAsyncMACETrainer::SetNumActionFrags(int num)
{
	mNumActionFrags = num;
	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto mace_trainer = std::static_pointer_cast<cMACETrainer>(mTrainers[i]);
		mace_trainer->SetNumActionFrags(mNumActionFrags);
	}
}

void cAsyncMACETrainer::SetActionFragSize(int size)
{
	mActionFragSize = size;
	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto mace_trainer = std::static_pointer_cast<cMACETrainer>(mTrainers[i]);
		mace_trainer->SetActionFragSize(mActionFragSize);
	}
}

void cAsyncMACETrainer::BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const
{
	out_trainer = std::shared_ptr<cNeuralNetTrainer>(new cMACETrainer());
}

void cAsyncMACETrainer::SetupTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer)
{
	cAsyncTrainer::SetupTrainer(out_trainer);

	auto mace_trainer = std::static_pointer_cast<cMACETrainer>(out_trainer);
	mace_trainer->SetNumActionFrags(mNumActionFrags);
	mace_trainer->SetActionFragSize(mActionFragSize);
}