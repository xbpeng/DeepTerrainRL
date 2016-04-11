#include "AsyncQNetTrainer.h"
#include "QNetTrainer.h"

cAsyncQNetTrainer::cAsyncQNetTrainer()
{
}

cAsyncQNetTrainer::~cAsyncQNetTrainer()
{
}

void cAsyncQNetTrainer::BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const
{
	out_trainer = std::shared_ptr<cNeuralNetTrainer>(new cQNetTrainer());
}