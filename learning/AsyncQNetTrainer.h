#pragma once

#include "learning/AsyncTrainer.h"

class cAsyncQNetTrainer : public cAsyncTrainer
{
public:

	cAsyncQNetTrainer();
	virtual ~cAsyncQNetTrainer();

protected:
	
	virtual void BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const;
};