#pragma once

#include "learning/AsyncACTrainer.h"

class cAsyncCaclaTrainer : public cAsyncACTrainer
{
public:

	cAsyncCaclaTrainer();
	virtual ~cAsyncCaclaTrainer();

protected:
	
	virtual void BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const;
};