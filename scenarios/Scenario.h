#pragma once

#include <string>
#include <functional>
#include "util/ArgParser.h"

class cScenario
{
public:
	typedef std::function<void()> tCallbackFunc;
	typedef std::function<void(double)> tTimeCallbackFunc;

	virtual ~cScenario();

	virtual void Init();
	virtual void ParseArgs(const cArgParser& parser);
	virtual void Reset();
	virtual void Clear();
	virtual void Run();
	virtual void Shutdown();

	virtual bool IsDone() const;
	virtual void Update(double time_elapsed);
	virtual void SetResetCallback(tCallbackFunc func);

	virtual std::string GetName() const;

protected:
	tCallbackFunc mResetCallback;

	cScenario();
};