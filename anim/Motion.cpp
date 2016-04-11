#include "Motion.h"
#include <assert.h>
#include <iostream>

#include "util/FileUtil.h"

const double gMinTime = 0;

// Json keys
const std::string gMotionKey = "Motion";
const std::string gFrameKey = "Frames";
const std::string gLoopKey = "Loop";

cMotion::cMotion()
{
	Clear();
	mLoop = false;
}

cMotion::~cMotion()
{

}

void cMotion::Clear()
{
	mFrames.resize(0, 0);
}

bool cMotion::Load(const std::string& file)
{
	Clear();
	
	std::ifstream f_stream(file);
	Json::Value root;
	Json::Reader reader;
	bool succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ)
	{
		succ = LoadJson(root);
		if (succ)
		{
			PostProcessFrames(mFrames);
		}
		else
		{
			printf("Failed to load motion from file %s\n", file.c_str());
			assert(false);
		}
	}
	else
	{
		printf("Failed to parse Json from %s\n", file.c_str());
		assert(false);
	}
	return succ;
}

bool cMotion::IsValid() const
{
	return GetNumFrames() > 0;
}

int cMotion::GetNumDof() const
{
	return GetFrameSize() - 1;
}

int cMotion::GetNumFrames() const
{
	return static_cast<int>(mFrames.rows());
}

int cMotion::GetFrameSize() const
{
	return static_cast<int>(mFrames.cols());
}

cMotion::tFrame cMotion::GetFrame(int i) const
{
	int frame_size = GetFrameSize();
	return mFrames.row(i).segment(1, frame_size - 1);
}

cMotion::tFrame cMotion::BlendFrames(int a, int b, double lerp) const
{
	lerp = cMathUtil::Saturate(lerp);

	// remove time params
	tFrame frame0 = GetFrame(a); 
	tFrame frame1 = GetFrame(b);

	tFrame frame = (1 - lerp) * frame0 + lerp * frame1;
	return frame;
}

cMotion::tFrame cMotion::CalcFrame(double time) const
{
	int idx;
	double phase;
	CalcIndexPhase(time, idx, phase);

	tFrame frame = BlendFrames(idx, idx + 1, phase);
	return frame;
}

bool cMotion::LoadJson(const Json::Value& root)
{
	bool succ = true;
	if (!root[gMotionKey].isNull())
	{
		Json::Value motion = root.get(gMotionKey, 0);
		if (!motion[gLoopKey].isNull())
		{
			mLoop = motion[gLoopKey].asBool();
		}

		if (!motion[gFrameKey].isNull())
		{
			Json::Value frames = motion.get(gFrameKey, 0);
			assert(frames.isArray());
			int num_frames = frames.size();
			
			int data_size = 0;
			if (num_frames > 0)
			{
				int idx0 = 0;
				Json::Value frame_json = frames.get(idx0, 0);
				data_size = frame_json.size();
				mFrames.resize(num_frames, data_size);
			}

			for (int f = 0; f < num_frames; ++f)
			{
				Eigen::VectorXd curr_frame;
				succ = ParseFrameJson(frames.get(f, 0), curr_frame);
				if (succ)
				{
					assert(mFrames.cols() == curr_frame.size());
					mFrames.row(f) = curr_frame;
				}
				else
				{
					mFrames.resize(0, 0);
					break;
				}
			}
		}
	}
	return succ;
}

bool cMotion::ParseFrameJson(const Json::Value& root, Eigen::VectorXd& out_frame) const
{
	bool succ = false;
	if (root.isArray())
	{
		int data_size = root.size();
		out_frame.resize(data_size);
		for (int i = 0; i < data_size; ++i)
		{
			Json::Value json_elem = root.get(i, 0);
			out_frame[i] = json_elem.asDouble();
		}

		succ = true;
	}
	return succ;
}

double cMotion::GetFrameTime(int i) const
{
	return mFrames(i, eFrameTime);
}

void cMotion::PostProcessFrames(Eigen::MatrixXd& frames) const
{
	int num_frames = static_cast<int>(frames.rows());
	double curr_time = gMinTime;
	for (int f = 0; f < num_frames; ++f)
	{
		// auto& curr_frames = (Eigen::Matrix &) frames.row(f);
		// double duration = curr_frames(0, eFrameTime);
		double duration = frames.row(f)(0, eFrameTime);
		// curr_frames(0, eFrameTime) = curr_time;
		frames.row(f)(0, eFrameTime) = curr_time;
		curr_time += duration;
	}
}

double cMotion::GetDuration() const
{
	int num_frames = GetNumFrames();
	double max_time = mFrames(num_frames - 1, eFrameTime);
	return max_time;
}

int cMotion::CalcCycleCount(double time) const
{
	double dur = GetDuration();
	double phases = time / dur;
	int count = static_cast<int>(std::floor(phases));
	return count;
}

void cMotion::CalcIndexPhase(double time, int& out_idx, double& out_phase) const
{
	double max_time = GetDuration();

	if (!mLoop)
	{
		if (time <= gMinTime)
		{
			out_idx = 0;
			out_phase = 0;
			return;
		}
		else if (time >= max_time)
		{
			out_idx = GetNumFrames() - 2;
			out_phase = 1;
			return;
		}
	}

	time = std::fmod(time, max_time);
	if (time < 0)
	{
		time += max_time;
	}

	const Eigen::VectorXd& frame_times = mFrames.col(eFrameTime);
	auto it = std::upper_bound(frame_times.data(), frame_times.data() + frame_times.size(), time);
	out_idx = static_cast<int>(it - frame_times.data() - 1);

	double time0 = frame_times(out_idx);
	double time1 = frame_times(out_idx + 1);
	out_phase = (time - time0) / (time1 - time0);
}
