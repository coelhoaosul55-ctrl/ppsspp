// Copyright (c) 2012- PPSSPP Project.

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, version 2.0 or later versions.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License 2.0 for more details.

// A copy of the GPL 2.0 should have been included with the program.
// If not, see http://www.gnu.org/licenses/

// Official git repository and contact information can be found at
// https://github.com/hrydgard/ppsspp and http://www.ppsspp.org/.

#include <dlfcn.h>
#include <time.h>
#include <unistd.h>

#include "Common/Log.h"
#include "android/jni/PerformanceHint.h"

struct APerformanceHintManager;

namespace {

// Half a frame. With CPU time as the measure, this leaves the other half as headroom, so the
// scheduler keeps us on a core fast enough to finish well before the frame is due.
constexpr int64_t TARGET_WORK_NS = 1000000000LL / 60 / 2;

// Looked up at runtime, since we build against API 21 and these are API 33.
struct HintApi {
	APerformanceHintManager *(*getManager)();
	APerformanceHintSession *(*createSession)(APerformanceHintManager *, const int32_t *, size_t, int64_t);
	int (*reportActualWorkDuration)(APerformanceHintSession *, int64_t);
	void (*closeSession)(APerformanceHintSession *);
	// API 35 (Android 15). Optional: without it we just keep the default.
	int (*setPreferPowerEfficiency)(APerformanceHintSession *, bool);

	bool Load() {
		void *lib = dlopen("libandroid.so", RTLD_NOW);
		if (!lib) {
			return false;
		}
		getManager = (decltype(getManager))dlsym(lib, "APerformanceHint_getManager");
		createSession = (decltype(createSession))dlsym(lib, "APerformanceHint_createSession");
		reportActualWorkDuration = (decltype(reportActualWorkDuration))dlsym(lib, "APerformanceHint_reportActualWorkDuration");
		closeSession = (decltype(closeSession))dlsym(lib, "APerformanceHint_closeSession");
		setPreferPowerEfficiency = (decltype(setPreferPowerEfficiency))dlsym(lib, "APerformanceHint_setPreferPowerEfficiency");
		return getManager && createSession && reportActualWorkDuration && closeSession;
	}
};

const HintApi *GetApi() {
	static HintApi api{};
	static const bool loaded = api.Load();
	return loaded ? &api : nullptr;
}

// Time spent running on a CPU, so sleeping in the frame limiter or blocking on vsync or the GPU
// doesn't count as work.
int64_t ThreadCpuTimeNs() {
	timespec ts;
	if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) != 0) {
		return -1;
	}
	return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

int64_t MonotonicNs() {
	timespec ts;
	if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
		return -1;
	}
	return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

}  // namespace

PerformanceHintSession::PerformanceHintSession(const char *name, int64_t minReportIntervalNs)
	: name_(name), minIntervalNs_(minReportIntervalNs) {}

PerformanceHintSession::~PerformanceHintSession() {
	if (session_) {
		GetApi()->closeSession(session_);
	}
}

void PerformanceHintSession::ReportFrame() {
	if (failed_) {
		return;
	}
	if (!session_) {
		const HintApi *api = GetApi();
		APerformanceHintManager *manager = api ? api->getManager() : nullptr;
		const int32_t tid = gettid();
		session_ = manager ? api->createSession(manager, &tid, 1, TARGET_WORK_NS) : nullptr;
		if (!session_) {
			INFO_LOG(Log::System, "Performance hint session not available for %s (thread %d)", name_, tid);
			failed_ = true;
			return;
		}
		// Android 15+: tell the system we want speed here, not battery. Without this, some vendors'
		// power HALs still favour efficiency for a thread that is mostly idle.
		int preferResult = -1;
		if (api->setPreferPowerEfficiency) {
			preferResult = api->setPreferPowerEfficiency(session_, false);
		}
		INFO_LOG(Log::System, "Performance hint session created for %s (thread %d), target %d us, preferPowerEfficiency(false)=%s",
			name_, tid, (int)(TARGET_WORK_NS / 1000), api->setPreferPowerEfficiency ? (preferResult == 0 ? "ok" : "failed") : "n/a");
	}
	const int64_t cpuNow = ThreadCpuTimeNs();
	if (minIntervalNs_ > 0) {
		// Batch up: only report once per interval, with all the CPU time used since the last report.
		const int64_t wallNow = MonotonicNs();
		if (lastReportWallNs_ < 0) {
			lastReportWallNs_ = wallNow;
			lastCpuTimeNs_ = cpuNow;
			return;
		}
		if (wallNow - lastReportWallNs_ < minIntervalNs_) {
			return;
		}
		lastReportWallNs_ = wallNow;
	}
	if (lastCpuTimeNs_ >= 0 && cpuNow > lastCpuTimeNs_) {
		GetApi()->reportActualWorkDuration(session_, cpuNow - lastCpuTimeNs_);
	}
	lastCpuTimeNs_ = cpuNow;
}
