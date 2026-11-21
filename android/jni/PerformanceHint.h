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

#pragma once

#include <cstdint>

struct APerformanceHintSession;

// An ADPF performance hint session (Android 13+) for one thread.
// Emulation does a burst of work each frame and then waits, so the scheduler sees a mostly idle
// thread and moves it to a slow core at low clocks, where the next burst then runs. Reporting each
// frame's work against a target tells it what we actually need. Does nothing where unsupported.
//
// With OpenGL the work is split over two threads (EmuThread runs the emulation, RenderThread executes
// the GL commands it queues), and each needs its own session or the unhinted one still gets parked
// on a slow core. Use one instance per thread.
class PerformanceHintSession {
public:
	// name is only for logging. If minReportIntervalNs > 0, ReportFrame can be called as often as
	// you like (for example once per queued task) and only reports once that much wall time has
	// passed, with the CPU time used in between. Use it for threads that don't have one iteration per frame.
	explicit PerformanceHintSession(const char *name = "thread", int64_t minReportIntervalNs = 0);
	~PerformanceHintSession();

	// Call from the same thread every time. The session is created on the first call.
	void ReportFrame();

	// True once a session has actually been created. Handy for diagnostics.
	bool Active() const { return session_ != nullptr; }

private:
	const char *name_;
	int64_t minIntervalNs_;
	APerformanceHintSession *session_ = nullptr;
	bool failed_ = false;
	int64_t lastCpuTimeNs_ = -1;
	int64_t lastReportWallNs_ = -1;
};
