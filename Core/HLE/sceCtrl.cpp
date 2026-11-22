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

#include <algorithm>
#include <cmath>
#include <atomic>
#include <mutex>
#include <vector>

#include "Common/Serialize/Serializer.h"
#include "Common/Serialize/SerializeFuncs.h"
#include "Common/Math/math_util.h"
#include "Core/CoreTiming.h"
#include "Core/HLE/HLE.h"
#include "Core/HLE/ErrorCodes.h"
#include "Core/HLE/FunctionWrappers.h"
#include "Core/HLE/sceCtrl.h"
#include "Core/HLE/sceKernel.h"
#include "Core/HLE/sceKernelThread.h"
#include "Core/HLE/sceKernelInterrupt.h"
#include "Core/HW/Display.h"
#include "Core/System.h"
#include "Core/Config.h"
#include "Common/System/OSD.h"
#include "Core/ELF/ParamSFO.h"
#include "Core/MemMap.h"
#include "Core/MemMapHelpers.h"
#include "Core/MIPS/MIPS.h"
#include "Core/Replay.h"
#include "Core/Util/AudioFormat.h"  // for clamp_u8

/* Index for the two analog directions */
#define CTRL_ANALOG_X   0
#define CTRL_ANALOG_Y   1
#define CTRL_ANALOG_CENTER 128

#define CTRL_MODE_DIGITAL   0
#define CTRL_MODE_ANALOG    1

const u32 NUM_CTRL_BUFFERS = 64;

enum {
	CTRL_WAIT_POSITIVE = 1,
	CTRL_WAIT_NEGATIVE = 2,
};

struct CtrlData {
	u32_le frame;
	u32_le buttons;
	// The PSP has only one stick, but has space for more info.
	// The second stick is populated for HD remasters and possibly in the PSP emulator on PS3/Vita.
	u8 analog[2][2];
	u8 unused[4];
};

struct CtrlLatch {
	u32_le btnMake;
	u32_le btnBreak;
	u32_le btnPress;
	u32_le btnRelease;
};


//////////////////////////////////////////////////////////////////////////
// STATE BEGIN
static bool analogEnabled = false;
static int ctrlLatchBufs = 0;
static u32 ctrlOldButtons = 0;

static CtrlData ctrlBufs[NUM_CTRL_BUFFERS];
static CtrlData ctrlCurrent;
static u32 ctrlBuf = 0;
static u32 ctrlBufRead = 0;
static CtrlLatch latch;
static u32 dialogBtnMake = 0;

static int ctrlIdleReset = -1;
static int ctrlIdleBack = -1;

static int ctrlCycle = 0;

static std::vector<SceUID> waitingThreads;
static std::mutex ctrlMutex;

static int ctrlTimer = -1;

static u16 leftVibration = 0;
static u16 rightVibration = 0;
// The higher the dropout, the longer Vibration will run
static u8 vibrationLeftDropout = 160;
static u8 vibrationRightDropout = 160;

// STATE END
//////////////////////////////////////////////////////////////////////////

// Not savestated, this is emu state.
// Not related to sceCtrl*RapidFire(), although it may do the same thing.
static bool emuRapidFire = false;
static u32 emuRapidFireFrames = 0;
static bool emuRapidFireToggle = true;
static u32 emuRapidFireInterval = 5;

// Set while a system overlay (the HOME button's quit screen) owns the controls: the game then reads
// no buttons and centered sticks, as impose does on a PSP. Host-side peeks still see the real state.
static std::atomic<bool> gameInputBlocked{ false };

// Buttons pressed since the last sample. A press and release that both land between two samples
// would otherwise never be seen by the game. Guarded by ctrlMutex.
static u32 ctrlUnsampledPresses = 0;

// These buttons are not affected by rapid fire (neither is analog.)
const u32 CTRL_EMU_RAPIDFIRE_MASK = CTRL_UP | CTRL_DOWN | CTRL_LEFT | CTRL_RIGHT;

static void __CtrlUpdateLatch()
{
	std::lock_guard<std::mutex> guard(ctrlMutex);
	u64 t = CoreTiming::GetGlobalTimeUs();

	u32 buttons = ctrlCurrent.buttons | ctrlUnsampledPresses;
	ctrlUnsampledPresses = 0;
	if (emuRapidFire && emuRapidFireToggle)
		buttons &= CTRL_EMU_RAPIDFIRE_MASK;

	ReplayApplyCtrl(buttons, ctrlCurrent.analog, t);

	// Copy in the current data to the current buffer.
	ctrlBufs[ctrlBuf] = ctrlCurrent;

	if (PSP_CoreParameter().compat.flags().DaxterRotatedAnalogStick) {
		// For some reason, Daxter rotates the analog input. See #17015
		float angle = (15.0f / 360.0f) * (2.0f * M_PI);
		float cosAngle = cosf(angle);
		float sinAngle = sinf(angle);
		float x = ctrlBufs[ctrlBuf].analog[0][0] - 128;
		float y = ctrlBufs[ctrlBuf].analog[0][1] - 128;
		float rX = x * cosAngle - y * sinAngle;
		float rY = x * sinAngle + y * cosAngle;
		ctrlBufs[ctrlBuf].analog[0][0] = (u8)Clamp(rX + 128, 0.0f, 255.0f);
		ctrlBufs[ctrlBuf].analog[0][1] = (u8)Clamp(rY + 128, 0.0f, 255.0f);
	}

	if (gameInputBlocked) {
		buttons = 0;
		memset(ctrlBufs[ctrlBuf].analog, CTRL_ANALOG_CENTER, sizeof(ctrlBufs[ctrlBuf].analog));
	}

	ctrlBufs[ctrlBuf].buttons = buttons;

	u32 changed = buttons ^ ctrlOldButtons;
	latch.btnMake |= buttons & changed;
	latch.btnBreak |= ctrlOldButtons & changed;
	latch.btnPress |= buttons;
	latch.btnRelease |= ~buttons;
	dialogBtnMake |= buttons & changed;
	ctrlLatchBufs++;

	ctrlOldButtons = buttons;

	ctrlBufs[ctrlBuf].frame = (u32)t;
	if (!analogEnabled)
		memset(ctrlBufs[ctrlBuf].analog, CTRL_ANALOG_CENTER, sizeof(ctrlBufs[ctrlBuf].analog));

	ctrlBuf = (ctrlBuf + 1) % NUM_CTRL_BUFFERS;

	// If we wrapped around, push the read head forward.
	// TODO: Is this right?
	if (ctrlBufRead == ctrlBuf)
		ctrlBufRead = (ctrlBufRead + 1) % NUM_CTRL_BUFFERS;
}

static int __CtrlResetLatch()
{
	int oldBufs = ctrlLatchBufs;
	memset(&latch, 0, sizeof(CtrlLatch));
	ctrlLatchBufs = 0;
	return oldBufs;
}

void __CtrlSetGameInputBlocked(bool blocked) {
	gameInputBlocked = blocked;
}

bool __CtrlIsGameInputBlocked() {
	return gameInputBlocked;
}

u32 __CtrlPeekButtons()
{
	std::lock_guard<std::mutex> guard(ctrlMutex);

	return ctrlCurrent.buttons;
}

u32 __CtrlPeekButtonsVisual()
{
	u32 buttons;
	{
		std::lock_guard<std::mutex> guard(ctrlMutex);
		buttons = ctrlCurrent.buttons;
	}

	if (emuRapidFire && emuRapidFireToggle)
		buttons &= CTRL_EMU_RAPIDFIRE_MASK;
	return buttons;
}

void __CtrlPeekAnalog(int stick, float *x, float *y)
{
	std::lock_guard<std::mutex> guard(ctrlMutex);

	*x = (ctrlCurrent.analog[stick][CTRL_ANALOG_X] - 127.5f) / 127.5f;
	*y = -(ctrlCurrent.analog[stick][CTRL_ANALOG_Y] - 127.5f) / 127.5f;
}


u32 __CtrlReadLatch()
{
	u32 ret = dialogBtnMake;
	dialogBtnMake = 0;
	return ret;
}

// Presses held for a number of vblank samples (the debugger's input.buttons.press). Counted down on
// the emulator thread at each vblank, so how long a press lasts doesn't depend on how fast the
// emulator runs. Guarded by ctrlMutex; not saved.
struct TimedPress {
	int id;
	u32 buttons;
	int samplesLeft;
};
static std::vector<TimedPress> timedPresses;
static int nextTimedPressId = 1;

int __CtrlPressFor(u32 buttons, int vblanks) {
	std::lock_guard<std::mutex> guard(ctrlMutex);
	buttons &= CTRL_MASK_USER;
	ctrlCurrent.buttons |= buttons;
	const int id = nextTimedPressId++;
	timedPresses.push_back(TimedPress{ id, buttons, std::max(vblanks, 1) });
	return id;
}

bool __CtrlPressActive(int id) {
	std::lock_guard<std::mutex> guard(ctrlMutex);
	for (const TimedPress &press : timedPresses) {
		if (press.id == id) {
			return true;
		}
	}
	return false;
}

// After the vblank's sample, so a press of N vblanks is seen by N samples.
static void __CtrlUpdateTimedPresses() {
	std::lock_guard<std::mutex> guard(ctrlMutex);
	if (timedPresses.empty()) {
		return;
	}
	u32 released = 0;
	for (TimedPress &press : timedPresses) {
		if (--press.samplesLeft <= 0) {
			released |= press.buttons;
		}
	}
	timedPresses.erase(std::remove_if(timedPresses.begin(), timedPresses.end(), [](const TimedPress &press) {
		return press.samplesLeft <= 0;
	}), timedPresses.end());
	// Unless another press still holds them.
	for (const TimedPress &press : timedPresses) {
		released &= ~press.buttons;
	}
	ctrlCurrent.buttons &= ~released;
}

void __CtrlUpdateButtons(u32 bitsToSet, u32 bitsToClear)
{
	bitsToClear &= CTRL_MASK_USER;
	bitsToSet &= CTRL_MASK_USER;

	std::lock_guard<std::mutex> guard(ctrlMutex);
	// There's no atomic operation for this, so mutex it is.
	ctrlUnsampledPresses |= bitsToSet & ~ctrlCurrent.buttons;
	ctrlCurrent.buttons =(ctrlCurrent.buttons & ~bitsToClear) | bitsToSet;
}

void __CtrlSetAnalogXY(int stick, float x, float y)
{
	u8 scaledX = clamp_u8((int)ceilf(x * 127.5f + 127.5f));
	// TODO: We might have too many negations of Y...
	u8 scaledY = clamp_u8((int)ceilf(-y * 127.5f + 127.5f));

	std::lock_guard<std::mutex> guard(ctrlMutex);
	ctrlCurrent.analog[stick][CTRL_ANALOG_X] = scaledX;
	ctrlCurrent.analog[stick][CTRL_ANALOG_Y] = scaledY;
}

// not making XY to use these due to mutex guard usage
void __CtrlSetAnalogX(int stick, float x) {
	u8 scaledX = clamp_u8((int)ceilf(x * 127.5f + 127.5f));
	std::lock_guard<std::mutex> guard(ctrlMutex);
	ctrlCurrent.analog[stick][CTRL_ANALOG_X] = scaledX;
}

void __CtrlSetAnalogY(int stick, float y) {
	u8 scaledY = clamp_u8((int)ceilf(-y * 127.5f + 127.5f));
	std::lock_guard<std::mutex> guard(ctrlMutex);
	ctrlCurrent.analog[stick][CTRL_ANALOG_Y] = scaledY;
}

void __CtrlSetRapidFire(bool state, int interval) {
	emuRapidFire = state;
	emuRapidFireToggle = true;
	emuRapidFireInterval = interval;
}

bool __CtrlGetRapidFire()
{
	return emuRapidFire;
}

static int __CtrlReadSingleBuffer(PSPPointer<CtrlData> data, bool negative)
{
	if (data.IsValid())
	{
		*data = ctrlBufs[ctrlBufRead];
		ctrlBufRead = (ctrlBufRead + 1) % NUM_CTRL_BUFFERS;

		// Mask out buttons games aren't allowed to see.
		data->buttons &= CTRL_MASK_USER;
		if (negative)
			data->buttons = ~data->buttons;

		return 1;
	}

	return 0;
}

static int __CtrlReadBuffer(u32 ctrlDataPtr, u32 nBufs, bool negative, bool peek)
{
	if (nBufs > NUM_CTRL_BUFFERS)
		return SCE_KERNEL_ERROR_INVALID_SIZE;

	if (!peek && !__KernelIsDispatchEnabled())
		return SCE_KERNEL_ERROR_CAN_NOT_WAIT;
	if (!peek && __IsInInterrupt())
		return SCE_KERNEL_ERROR_ILLEGAL_CONTEXT;

	u32 resetRead = ctrlBufRead;

	u32 availBufs;
	// Peeks always work, they just go go from now X buffers.
	if (peek)
		availBufs = nBufs;
	else
	{
		availBufs = (ctrlBuf - ctrlBufRead + NUM_CTRL_BUFFERS) % NUM_CTRL_BUFFERS;
		if (availBufs > nBufs)
			availBufs = nBufs;
	}
	ctrlBufRead = (ctrlBuf - availBufs + NUM_CTRL_BUFFERS) % NUM_CTRL_BUFFERS;

	int done = 0;
	auto data = PSPPointer<CtrlData>::Create(ctrlDataPtr);
	for (u32 i = 0; i < availBufs; ++i)
		done += __CtrlReadSingleBuffer(data++, negative);

	if (peek)
		ctrlBufRead = resetRead;

	return done;
}

static void __CtrlDoSample()
{
	// This samples the ctrl data into the buffers and updates the latch.
	__CtrlUpdateLatch();

	// Wake up a single thread that was waiting for the buffer.
retry:
	if (!waitingThreads.empty() && ctrlBuf != ctrlBufRead)
	{
		SceUID threadID = waitingThreads[0];
		waitingThreads.erase(waitingThreads.begin());

		u32 error;
		SceUID wVal = __KernelGetWaitID(threadID, WAITTYPE_CTRL, error);
		// Make sure it didn't get woken or something.
		if (wVal == 0)
			goto retry;

		PSPPointer<CtrlData> ctrlDataPtr;
		ctrlDataPtr = __KernelGetWaitValue(threadID, error);
		int retVal = __CtrlReadSingleBuffer(ctrlDataPtr, wVal == CTRL_WAIT_NEGATIVE);
		__KernelResumeThreadFromWait(threadID, retVal);
		__KernelReSchedule("ctrl buffers updated");
	}
}


// Star Wars Battlefront: Elite Squadron (ULUS10390) explosion rumble.
// A small game-side hook (see the cheat file) stores, at kExplosionStrengthAddr, the strongest
// camera-rumble strength (amplitude * distance falloff * envelope) it saw this frame. We consume it
// here (zero it after reading), normalize it against the strongest value seen so far, and turn it
// into a short pulse pattern on the controller motors.
static void UpdateStarWarsEliteSquadronRumble() {
	static const u32 kExplosionStrengthAddr = 0x08803140;
	static const u32 kPackEventAddr = 0x08803150;
	// Tunables, written by the cheat file (a zero word/half means "use the default"):
	//   +0x00 explosion  [L u16][R u16] motor strength 0..FFFF   +0x04 [frames/step u16][min level % u16]
	//   +0x08 jump pack  [L u16][R u16]                          +0x0C [frames/step u16][unused]
	//   +0x10 jet pack   [L u16][R u16]                          +0x14 [frames/step u16][unused]
	//   +0x18 debug on-screen messages (non-zero = on)
	//   +0x1C ship-fire button mask (PSP button bits; 0 = default R+Cross)   +0x20 ship fire [L u16][R u16] strength
	static const u32 kParams = 0x08803180;
	static bool isGame = false;
	static std::string lastId;
	const std::string id = g_paramSFO.GetValueString("DISC_ID");
	if (id != lastId) {
		lastId = id;
		isGame = (id == "ULUS10390");
	}
	if (!isGame || !Memory::IsValidRange(kParams, 0x30))
		return;

	auto param16 = [&](u32 off, int hi) -> int { return Memory::ReadUnchecked_U16(kParams + off + (hi ? 2 : 0)); };
	const bool debug = Memory::ReadUnchecked_U32(kParams + 0x18) != 0;

	auto apply = [&](float level, float lMul, float rMul, int percent) {
		level = std::clamp(level * percent / 100.0f, 0.0f, 1.0f);
		const u16 l = (u16)(std::clamp(level * lMul, 0.0f, 1.0f) * 65535.0f);
		const u16 r = (u16)(std::clamp(level * rMul, 0.0f, 1.0f) * 65535.0f);
		// The Android side samples once per UI frame, so the value must persist a few frames (slow fade).
		if (l > leftVibration) { leftVibration = l; vibrationLeftDropout = 192; }
		if (r > rightVibration) { rightVibration = r; vibrationRightDropout = 192; }
	};

	// One-shot pack events: the cheat's hook stores 1 (jump pack) or 2 (jet pack) at kPackEventAddr.
	struct PackPattern { int steps; int stepFrames; float level[5]; float left; float right; };
	static const PackPattern kPatterns[2] = {
		{ 3, 3, { 1.0f, 0.5f, 0.25f }, 0.40f, 0.90f },             // jump pack: quick snap
		{ 5, 4, { 0.7f, 0.4f, 0.6f, 0.4f, 0.2f }, 0.80f, 0.45f },   // jet pack: longer rumble
	};
	static int packKind = -1, packFrame = 0;
	const u32 ev = Memory::ReadUnchecked_U32(kPackEventAddr);
	if (ev == 1 || ev == 2) {
		Memory::WriteUnchecked_U32(0, kPackEventAddr);
		packKind = (int)ev - 1;
		packFrame = 0;
		if (debug)
			g_OSD.Show(OSDType::MESSAGE_INFO, packKind == 0 ? "Jump pack rumble" : "Jet pack rumble", 1.5f, "sw_rumble_pack");
	}
	if (packKind >= 0) {
		const PackPattern &pt = kPatterns[packKind];
		const u32 base = packKind == 0 ? 0x08 : 0x10;
		const int pl = param16(base, 0), pr = param16(base, 1);
		const int pf = param16(base + 4, 0);
		const int frames = pf > 0 ? std::min(pf, 60) : pt.stepFrames;
		const int step = packFrame / frames;
		if (step >= pt.steps) {
			packKind = -1;
		} else {
			apply(pt.level[step],
				pl > 0 ? pl / 65535.0f : pt.left,
				pr > 0 ? pr / 65535.0f : pt.right,
				packKind == 0 ? g_Config.iRumbleJumppack : g_Config.iRumbleJetpack);
			packFrame++;
		}
	}

	// Vehicle weapon fire: the cheat hook flags that *some* vehicle fired a weapon (AI included), so we only
	// vibrate while the player is holding a fire button. Event word: kShipFireAddr.
	static const u32 kShipFireAddr = 0x08803154;
	static int shipFrame = -1;
	if (Memory::ReadUnchecked_U32(kShipFireAddr) != 0) {
		Memory::WriteUnchecked_U32(0, kShipFireAddr);
		u32 mask = Memory::ReadUnchecked_U32(kParams + 0x1C);
		if (mask == 0)
			mask = CTRL_RTRIGGER | CTRL_CROSS;
		if (__CtrlPeekButtons() & mask)
			shipFrame = 0;
	}
	if (shipFrame >= 0) {
		const int sl = param16(0x20, 0), sr = param16(0x20, 1);
		if (shipFrame < 2) {
			apply(1.0f, sl > 0 ? sl / 65535.0f : 0.25f, sr > 0 ? sr / 65535.0f : 0.5f, g_Config.iRumbleShip);
			shipFrame++;
		} else {
			shipFrame = -1;
		}
	}

	// Diagnostics (only with the debug word set): is the explosion hook installed?
	static int diagTimer = 0;
	static int hookState = -1;
	if (debug && ++diagTimer >= 300) {
		diagTimer = 0;
		const int st = Memory::ReadUnchecked_U32(0x089CD058) == 0x0E200C40 ? 1 : 0;
		if (st != hookState) {
			hookState = st;
			g_OSD.Show(OSDType::MESSAGE_INFO, st ? "Explosion rumble hook: ACTIVE" : "Explosion rumble hook: NOT installed (enable the cheat)", 4.0f, "sw_rumble_hook");
		}
	}

	// Explosions / ship boosts / anything going through the game's camera-rumble system.
	static float peak = 0.0f;       // strongest strength seen (slowly decays so a lone big blast doesn't dominate forever)
	static int frame = -1;          // frames since the current event started, -1 = idle
	static int quiet = 0;           // consecutive frames without a signal
	const float s = Memory::ReadUnchecked_Float(kExplosionStrengthAddr);
	Memory::WriteUnchecked_U32(0, kExplosionStrengthAddr);

	if (!(s > 0.0f) || !std::isfinite(s)) {
		if (++quiet > 6)
			frame = -1;
		return;
	}
	quiet = 0;
	peak = std::max(peak * 0.9995f, s);
	if (frame < 0) {
		frame = 0;
		if (debug) {
			char msg[96];
			snprintf(msg, sizeof(msg), "Rumble event: strength %.5f (peak %.5f)", s, peak);
			g_OSD.Show(OSDType::MESSAGE_INFO, msg, 2.0f, "sw_rumble_evt");
		}
	} else {
		frame++;
	}

	const int el = param16(0x00, 0), er = param16(0x00, 1);
	const int ef = param16(0x04, 0), emin = param16(0x04, 1);
	const int stepFrames = ef > 0 ? std::min(ef, 60) : 4;
	const float floorLevel = (emin > 0 ? std::min(emin, 100) : 12) / 100.0f;

	// Relative pulse shape (30,10,50,30 -> normalized), then follow the game's decay.
	static const float shape[4] = { 0.6f, 0.2f, 1.0f, 0.6f };
	const float f = frame < stepFrames * 4 ? shape[frame / stepFrames] : 0.6f;
	// Compress the range so weak events (a ship boost next to a big blast) are still felt:
	// strength ratio^0.4, with a minimum level.
	const float ratio = std::min(1.0f, s / std::max(peak, 0.0001f));
	const float level = std::max(std::pow(ratio, 0.4f), floorLevel) * f;
	apply(level, el > 0 ? el / 65535.0f : 1.0f, er > 0 ? er / 65535.0f : 0.6f, g_Config.iRumbleExplosion);
}

void __CtrlVblank() {
	emuRapidFireFrames++;
	if (emuRapidFireFrames >= emuRapidFireInterval) {
		emuRapidFireFrames = 0;
		emuRapidFireToggle = !emuRapidFireToggle;
	}

	// Reduce gamepad Vibration by set % each frame
	leftVibration *= (float)vibrationLeftDropout / 256.0f;
	rightVibration *= (float)vibrationRightDropout / 256.0f;

	UpdateStarWarsEliteSquadronRumble();

	// This always runs, so make sure we're in vblank mode.
	if (ctrlCycle == 0)
		__CtrlDoSample();
	__CtrlUpdateTimedPresses();
}

static void __CtrlTimerUpdate(u64 userdata, int cyclesLate)
{
	// This only runs in timer mode (ctrlCycle > 0.)
	_dbg_assert_msg_(ctrlCycle > 0, "Ctrl: sampling cycle should be > 0");

	CoreTiming::ScheduleEvent(usToCycles(ctrlCycle) - cyclesLate, ctrlTimer, 0);

	__CtrlDoSample();
}

void __CtrlInit() {
	ctrlTimer = CoreTiming::RegisterEvent("CtrlSampleTimer", __CtrlTimerUpdate);

	ctrlIdleReset = -1;
	ctrlIdleBack = -1;
	ctrlCycle = 0;

	std::lock_guard<std::mutex> guard(ctrlMutex);

	ctrlBuf = 1;
	ctrlBufRead = 0;
	ctrlOldButtons = 0;
	ctrlLatchBufs = 0;
	dialogBtnMake = 0;

	memset(&latch, 0, sizeof(latch));
	// Start with everything released.
	latch.btnRelease = 0xffffffff;

	gameInputBlocked = false;
	ctrlUnsampledPresses = 0;
	memset(&ctrlCurrent, 0, sizeof(ctrlCurrent));
	memset(ctrlCurrent.analog, CTRL_ANALOG_CENTER, sizeof(ctrlCurrent.analog));
	analogEnabled = false;

	for (u32 i = 0; i < NUM_CTRL_BUFFERS; i++)
		memcpy(&ctrlBufs[i], &ctrlCurrent, sizeof(CtrlData));
}

void __CtrlDoState(PointerWrap &p)
{
	std::lock_guard<std::mutex> guard(ctrlMutex);

	auto s = p.Section("sceCtrl", 1, 3);
	if (!s)
		return;

	// Host input from before the load shouldn't show up in the loaded state.
	ctrlUnsampledPresses = 0;

	Do(p, analogEnabled);
	Do(p, ctrlLatchBufs);
	Do(p, ctrlOldButtons);

	p.DoVoid(ctrlBufs, sizeof(ctrlBufs));
	if (s <= 2) {
		CtrlData dummy = {0};
		Do(p, dummy);
	}
	Do(p, ctrlBuf);
	Do(p, ctrlBufRead);
	if (ctrlBuf >= NUM_CTRL_BUFFERS || ctrlBufRead >= NUM_CTRL_BUFFERS) {
		ERROR_LOG(Log::sceCtrl, "Savestate failure: invalid ctrl buffer index");
		p.SetError(p.ERROR_FAILURE);
		ctrlBuf = 0;
		ctrlBufRead = 0;
		return;
	}
	Do(p, latch);
	if (s == 1) {
		dialogBtnMake = 0;
	} else {
		Do(p, dialogBtnMake);
	}

	Do(p, ctrlIdleReset);
	Do(p, ctrlIdleBack);

	Do(p, ctrlCycle);

	SceUID dv = 0;
	Do(p, waitingThreads, dv);

	Do(p, ctrlTimer);
	CoreTiming::RestoreRegisterEvent(ctrlTimer, "CtrlSampleTimer", __CtrlTimerUpdate);
}

void __CtrlShutdown()
{
	waitingThreads.clear();
	std::lock_guard<std::mutex> guard(ctrlMutex);
	timedPresses.clear();
}

static u32 sceCtrlSetSamplingCycle(u32 cycle)
{
	if ((cycle > 0 && cycle < 5555) || cycle > 20000) {
		return hleLogWarning(Log::sceCtrl, SCE_KERNEL_ERROR_INVALID_VALUE);
	}

	u32 prev = ctrlCycle;
	ctrlCycle = cycle;

	if (prev > 0)
		CoreTiming::UnscheduleEvent(ctrlTimer, 0);
	if (cycle > 0)
		CoreTiming::ScheduleEvent(usToCycles(ctrlCycle), ctrlTimer, 0);

	return hleLogDebug(Log::sceCtrl, prev);
}

static int sceCtrlGetSamplingCycle(u32 cyclePtr)
{
	if (Memory::IsValidAddress(cyclePtr))
		Memory::WriteUnchecked_U32(ctrlCycle, cyclePtr);
	return hleLogDebug(Log::sceCtrl, 0);
}

static u32 sceCtrlSetSamplingMode(u32 mode)
{
	u32 retVal = 0;
	if (mode > 1)
		return hleLogError(Log::sceCtrl, SCE_KERNEL_ERROR_INVALID_MODE);

	retVal = analogEnabled == true ? CTRL_MODE_ANALOG : CTRL_MODE_DIGITAL;
	analogEnabled = mode == CTRL_MODE_ANALOG ? true : false;
	return hleLogDebug(Log::sceCtrl, retVal);
}

static int sceCtrlGetSamplingMode(u32 modePtr)
{
	u32 retVal = analogEnabled == true ? CTRL_MODE_ANALOG : CTRL_MODE_DIGITAL;

	if (Memory::IsValidAddress(modePtr))
		Memory::WriteUnchecked_U32(retVal, modePtr);

	return hleLogDebug(Log::sceCtrl, 0);
}

// Hides buttons from user mode readers, for kernel code that takes them over (Daedalus, for the
// system buttons, which games don't get here anyway).
static int sceCtrlSetButtonIntercept(u32 buttons, int intercept) {
	return hleLogDebug(Log::sceCtrl, 0);
}

static int sceCtrlSetIdleCancelThreshold(int idleReset, int idleBack)
{
	if (idleReset < -1 || idleBack < -1 || idleReset > 128 || idleBack > 128)
		return hleLogError(Log::sceCtrl, SCE_KERNEL_ERROR_INVALID_VALUE);

	ctrlIdleReset = idleReset;
	ctrlIdleBack = idleBack;
	return hleLogDebug(Log::sceCtrl, 0);
}

static int sceCtrlGetIdleCancelThreshold(u32 idleResetPtr, u32 idleBackPtr)
{
	if (idleResetPtr && !Memory::IsValid4AlignedAddress(idleResetPtr))
		return hleLogError(Log::sceCtrl, SCE_KERNEL_ERROR_PRIV_REQUIRED);
	if (idleBackPtr && !Memory::IsValid4AlignedAddress(idleBackPtr))
		return hleLogError(Log::sceCtrl, SCE_KERNEL_ERROR_PRIV_REQUIRED);

	if (idleResetPtr)
		Memory::WriteUnchecked_U32(ctrlIdleReset, idleResetPtr);
	if (idleBackPtr)
		Memory::WriteUnchecked_U32(ctrlIdleBack, idleBackPtr);

	return hleLogDebug(Log::sceCtrl, 0);
}

int sceCtrlReadBufferPositive(u32 ctrlDataPtr, u32 nBufs)
{
	int done = __CtrlReadBuffer(ctrlDataPtr, nBufs, false, false);
	hleEatCycles(330);
	if (done != 0) {
		return hleLogDebug(Log::sceCtrl, done);
	} else {
		waitingThreads.push_back(__KernelGetCurThread());
		__KernelWaitCurThread(WAITTYPE_CTRL, CTRL_WAIT_POSITIVE, ctrlDataPtr, 0, false, "ctrl buffer waited");
		return hleLogDebug(Log::sceCtrl, done, "waiting");
	}
}

static int sceCtrlReadBufferNegative(u32 ctrlDataPtr, u32 nBufs)
{
	int done = __CtrlReadBuffer(ctrlDataPtr, nBufs, true, false);
	hleEatCycles(330);
	if (done != 0) {
		return hleLogDebug(Log::sceCtrl, done);
	}
	else
	{
		waitingThreads.push_back(__KernelGetCurThread());
		__KernelWaitCurThread(WAITTYPE_CTRL, CTRL_WAIT_NEGATIVE, ctrlDataPtr, 0, false, "ctrl buffer waited");
		return hleLogDebug(Log::sceCtrl, done, "waiting");
	}
}

static int sceCtrlPeekBufferPositive(u32 ctrlDataPtr, u32 nBufs)
{
	int done = __CtrlReadBuffer(ctrlDataPtr, nBufs, false, true);
	// ~0.2us more for each sample copied: 15us for 64 on hardware.
	hleEatCycles(330 + 45 * std::max(done, 0));
	// Some homebrew call this in a tight loop - so VERBOSE it is.
	return hleLogVerbose(Log::sceCtrl, done);
}

static int sceCtrlPeekBufferNegative(u32 ctrlDataPtr, u32 nBufs)
{
	int done = __CtrlReadBuffer(ctrlDataPtr, nBufs, true, true);
	// ~0.2us more for each sample copied: 15us for 64 on hardware.
	hleEatCycles(330 + 45 * std::max(done, 0));
	// Some homebrew call this in a tight loop - so VERBOSE it is.
	return hleLogVerbose(Log::sceCtrl, done);
}

static void __CtrlWriteUserLatch(CtrlLatch *userLatch, int bufs) {
	*userLatch = latch;
	userLatch->btnBreak &= CTRL_MASK_USER;
	userLatch->btnMake &= CTRL_MASK_USER;
	userLatch->btnPress &= CTRL_MASK_USER;
	if (bufs > 0) {
		userLatch->btnRelease |= ~CTRL_MASK_USER;
	}
}

static u32 sceCtrlPeekLatch(u32 latchDataPtr) {
	auto userLatch = PSPPointer<CtrlLatch>::Create(latchDataPtr);
	if (userLatch.IsValid()) {
		__CtrlWriteUserLatch(userLatch, ctrlLatchBufs);
	}
	return hleLogDebug(Log::sceCtrl, ctrlLatchBufs);
}

static u32 sceCtrlReadLatch(u32 latchDataPtr) {
	auto userLatch = PSPPointer<CtrlLatch>::Create(latchDataPtr);
	if (userLatch.IsValid()) {
		__CtrlWriteUserLatch(userLatch, ctrlLatchBufs);
	}
	return hleLogDebug(Log::sceCtrl, __CtrlResetLatch());
}

static const HLEFunction sceCtrl[] =
{
	{0X3E65A0EA, nullptr,                                  "sceCtrlInit",                      '?', ""  }, //(int unknown), init with 0
	{0X1F4011E6, &WrapU_U<sceCtrlSetSamplingMode>,         "sceCtrlSetSamplingMode",           'x', "x" },
	{0X6A2774F3, &WrapU_U<sceCtrlSetSamplingCycle>,        "sceCtrlSetSamplingCycle",          'x', "x" },
	{0X02BAAD91, &WrapI_U<sceCtrlGetSamplingCycle>,        "sceCtrlGetSamplingCycle",          'i', "x" },
	{0XDA6B76A1, &WrapI_U<sceCtrlGetSamplingMode>,         "sceCtrlGetSamplingMode",           'i', "x" },
	{0X1F803938, &WrapI_UU<sceCtrlReadBufferPositive>,     "sceCtrlReadBufferPositive",        'i', "xx"},
	{0X3A622550, &WrapI_UU<sceCtrlPeekBufferPositive>,     "sceCtrlPeekBufferPositive",        'i', "xx"},
	{0XC152080A, &WrapI_UU<sceCtrlPeekBufferNegative>,     "sceCtrlPeekBufferNegative",        'i', "xx"},
	{0X60B81F86, &WrapI_UU<sceCtrlReadBufferNegative>,     "sceCtrlReadBufferNegative",        'i', "xx"},
	{0XB1D0E5CD, &WrapU_U<sceCtrlPeekLatch>,               "sceCtrlPeekLatch",                 'i', "x" },
	{0X0B588501, &WrapU_U<sceCtrlReadLatch>,               "sceCtrlReadLatch",                 'i', "x" },
	{0X348D99D4, nullptr,                                  "sceCtrlSetSuspendingExtraSamples", '?', ""  },
	{0XAF5960F3, nullptr,                                  "sceCtrlGetSuspendingExtraSamples", '?', ""  },
	{0XA68FD260, nullptr,                                  "sceCtrlClearRapidFire",            '?', ""  },
	{0X6841BE1A, nullptr,                                  "sceCtrlSetRapidFire",              '?', ""  },
	{0XA7144800, &WrapI_II<sceCtrlSetIdleCancelThreshold>, "sceCtrlSetIdleCancelThreshold",    'i', "ii"},
	{0X687660FA, &WrapI_UU<sceCtrlGetIdleCancelThreshold>, "sceCtrlGetIdleCancelThreshold",    'i', "xx"},
	// The "2" variants take a port first: (int port, data, int count).
	{0X5A36B1C2, nullptr,                                  "sceCtrlPeekBufferPositive2",       '?', ""  },
	{0X239A6BA7, nullptr,                                  "sceCtrlPeekBufferNegative2",       '?', ""  },
	{0X1098030B, nullptr,                                  "sceCtrlReadBufferPositive2",       '?', ""  },
	{0X7C3675AB, nullptr,                                  "sceCtrlReadBufferNegative2",       '?', ""  },
	// sceCtrl_driver's NID for sceCtrlPeekBufferPositive in 3.71. Imported by NJEMU's SystemButtons.prx
	// kernel plugin, which picks it when sceKernelDevkitVersion() is 3.71 or newer (#8925).
	{0XC4AAD55F, &WrapI_UU<sceCtrlPeekBufferPositive>,     "sceCtrlPeekBufferPositive",        'i', "xx"},
	// sceCtrl_driver NID for sceCtrlReadBufferPositive (firmware 3.71+, per JPCSP).
	// Imported by translation-patch loaders like CLANNAD's chfont.prx.
	{0X454455AC, &WrapI_UU<sceCtrlReadBufferPositive>,     "sceCtrlReadBufferPositive",        'i', "xx"},
	// Driver only.
	{0X7CA723DC, &WrapI_UI<sceCtrlSetButtonIntercept>,     "sceCtrlSetButtonIntercept",        'i', "xi"},
	// sceCtrl_driver NIDs in 3.71-3.73's ctrl.prx (at the same addresses as the user exports), which
	// LightMP3's support.prx imports.
	{0X28E71A16, &WrapU_U<sceCtrlSetSamplingMode>,         "sceCtrlSetSamplingMode",           'x', "x" },
	{0X6D74BF08, &WrapU_U<sceCtrlSetSamplingCycle>,        "sceCtrlSetSamplingCycle",          'x', "x" },
};

void Register_sceCtrl()
{
	RegisterHLEModule("sceCtrl", ARRAY_SIZE(sceCtrl), sceCtrl);
}

void Register_sceCtrl_driver()
{
	RegisterHLEModule("sceCtrl_driver", ARRAY_SIZE(sceCtrl), sceCtrl);
}

u16 sceCtrlGetRightVibration() {
	return hleNoLog(rightVibration);
}

u16 sceCtrlGetLeftVibration() {
	return hleNoLog(leftVibration);
}

namespace SceCtrl {
	void SetRightVibration(u16 rVibration) {
		rightVibration = rVibration;
	}
	void SetLeftVibration(u16 lVibration) {
		leftVibration = lVibration;
	}
	void SetVibrationRightDropout(u8 vibrationRDropout) {
		vibrationRightDropout = vibrationRDropout;
	}
	void SetVibrationLeftDropout(u8 vibrationLDropout) {
		vibrationLeftDropout = vibrationLDropout;
	}
}
