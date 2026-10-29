/* Copyright (c) 2026 NewDevNow2026
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
/* Cartucho network link cable: GBA multiplayer (SIO_MULTI) over TCP on a LAN.
 *
 * One peer hosts (player 1, link master); up to three peers join. Transfers
 * are not lockstepped: the master starts a transfer, asks the children for
 * the value currently latched in their SIOMLT_SEND register, and completes
 * the transfer when every answer arrived (or after a timeout). To the games
 * it looks like a link cable with a few milliseconds of latency.
 */
#ifndef CARTUCHO_NETLINK_H
#define CARTUCHO_NETLINK_H

#include <mgba-util/common.h>
#include <mgba/gba/interface.h>
#include <mgba/core/timing.h>
#include <mgba/core/interface.h>
#include <pthread.h>

CXX_GUARD_START

#define NETLINK_MAX_PLAYERS 4
#define NETLINK_PROTOCOL_VERSION 1
#define NETLINK_DEFAULT_PORT 29730
#define NETLINK_FAST_MAX 32

struct NetLinkMsg {
	uint8_t type;
	uint8_t a;
	uint16_t b;
	uint32_t seq;
	uint16_t v[4];
};

struct NetLinkPeer {
	int fd;
	bool connected;
	bool multi;
	size_t rxLen;
	uint8_t rx[sizeof(struct NetLinkMsg)];
};

struct GBASIONetLink {
	struct GBASIODriver d;

	/* Configuration */
	bool host;
	char address[64];
	uint16_t port;

	/* Network thread */
	pthread_t thread;
	bool threadRunning;
	volatile bool running;
	int listenFd;
	struct NetLinkPeer peers[NETLINK_MAX_PLAYERS]; /* host: peers[1..3]; client: peers[0] is the host */
	pthread_mutex_t lock;
	pthread_mutex_t sendLock;
	pthread_cond_t respCond;   /* signalled when a transfer answer (or a disconnect) arrives */
	volatile bool stopping;

	/* Shared state (guarded by lock) */
	int id;                 /* our player id, 0 = master */
	int attached;           /* number of connected players including us */
	uint8_t multiMask;      /* players currently in multiplayer mode */
	bool selfMulti;
	bool connected;         /* client: connected to a host */
	uint16_t sendLatch;     /* child: last value written to SIOMLT_SEND */

	/* Inbound event queue (network thread -> core thread) */
	struct NetLinkMsg queue[64];
	int qHead;
	int qTail;

	/* Child: request waiting for the game to prepare its SIOMLT_SEND (core thread) */
	bool reqPending;
	uint32_t reqSeq;
	int32_t lastDoneAt;      /* core time when the last transfer result was applied */
	bool haveDone;
	bool sendWrittenSinceDone; /* the game wrote SIOMLT_SEND after the last result: its IRQ handler ran */
	uint64_t lastDoneWallNs;   /* child: wall time of the last message from the master */
	/* Frame lockstep: the master reports its frame counter in requests and
	 * pings; the child never runs more than one frame ahead of it. */
	uint32_t masterFrame;      /* unwrapped master frame counter as last reported */
	uint16_t masterFrame16;
	bool lockstepRef;          /* reference taken */
	uint32_t refMasterFrame;
	uint32_t refChildFrame;

	/* Frontend hooks: wake the core out of its audio/video sync wait while a
	 * link exchange is in progress (nudge) and restore it afterwards. Both
	 * may be NULL. nudge is called from the network thread, unnudge from the
	 * core thread. */
	void (*nudge)(void* context);
	void (*unnudge)(void* context);
	void* nudgeContext;
	bool nudged;

	/* Transfer in flight (master only, core thread) */
	bool transferActive;
	uint32_t seq;
	uint16_t recv[4];
	uint8_t gotMask;
	int32_t cyclesLeft;
	uint64_t startedNs;

	/* Fast link: speculative execution of a whole block of transfers so the
	 * network is crossed once per frame instead of once per transfer. */
	bool fastEnabled;
	bool fastActive;
	int fastPhase;
	void* snapshot;            /* struct GBASerializedState* (master: frame start; child: block start) */
	bool snapshotValid;
	uint16_t snapSend;
	uint8_t* saveSnapshot;     /* savedata contents are not part of the savestate */
	size_t saveSnapshotSize;
	uint32_t lastTickAt;
	uint32_t snapCycles;
	uint32_t reportFrame;      /* frame number advertised to children (committed timeline only) */
	uint32_t testAbortEvery;   /* debug: NETLINK_TEST_ABORT_EVERY forces an abort every N blocks */
	uint32_t specHash[16];     /* debug: end-of-spec-frame fingerprint */
	bool specHashValid;         /* SIOMLT_SEND is not part of the savestate */
	uint32_t snapFrame;
	uint16_t frozenKeys;
	uint32_t lastFrameSeen;
	bool exactThisFrame;
	bool abortFlag;
	bool specOverflow;
	bool localTransfer;
	uint16_t specWords[NETLINK_FAST_MAX];
	uint16_t childWords[NETLINK_FAST_MAX];
	/* Child words seen in the previous frame, by transfer index within the frame:
	 * used as placeholders while speculating. A speculation segment starts at a
	 * frame boundary or right after an exact (networked) transfer completes. */
	uint16_t prevFrameChild[NETLINK_FAST_MAX];
	int prevFrameCount;
	uint16_t curFrameChild[NETLINK_FAST_MAX];
	int curFrameCount;
	int frameIdx;              /* transfers started in this frame so far */
	int segStartIdx;           /* frameIdx at the segment snapshot */
	int activeIdx;             /* frameIdx of the transfer in flight */
	uint32_t activeFrame;
	int specCount;
	int realIndex;
	uint32_t blockSeq;
	int respK;
	int respRecv;
	bool respComplete;
	int consecutiveAborts;
	uint32_t fastBlocks;
	uint32_t fastAborts;
	/* child side of a block */
	uint16_t blockMaster[NETLINK_FAST_MAX];
	uint16_t blockChild[NETLINK_FAST_MAX];
	int blockK;
	int blockIndex;
	int blockRecv;
	uint32_t blockReqSeq;
	bool blockActive;
	bool childSnapshotValid;
	bool blockSent;
	/* Input is sampled once per frame while speculating so both runs see the same keys. */
	struct mKeyCallback keyCb;
	struct mKeyCallback* savedKeyCallback;
	bool keysOverridden;
	bool keysTracking;      /* frontend uses setKeys: latest value is tracked in pendingKeys */
	uint16_t pendingKeys;
	struct mCoreCallbacks coreCb;
	bool coreCbRegistered;

	/* Statistics */
	uint32_t transfers;
	uint32_t timeouts;
	uint32_t lastLatencyUs;
	uint32_t avgLatencyUs;
	uint32_t maxLatencyUs;
	uint64_t lastPingNs;

	struct mTimingEvent event;
	struct GBASIO* sio;
};

void GBASIONetLinkCreate(struct GBASIONetLink* link);
bool GBASIONetLinkHost(struct GBASIONetLink* link, uint16_t port);
bool GBASIONetLinkJoin(struct GBASIONetLink* link, const char* address, uint16_t port);
void GBASIONetLinkStop(struct GBASIONetLink* link);
/* Snapshot of the connection for UIs. */
void GBASIONetLinkStatus(struct GBASIONetLink* link, int* id, int* attached, bool* connected, uint32_t* transfers);
/* Enable or disable the speculative fast link (default: enabled). */
void GBASIONetLinkSetFast(struct GBASIONetLink* link, bool enable);
/* Fast link counters: accepted blocks, aborts, and whether fast mode is currently in use. */
void GBASIONetLinkFastStats(struct GBASIONetLink* link, uint32_t* blocks, uint32_t* aborts, bool* active);
void GBASIONetLinkSetNudge(struct GBASIONetLink* link, void (*nudge)(void*), void (*unnudge)(void*), void* context);
/* Master-side transfer statistics (timeouts and round-trip latency in microseconds). */
void GBASIONetLinkStats(struct GBASIONetLink* link, uint32_t* timeouts, uint32_t* avgLatencyUs, uint32_t* maxLatencyUs);

CXX_GUARD_END

#endif
