/* Copyright (c) 2026 NewDevNow2026
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include <mgba/internal/gba/sio/netlink.h>

#include <mgba/internal/gba/gba.h>
#include <mgba/internal/gba/io.h>
#include <mgba/internal/gba/sio.h>
#include <mgba/internal/gba/video.h>
#include <mgba/internal/gba/serialize.h>
#include <mgba/internal/gba/audio.h>
#include <mgba/core/blip_buf.h>
#include <mgba/internal/gba/savedata.h>
#include <mgba/internal/arm/macros.h>
#include <stddef.h>

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#ifdef __ANDROID__
#include <android/log.h>
#define NL_LOG(...) __android_log_print(ANDROID_LOG_INFO, "NetLink", __VA_ARGS__)
#else
#define NL_LOG(...) do { fprintf(stderr, "[netlink] " __VA_ARGS__); fputc('\n', stderr); } while (0)
#endif

enum {
	MSG_HELLO = 1,     /* client -> host: a = protocol version */
	MSG_WELCOME = 2,   /* host -> client: a = player id, b = attached */
	MSG_STATE = 3,     /* host -> all: a = attached, b = multiMask */
	MSG_MODE = 4,      /* client -> host: a = in multiplayer mode */
	MSG_XFER_REQ = 5,  /* host -> clients: seq, a = baud, v[0] = master value */
	MSG_XFER_RESP = 6, /* client -> host: seq, a = id, v[0] = value */
	MSG_XFER_DONE = 7, /* host -> clients: seq, v[0..3] */
	MSG_PING = 8,       /* both ways, keeps the Wi-Fi radios awake; carries the master frame */
	MSG_BLOCK_REQ = 10, /* host -> child: seq, a = K words, b = master frame; followed by BLOCK_DATA */
	MSG_BLOCK_DATA = 11,/* both ways: seq, a = word offset, v[0..3] */
	MSG_BLOCK_RESP = 12,/* child -> host: seq, a = K; followed by BLOCK_DATA */
	MSG_BLOCK_ABORT = 13,/* host -> child: discard the block, restore and go exact */
	/* internal events */
	EV_XFER_START = 32,
	EV_XFER_DONE = 33,
	EV_XFER_RESP = 34,
	EV_STATE = 35,
	EV_BLOCK_START = 36,
	EV_BLOCK_ABORT = 37,
};

enum FastPhase {
	FAST_IDLE = 0,
	FAST_SPEC,   /* speculative frame running, recording our words */
	FAST_REAL,   /* real frame running with the child's words known */
};
#define FAST_MAX_ABORTS 8
#define RESTORE_ALIGN 1024 /* largest timer prescaler */

#define RESTORE_MAX_TRIES 16


/* Master-clock cycles to wait before a restore keeps the prescaler phase (0 =
 * aligned now). Only timing->masterCycles counts: the CPU's pending cycles are
 * part of the saved state and come back with it. */
static int32_t _restoreWait(struct mTiming* timing, uint32_t snapMaster) {
	return (int32_t) ((RESTORE_ALIGN - ((timing->masterCycles - snapMaster) & (RESTORE_ALIGN - 1))) & (RESTORE_ALIGN - 1));
}

/* Delay to pass to mTimingSchedule so the event lands exactly 'masterWait' master cycles from now. */
static int32_t _scheduleDelay(struct mTiming* timing, int32_t masterWait) {
	return masterWait - *timing->relativeCycles;
}

/* Regular ticks sit on absolute multiples of RESTORE_ALIGN, so the tick grid is
 * the same in the speculative and the real pass whatever instruction overshoot
 * the snapshot or restore tick had. */
static int32_t _latticeWait(uint32_t now) {
	return (int32_t) (RESTORE_ALIGN - (now & (RESTORE_ALIGN - 1)));
}

#define POLL_PERIOD 1024           /* cycles between queue checks (~61 us) */
/* A child answers the next request once its game has serviced the serial IRQ
 * of the previous transfer (detected by a write to SIOMLT_SEND) or, as a
 * fallback for games that do not reload the register, after this many cycles. */
#define CHILD_REPLY_FALLBACK 67108 /* ~4 ms */
/* A child never runs more than this far ahead of the master's transfers while
 * the master is actively transferring: keeps both games in step (the link
 * protocols assume both consoles run at 60 fps). */
#define CHILD_MAX_AHEAD_FRAMES 1
#define CHILD_ACTIVE_WINDOW_NS 2000000000ULL /* no word from the master for this long: run free */
/* How long the master stalls its emulated clock waiting for the children's
 * answers before giving up on a transfer (an absent GBA reads as 0xFFFF). */
#define TRANSFER_TIMEOUT_NS 400000000ULL
#define KEEPALIVE_NS 20000000ULL

static int s_trace = -1;
#ifdef __ANDROID__
#include <sys/system_properties.h>
#endif
static bool _traceOn(void) {
	if (s_trace < 0) {
		s_trace = getenv("NETLINK_TRACE") ? 1 : 0;
#ifdef __ANDROID__
		char prop[PROP_VALUE_MAX] = "";
		if (__system_property_get("debug.cartucho.nltrace", prop) > 0 && prop[0] == '1') {
			s_trace = 1;
		}
#endif
	}
	return s_trace;
}
#ifdef __ANDROID__
#define NL_TRACE(...) do { if (_traceOn()) { __android_log_print(ANDROID_LOG_DEBUG, "NetLinkTrace", __VA_ARGS__); } } while (0)
#else
#define NL_TRACE(...) do { if (_traceOn()) { fprintf(stderr, "[nltrace %llu] ", (unsigned long long) (_nowNs() / 1000000ULL % 1000000ULL)); fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } } while (0)
#endif

static uint64_t _nowNs(void);

/* Debug aids (NETLINK_TRACE): watch one IWRAM word and dump the CPU at every tick of one frame. */
static uint32_t s_watchOff = 0xFFFFFFFF;
static uint32_t s_watchLast;
static int64_t s_traceFrame = -1;
static void _debugTick(struct GBASIONetLink* link, uint32_t nowCycles) {
	if (!_traceOn()) {
		return;
	}
	struct GBA* gba = link->sio->p;
	if (s_watchOff == 0xFFFFFFFF) {
		const char* w = getenv("NETLINK_WATCH");
		s_watchOff = w ? (uint32_t) strtoul(w, NULL, 16) : 0xFFFFFFFE;
		const char* f = getenv("NETLINK_TRACE_FRAME");
		s_traceFrame = f ? atol(f) : -1;
	}
	if (s_watchOff < 0x8000) {
		uint32_t v = *(uint32_t*) ((uint8_t*) gba->memory.iwram + s_watchOff);
		if (v != s_watchLast) {
			NL_TRACE("%c watch frame=%u cyc=%u val=%08x pc=%08x phase=%d", link->host ? 'M' : 'C', gba->video.frameCounter, nowCycles - link->snapCycles, v, gba->cpu->gprs[15], link->fastPhase);
			s_watchLast = v;
		}
	}
	if ((s_traceFrame >= 0 && gba->video.frameCounter == (uint32_t) s_traceFrame) || (s_traceFrame == -2 && link->fastPhase != FAST_IDLE)) {
		NL_TRACE("%c tick cyc=%u pc=%08x vc=%d cycles=%d halt=%d phase=%d r0=%08x sp=%08x lr=%08x", link->host ? 'M' : 'C', nowCycles - link->snapCycles, gba->cpu->gprs[15], gba->video.vcount, gba->cpu->cycles, gba->cpu->halted, link->fastPhase, gba->cpu->gprs[0], gba->cpu->gprs[13], gba->cpu->gprs[14]);
	}
}

static void _hostFrameBoundary(struct GBASIONetLink* link, struct mTiming* timing);
static void _overrideKeys(struct GBASIONetLink* link, bool on);
static void _applyStatusBits(struct GBASIONetLink* link);
static void _hostAbortBlock(struct GBASIONetLink* link, struct mTiming* timing, const char* why);
static bool _init(struct GBASIODriver* driver);
static void _deinit(struct GBASIODriver* driver);
static bool _load(struct GBASIODriver* driver);
static bool _unload(struct GBASIODriver* driver);
static uint16_t _writeRegister(struct GBASIODriver* driver, uint32_t address, uint16_t value);
static void _tick(struct mTiming* timing, void* context, uint32_t cyclesLate);

static uint64_t _nowNs(void) {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t) ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

/* ---------------------------------------------------------------------- */
/* Messaging                                                               */

static void _sendTo(struct GBASIONetLink* link, int fd, const struct NetLinkMsg* msg) {
	if (fd < 0) {
		return;
	}
	pthread_mutex_lock(&link->sendLock);
	const uint8_t* p = (const uint8_t*) msg;
	size_t left = sizeof(*msg);
	while (left) {
		ssize_t n = send(fd, p, left, MSG_NOSIGNAL);
		if (n <= 0) {
			if (n < 0 && errno == EINTR) {
				continue;
			}
			break;
		}
		p += n;
		left -= n;
	}
	pthread_mutex_unlock(&link->sendLock);
}

static void _broadcast(struct GBASIONetLink* link, const struct NetLinkMsg* msg) {
	int i;
	for (i = 1; i < NETLINK_MAX_PLAYERS; ++i) {
		if (link->peers[i].connected) {
			_sendTo(link, link->peers[i].fd, msg);
		}
	}
}

static void _sendWords(struct GBASIONetLink* link, int fd, uint32_t seq, const uint16_t* words, int count) {
	int i;
	for (i = 0; i < count; i += 4) {
		struct NetLinkMsg m = { .type = MSG_BLOCK_DATA, .a = (uint8_t) i, .seq = seq };
		int j;
		for (j = 0; j < 4 && i + j < count; ++j) {
			m.v[j] = words[i + j];
		}
		_sendTo(link, fd, &m);
	}
}

static void _push(struct GBASIONetLink* link, const struct NetLinkMsg* msg) {
	/* Caller holds link->lock */
	int next = (link->qTail + 1) % 64;
	if (next == link->qHead) {
		return; /* queue full, drop */
	}
	link->queue[link->qTail] = *msg;
	link->qTail = next;
}

static bool _pop(struct GBASIONetLink* link, struct NetLinkMsg* msg) {
	/* Caller holds link->lock */
	if (link->qHead == link->qTail) {
		return false;
	}
	*msg = link->queue[link->qHead];
	link->qHead = (link->qHead + 1) % 64;
	return true;
}

/* Frame number reported to the children. While speculating, the live frame
 * counter briefly runs one frame ahead before the restore; reporting that
 * transient value would let the children drift ahead, so only committed
 * frames are advertised. */
static uint16_t _masterFrame16(struct GBASIONetLink* link) {
	if (!link->sio) {
		return 0;
	}
	if (link->fastEnabled && link->selfMulti) {
		return (uint16_t) link->reportFrame;
	}
	return (uint16_t) link->sio->p->video.frameCounter;
}

/* Child: fold a 16-bit master frame report into the unwrapped counter. */
static void _noteMasterFrame(struct GBASIONetLink* link, uint16_t f16) {
	int16_t delta = (int16_t) (f16 - link->masterFrame16);
	if (delta > 0) {
		link->masterFrame += delta;
	}
	link->masterFrame16 = f16;
	link->lastDoneWallNs = _nowNs();
}

static void _nudge(struct GBASIONetLink* link) {
	if (link->nudge && !link->nudged) {
		link->nudged = true;
		link->nudge(link->nudgeContext);
	}
}

static void _unnudge(struct GBASIONetLink* link) {
	if (link->unnudge && link->nudged) {
		link->nudged = false;
		link->unnudge(link->nudgeContext);
	}
}

void GBASIONetLinkSetNudge(struct GBASIONetLink* link, void (*nudge)(void*), void (*unnudge)(void*), void* context) {
	link->nudge = nudge;
	link->unnudge = unnudge;
	link->nudgeContext = context;
}

static void _hostBroadcastState(struct GBASIONetLink* link) {
	/* Caller holds link->lock */
	struct NetLinkMsg msg = { .type = MSG_STATE, .a = (uint8_t) link->attached, .b = link->multiMask };
	_broadcast(link, &msg);
	struct NetLinkMsg ev = { .type = EV_STATE, .a = (uint8_t) link->attached, .b = link->multiMask };
	_push(link, &ev);
}

static void _setNoDelay(int fd) {
	int one = 1;
	setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
}

/* ---------------------------------------------------------------------- */
/* Network thread                                                          */

static void _handleHostMessage(struct GBASIONetLink* link, int peerId, const struct NetLinkMsg* msg) {
	pthread_mutex_lock(&link->lock);
	switch (msg->type) {
	case MSG_MODE:
		link->peers[peerId].multi = msg->a;
		if (msg->a) {
			link->multiMask |= 1 << peerId;
		} else {
			link->multiMask &= ~(1 << peerId);
		}
		_hostBroadcastState(link);
		break;
	case MSG_XFER_RESP: {
		struct NetLinkMsg ev = *msg;
		ev.type = EV_XFER_RESP;
		ev.a = (uint8_t) peerId;
		_push(link, &ev);
		pthread_cond_broadcast(&link->respCond);
		break;
	}
	case MSG_BLOCK_RESP:
		if (msg->seq == link->blockSeq) {
			link->respK = msg->a;
			link->respRecv = 0;
			link->respComplete = false;
		}
		break;
	case MSG_BLOCK_DATA:
		if (msg->seq == link->blockSeq && !link->respComplete) {
			int j;
			for (j = 0; j < 4 && msg->a + j < link->respK && msg->a + j < NETLINK_FAST_MAX; ++j) {
				link->childWords[msg->a + j] = msg->v[j];
			}
			link->respRecv += 4;
			if (link->respRecv >= link->respK) {
				link->respComplete = true;
				pthread_cond_broadcast(&link->respCond);
			}
		}
		break;
	default:
		break;
	}
	pthread_mutex_unlock(&link->lock);
}

static void _handleClientMessage(struct GBASIONetLink* link, const struct NetLinkMsg* msg) {
	pthread_mutex_lock(&link->lock);
	switch (msg->type) {
	case MSG_WELCOME:
		link->id = msg->a;
		link->attached = msg->b;
		link->connected = true;
		NL_LOG("Joined as player %d (%d attached)", link->id + 1, link->attached);
		{
			struct NetLinkMsg ev = { .type = EV_STATE, .a = (uint8_t) link->attached, .b = link->multiMask };
			_push(link, &ev);
			struct NetLinkMsg mode = { .type = MSG_MODE, .a = link->selfMulti };
			pthread_mutex_unlock(&link->lock);
			_sendTo(link, link->peers[0].fd, &mode);
			return;
		}
	case MSG_STATE: {
		link->attached = msg->a;
		link->multiMask = msg->b;
		struct NetLinkMsg ev = *msg;
		ev.type = EV_STATE;
		_push(link, &ev);
		break;
	}
	case MSG_XFER_REQ: {
		if (!link->selfMulti) {
			/* Not in multiplayer mode: nothing to send, answer at once. */
			struct NetLinkMsg resp = { .type = MSG_XFER_RESP, .a = (uint8_t) link->id, .seq = msg->seq };
			resp.v[0] = 0xFFFF;
			pthread_mutex_unlock(&link->lock);
			_sendTo(link, link->peers[0].fd, &resp);
			return;
		}
		/* The answer is produced on the core thread (see _tick): the game must
		 * first service the IRQ of the previous transfer and load SIOMLT_SEND. */
		struct NetLinkMsg ev = *msg;
		ev.type = EV_XFER_START;
		_push(link, &ev);
		_noteMasterFrame(link, msg->b);
		pthread_cond_broadcast(&link->respCond);
		_nudge(link);
		break;
	}
	case MSG_XFER_DONE: {
		struct NetLinkMsg ev = *msg;
		ev.type = EV_XFER_DONE;
		_push(link, &ev);
		link->lastDoneWallNs = _nowNs();
		pthread_cond_broadcast(&link->respCond);
		_nudge(link);
		break;
	}
	case MSG_PING:
		_noteMasterFrame(link, msg->b);
		pthread_cond_broadcast(&link->respCond);
		break;
	case MSG_BLOCK_REQ:
		link->blockK = msg->a > NETLINK_FAST_MAX ? NETLINK_FAST_MAX : msg->a;
		link->blockReqSeq = msg->seq;
		link->blockRecv = 0;
		_noteMasterFrame(link, msg->b);
		break;
	case MSG_BLOCK_DATA:
		if (msg->seq == link->blockReqSeq) {
			int j;
			for (j = 0; j < 4 && msg->a + j < link->blockK; ++j) {
				link->blockMaster[msg->a + j] = msg->v[j];
			}
			link->blockRecv += 4;
			if (link->blockRecv >= link->blockK) {
				if (!link->selfMulti) {
					/* Not in multiplayer mode: answer the whole block with 0xFFFF. */
					uint16_t words[NETLINK_FAST_MAX];
					memset(words, 0xFF, sizeof(words));
					struct NetLinkMsg hdr = { .type = MSG_BLOCK_RESP, .a = (uint8_t) link->blockK, .seq = msg->seq };
					int fd = link->peers[0].fd;
					int k = link->blockK;
					pthread_mutex_unlock(&link->lock);
					_sendTo(link, fd, &hdr);
					_sendWords(link, fd, msg->seq, words, k);
					return;
				}
				struct NetLinkMsg ev = { .type = EV_BLOCK_START, .seq = msg->seq };
				_push(link, &ev);
				link->lastDoneWallNs = _nowNs();
				pthread_cond_broadcast(&link->respCond);
				_nudge(link);
			}
		}
		break;
	case MSG_BLOCK_ABORT: {
		struct NetLinkMsg ev = { .type = EV_BLOCK_ABORT, .seq = msg->seq };
		_push(link, &ev);
		pthread_cond_broadcast(&link->respCond);
		_nudge(link);
		break;
	}
	default:
		break;
	}
	pthread_mutex_unlock(&link->lock);
}

static void _dropPeer(struct GBASIONetLink* link, int i) {
	pthread_mutex_lock(&link->lock);
	if (link->peers[i].fd >= 0) {
		close(link->peers[i].fd);
	}
	link->peers[i].fd = -1;
	link->peers[i].connected = false;
	link->peers[i].multi = false;
	link->peers[i].rxLen = 0;
	if (link->host) {
		link->multiMask &= ~(1 << i);
		--link->attached;
		NL_LOG("Player %d left (%d attached)", i + 1, link->attached);
		_hostBroadcastState(link);
		pthread_cond_broadcast(&link->respCond);
	} else {
		link->connected = false;
		link->attached = 1;
		link->multiMask = link->selfMulti ? 1 : 0;
		NL_LOG("Disconnected from host");
		struct NetLinkMsg ev = { .type = EV_STATE, .a = 1, .b = link->multiMask };
		_push(link, &ev);
	}
	pthread_mutex_unlock(&link->lock);
}

static void _readPeer(struct GBASIONetLink* link, int i) {
	struct NetLinkPeer* peer = &link->peers[i];
	ssize_t n = recv(peer->fd, peer->rx + peer->rxLen, sizeof(peer->rx) - peer->rxLen, 0);
	if (n <= 0) {
		if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
			return;
		}
		_dropPeer(link, i);
		return;
	}
	peer->rxLen += n;
	if (peer->rxLen == sizeof(peer->rx)) {
		struct NetLinkMsg msg;
		memcpy(&msg, peer->rx, sizeof(msg));
		peer->rxLen = 0;
		if (link->host) {
			_handleHostMessage(link, i, &msg);
		} else {
			_handleClientMessage(link, &msg);
		}
	}
}

static void _acceptPeer(struct GBASIONetLink* link) {
	int fd = accept(link->listenFd, NULL, NULL);
	if (fd < 0) {
		return;
	}
	pthread_mutex_lock(&link->lock);
	int slot = -1;
	int i;
	for (i = 1; i < NETLINK_MAX_PLAYERS; ++i) {
		if (!link->peers[i].connected) {
			slot = i;
			break;
		}
	}
	if (slot < 0) {
		pthread_mutex_unlock(&link->lock);
		close(fd);
		return;
	}
	_setNoDelay(fd);
	link->peers[slot].fd = fd;
	link->peers[slot].connected = true;
	link->peers[slot].multi = false;
	link->peers[slot].rxLen = 0;
	++link->attached;
	NL_LOG("Player %d joined (%d attached)", slot + 1, link->attached);
	struct NetLinkMsg welcome = { .type = MSG_WELCOME, .a = (uint8_t) slot, .b = (uint16_t) link->attached };
	_sendTo(link, fd, &welcome);
	_hostBroadcastState(link);
	pthread_mutex_unlock(&link->lock);
}

static void _keepalive(struct GBASIONetLink* link) {
	struct NetLinkMsg ping = { .type = MSG_PING };
	uint64_t now = _nowNs();
	if (now - link->lastPingNs < KEEPALIVE_NS) {
		return;
	}
	link->lastPingNs = now;
	if (link->host) {
		ping.b = _masterFrame16(link);
		_broadcast(link, &ping);
	} else if (link->peers[0].connected) {
		_sendTo(link, link->peers[0].fd, &ping);
	}
}

static void* _netThread(void* arg) {
	struct GBASIONetLink* link = arg;
	while (link->running) {
		_keepalive(link);
		struct pollfd fds[NETLINK_MAX_PLAYERS + 1];
		int map[NETLINK_MAX_PLAYERS + 1];
		int n = 0;
		if (link->host && link->listenFd >= 0) {
			fds[n].fd = link->listenFd;
			fds[n].events = POLLIN;
			map[n] = -1;
			++n;
		}
		int i;
		for (i = 0; i < NETLINK_MAX_PLAYERS; ++i) {
			if (link->peers[i].connected && link->peers[i].fd >= 0) {
				fds[n].fd = link->peers[i].fd;
				fds[n].events = POLLIN;
				map[n] = i;
				++n;
			}
		}
		if (!n) {
			usleep(50000);
			continue;
		}
		int r = poll(fds, n, 10);
		if (r <= 0) {
			continue;
		}
		for (i = 0; i < n; ++i) {
			if (!(fds[i].revents & (POLLIN | POLLHUP | POLLERR))) {
				continue;
			}
			if (map[i] < 0) {
				_acceptPeer(link);
			} else {
				_readPeer(link, map[i]);
			}
		}
	}
	return NULL;
}

/* ---------------------------------------------------------------------- */
/* Public API                                                              */

void GBASIONetLinkCreate(struct GBASIONetLink* link) {
	memset(link, 0, sizeof(*link));
	link->d.init = _init;
	link->d.deinit = _deinit;
	link->d.load = _load;
	link->d.unload = _unload;
	link->d.writeRegister = _writeRegister;
	link->listenFd = -1;
	int i;
	for (i = 0; i < NETLINK_MAX_PLAYERS; ++i) {
		link->peers[i].fd = -1;
	}
	link->attached = 1;
	link->fastEnabled = true;
	{
		const char* env = getenv("NETLINK_TEST_ABORT_EVERY");
		if (env) {
			link->testAbortEvery = (uint32_t) atoi(env);
		}
	}
	link->event.context = link;
	link->event.callback = _tick;
	link->event.name = "Cartucho NetLink";
	link->event.priority = 0x80;
	pthread_mutex_init(&link->lock, NULL);
	pthread_mutex_init(&link->sendLock, NULL);
	pthread_cond_init(&link->respCond, NULL);
}

static void _startThread(struct GBASIONetLink* link) {
	link->running = true;
	pthread_create(&link->thread, NULL, _netThread, link);
	link->threadRunning = true;
}

bool GBASIONetLinkHost(struct GBASIONetLink* link, uint16_t port) {
	GBASIONetLinkStop(link);
	int fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd < 0) {
		return false;
	}
	int one = 1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
	struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(port), .sin_addr.s_addr = htonl(INADDR_ANY) };
	if (bind(fd, (struct sockaddr*) &addr, sizeof(addr)) < 0 || listen(fd, 3) < 0) {
		NL_LOG("Cannot listen on port %u: %s", port, strerror(errno));
		close(fd);
		return false;
	}
	pthread_mutex_lock(&link->lock);
	link->host = true;
	link->id = 0;
	link->attached = 1;
	link->connected = true;
	link->port = port;
	link->listenFd = fd;
	link->multiMask = link->selfMulti ? 1 : 0;
	pthread_mutex_unlock(&link->lock);
	_startThread(link);
	NL_LOG("Hosting on port %u", port);
	return true;
}

bool GBASIONetLinkJoin(struct GBASIONetLink* link, const char* address, uint16_t port) {
	GBASIONetLinkStop(link);
	struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
	struct addrinfo* res = NULL;
	char portStr[8];
	snprintf(portStr, sizeof(portStr), "%u", port);
	if (getaddrinfo(address, portStr, &hints, &res) != 0 || !res) {
		NL_LOG("Cannot resolve %s", address);
		return false;
	}
	int fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
	if (fd < 0) {
		freeaddrinfo(res);
		return false;
	}
	/* Connect with a timeout so the UI does not hang on a wrong address. */
	fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
	int rc = connect(fd, res->ai_addr, res->ai_addrlen);
	freeaddrinfo(res);
	if (rc < 0 && errno == EINPROGRESS) {
		struct pollfd pfd = { .fd = fd, .events = POLLOUT };
		if (poll(&pfd, 1, 4000) <= 0) {
			rc = -1;
		} else {
			int err = 0;
			socklen_t len = sizeof(err);
			getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len);
			rc = err ? -1 : 0;
		}
	}
	if (rc < 0) {
		NL_LOG("Cannot connect to %s:%u", address, port);
		close(fd);
		return false;
	}
	fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) & ~O_NONBLOCK);
	_setNoDelay(fd);
	pthread_mutex_lock(&link->lock);
	link->host = false;
	snprintf(link->address, sizeof(link->address), "%s", address);
	link->port = port;
	link->peers[0].fd = fd;
	link->peers[0].connected = true;
	link->peers[0].rxLen = 0;
	link->connected = false; /* until WELCOME */
	link->attached = 1;
	pthread_mutex_unlock(&link->lock);
	struct NetLinkMsg hello = { .type = MSG_HELLO, .a = NETLINK_PROTOCOL_VERSION };
	_sendTo(link, fd, &hello);
	_startThread(link);
	NL_LOG("Connecting to %s:%u", address, port);
	return true;
}

void GBASIONetLinkStop(struct GBASIONetLink* link) {
	link->stopping = true;
	pthread_mutex_lock(&link->lock);
	pthread_cond_broadcast(&link->respCond);
	pthread_mutex_unlock(&link->lock);
	if (link->threadRunning) {
		link->running = false;
		pthread_join(link->thread, NULL);
		link->threadRunning = false;
	}
	pthread_mutex_lock(&link->lock);
	int i;
	for (i = 0; i < NETLINK_MAX_PLAYERS; ++i) {
		if (link->peers[i].fd >= 0) {
			close(link->peers[i].fd);
		}
		link->peers[i].fd = -1;
		link->peers[i].connected = false;
		link->peers[i].multi = false;
		link->peers[i].rxLen = 0;
	}
	if (link->listenFd >= 0) {
		close(link->listenFd);
		link->listenFd = -1;
	}
	link->connected = false;
	link->attached = 1;
	link->multiMask = link->selfMulti ? 1 : 0;
	link->qHead = link->qTail = 0;
	link->transferActive = false;
	link->reqPending = false;
	link->haveDone = false;
	link->sendWrittenSinceDone = false;
	link->nudged = false;
	link->lockstepRef = false;
	link->masterFrame = 0;
	link->masterFrame16 = 0;
	link->fastPhase = FAST_IDLE;
	link->fastActive = false;
	link->snapshotValid = false;
	link->childSnapshotValid = false;
	link->blockActive = false;
	link->exactThisFrame = false;
	link->abortFlag = false;
	link->localTransfer = false;
	link->consecutiveAborts = 0;
	link->prevFrameCount = 0;
	link->curFrameCount = 0;
	link->frameIdx = 0;
	link->blockSent = false;
	link->boundaryPending = false;
	link->specFrozen = false;
	link->childRestorePending = false;
	if (link->sio) {
		_overrideKeys(link, false);
	}
	link->id = 0;
	link->stopping = false;
	pthread_mutex_unlock(&link->lock);
}

void GBASIONetLinkStatus(struct GBASIONetLink* link, int* id, int* attached, bool* connected, uint32_t* transfers) {
	pthread_mutex_lock(&link->lock);
	if (id) *id = link->id;
	if (attached) *attached = link->attached;
	if (connected) *connected = link->connected;
	if (transfers) *transfers = link->transfers;
	pthread_mutex_unlock(&link->lock);
}

/* ---------------------------------------------------------------------- */
/* SIO driver (core thread)                                                */

static uint16_t _frozenKeys(struct mKeyCallback* cb) {
	struct GBASIONetLink* link = (struct GBASIONetLink*) ((char*) cb - offsetof(struct GBASIONetLink, keyCb));
	return link->frozenKeys;
}

static uint16_t _liveKeys(struct GBASIONetLink* link) {
	struct GBA* gba = link->sio->p;
	if (link->keysOverridden) {
		return link->savedKeyCallback->readKeys(link->savedKeyCallback);
	}
	if (gba->keyCallback) {
		return gba->keyCallback->readKeys(gba->keyCallback);
	}
	if (link->keysTracking) {
		if (gba->keysActive != link->frozenKeys) {
			link->pendingKeys = gba->keysActive;
		}
		return link->pendingKeys;
	}
	return gba->keysActive;
}

/* While a frame may be replayed, KEYINPUT must return the same value in both
 * runs. Frontends either install a key callback (we wrap it) or call setKeys,
 * which writes gba->keysActive directly (we track the latest value and pin the
 * register to the frozen one every tick). */
static void _overrideKeys(struct GBASIONetLink* link, bool on) {
	struct GBA* gba = link->sio->p;
	if (on) {
		if (gba->keyCallback && !link->keysOverridden) {
			link->savedKeyCallback = gba->keyCallback;
			link->keyCb.readKeys = _frozenKeys;
			link->keyCb.requireOpposingDirections = gba->keyCallback->requireOpposingDirections;
			gba->keyCallback = &link->keyCb;
			link->keysOverridden = true;
		} else if (!gba->keyCallback && !link->keysOverridden && !link->keysTracking) {
			link->pendingKeys = gba->keysActive;
			link->keysTracking = true;
		}
	} else {
		if (link->keysOverridden) {
			if (gba->keyCallback == &link->keyCb) {
				gba->keyCallback = link->savedKeyCallback;
			}
			link->keysOverridden = false;
		}
		if (link->keysTracking) {
			if (gba->keysActive != link->frozenKeys) {
				link->pendingKeys = gba->keysActive;
			}
			gba->keysActive = link->pendingKeys;
			link->keysTracking = false;
		}
	}
}

/* Runs inside the video event at the start of VBlank: make the link tick run
 * right now, before the CPU executes any VBlank code, so the master snapshot
 * (or restore) happens before the first transfer of the frame. */
static void _frameEndedHook(void* context) {
	struct GBASIONetLink* link = context;
	if (!link->host || !link->sio) {
		return;
	}
	struct mTiming* timing = &link->sio->p->timing;
	if (mTimingIsScheduled(timing, &link->event)) {
		mTimingDeschedule(timing, &link->event);
		mTimingSchedule(timing, &link->event, 0);
	}
}

static bool _init(struct GBASIODriver* driver) {
	struct GBASIONetLink* link = (struct GBASIONetLink*) driver;
	link->sio = driver->p;
	if (!link->snapshot) {
		link->snapshot = malloc(sizeof(struct GBASerializedState));
	}
	if (!link->coreCbRegistered) {
		memset(&link->coreCb, 0, sizeof(link->coreCb));
		link->coreCb.context = link;
		link->coreCb.videoFrameEnded = _frameEndedHook;
		*mCoreCallbacksListAppend(&link->sio->p->coreCallbacks) = link->coreCb;
		link->coreCbRegistered = true;
	}
	return true;
}

static void _unregisterCoreCb(struct GBASIONetLink* link) {
	if (!link->coreCbRegistered || !link->sio) {
		return;
	}
	struct mCoreCallbacksList* list = &link->sio->p->coreCallbacks;
	size_t i;
	for (i = 0; i < mCoreCallbacksListSize(list); ++i) {
		if (mCoreCallbacksListGetPointer(list, i)->context == link) {
			mCoreCallbacksListShift(list, i, 1);
			break;
		}
	}
	link->coreCbRegistered = false;
}

/* Load the snapshot without disturbing the timing loop we are called from:
 * mTimingTick caches masterCycles, so the restored state is rebased onto the
 * current master clock (events are stored relative; DMA timestamps are absolute). */

/* Debug: fingerprint of the game-visible state, printed with NETLINK_TRACE. */
static uint32_t _fnv(uint32_t h, const void* data, size_t len) {
	const uint8_t* p = data;
	size_t i;
	for (i = 0; i < len; ++i) {
		h ^= p[i];
		h *= 16777619u;
	}
	return h;
}

struct NetLinkHash {
	uint32_t iw, w, io, cpu, tim, save, pal;
	uint32_t pc, frame;
	uint16_t siocnt, rcnt, send, m0, m1, keys;
};

static void _computeHashRel(struct GBASIONetLink* link, struct NetLinkHash* h, int32_t* relOut, int* relN);

static void _computeHash(struct GBASIONetLink* link, struct NetLinkHash* h) {
	_computeHashRel(link, h, NULL, NULL);
}

static void _computeHashRel(struct GBASIONetLink* link, struct NetLinkHash* h, int32_t* relOut, int* relN) {
	struct GBA* gba = link->sio->p;
	struct mTiming* t = &gba->timing;
	uint32_t now = mTimingCurrentTime(t);
	memset(h, 0, sizeof(*h));
	h->iw = _fnv(2166136261u, gba->memory.iwram, 0x8000);
	h->w = _fnv(2166136261u, gba->memory.wram, 0x40000);
	h->io = _fnv(2166136261u, gba->memory.io, 0x400);
	h->pal = _fnv(2166136261u, gba->video.palette, 0x400);
	h->pal = _fnv(h->pal, gba->video.oam.raw, 0x400);
	uint32_t c = _fnv(2166136261u, gba->cpu->gprs, sizeof(gba->cpu->gprs));
	c = _fnv(c, &gba->cpu->cpsr, sizeof(gba->cpu->cpsr));
	c = _fnv(c, &gba->cpu->cycles, sizeof(int32_t));
	c = _fnv(c, gba->cpu->prefetch, sizeof(gba->cpu->prefetch));
	h->cpu = c;
	int32_t rel[20];
	int n = 0;
	int i;
	for (i = 0; i < 4; ++i) {
		rel[n++] = gba->timers[i].lastEvent - now;
		rel[n++] = mTimingIsScheduled(t, &gba->timers[i].event) ? (int32_t) (gba->timers[i].event.when - now) : 0x7fffffff;
		rel[n++] = gba->memory.dma[i].when - now;
	}
	rel[n++] = mTimingIsScheduled(t, &gba->irqEvent) ? (int32_t) (gba->irqEvent.when - now) : 0x7fffffff;
	rel[n++] = gba->video.event.when - now;
	rel[n++] = mTimingIsScheduled(t, &gba->memory.savedata.dust) ? (int32_t) (gba->memory.savedata.dust.when - now) : 0x7fffffff;
	rel[n++] = gba->video.vcount | (gba->cpu->halted << 16) | (gba->cpuBlocked << 17) | (gba->memory.savedata.flashState << 18) | (gba->memory.savedata.command << 20);
	h->tim = _fnv(2166136261u, rel, n * sizeof(int32_t));
	if (relOut) {
		memcpy(relOut, rel, n * sizeof(int32_t));
		*relN = n;
	}
	struct GBASavedata* sd = &gba->memory.savedata;
	if (sd->data) {
		size_t size = GBASavedataSize(sd);
		if (size <= 0x20000) {
			h->save = _fnv(2166136261u, sd->data, size);
		}
	}
	h->pc = gba->cpu->gprs[15];
	h->frame = gba->video.frameCounter;
	h->siocnt = link->sio->siocnt;
	h->rcnt = link->sio->rcnt;
	h->send = gba->memory.io[REG_SIOMLT_SEND >> 1];
	h->m0 = gba->memory.io[REG_SIOMULTI0 >> 1];
	h->m1 = gba->memory.io[REG_SIOMULTI1 >> 1];
	h->keys = gba->keysActive;
}

static void _printHash(struct GBASIONetLink* link, const char* tag, const struct NetLinkHash* h) {
	struct GBA* gba = link->sio->p;
	NL_TRACE("%c %s frame=%u pc=%08x iw=%08x w=%08x io=%08x cpu=%08x tim=%08x save=%08x pal=%08x siocnt=%04x rcnt=%04x send=%04x m0=%04x m1=%04x keys=%04x mc=%u ccyc=%d nev=%d halt=%d vc=%d",
	         link->host ? 'M' : 'C', tag, h->frame, h->pc, h->iw, h->w, h->io, h->cpu, h->tim, h->save, h->pal, h->siocnt, h->rcnt, h->send, h->m0, h->m1, h->keys,
	         gba->timing.masterCycles & 1023, gba->cpu->cycles, gba->cpu->nextEvent, gba->cpu->halted, gba->video.vcount);
}

static void _traceStateHash(struct GBASIONetLink* link, const char* tag) {
	if (!_traceOn()) {
		return;
	}
	struct NetLinkHash h;
	_computeHash(link, &h);
	_printHash(link, tag, &h);
}

/* The SIO status bits live in sio->siocnt/rcnt, not in the I/O array a
 * savestate restores from, so sync them first. */
static void _takeSnapshot(struct GBASIONetLink* link) {
	struct GBA* gba = link->sio->p;
	gba->memory.io[REG_SIOCNT >> 1] = link->sio->siocnt;
	gba->memory.io[REG_RCNT >> 1] = link->sio->rcnt;
	link->snapSend = gba->memory.io[REG_SIOMLT_SEND >> 1];
	link->snapCycles = mTimingCurrentTime(&gba->timing);
	link->snapMaster = gba->timing.masterCycles;
	GBASerialize(gba, link->snapshot);
	/* Flash/SRAM contents are not in the state; a frame that saves must be replayable too. */
	struct GBASavedata* sd = &gba->memory.savedata;
	size_t size = sd->data ? GBASavedataSize(sd) : 0;
	if (size > 0x20000) {
		size = 0;
	}
	if (size && !link->saveSnapshot) {
		link->saveSnapshot = malloc(0x20000);
	}
	if (size && link->saveSnapshot) {
		memcpy(link->saveSnapshot, sd->data, size);
		link->saveSnapshotSize = size;
	} else {
		link->saveSnapshotSize = 0;
	}
	_traceStateHash(link, "snap");
}

static bool _restoreSnapshot(struct GBASIONetLink* link, struct mTiming* timing) {
	struct GBA* gba = link->sio->p;
	struct GBASerializedState* state = link->snapshot;
	uint32_t snapMaster;
	LOAD_32(snapMaster, 0, &state->masterCycles);
	uint32_t delta = timing->masterCycles - snapMaster;
	STORE_32(timing->masterCycles, 0, &state->masterCycles);
	STORE_64LE(timing->globalCycles, 0, &state->globalCycles);
	int i;
	for (i = 0; i < 4; ++i) {
		uint32_t when;
		LOAD_32(when, 0, &state->dma[i].when);
		when += delta;
		STORE_32(when, 0, &state->dma[i].when);
	}
	/* GBADeserialize writes the SIO registers through this driver, which locks. */
	pthread_mutex_unlock(&link->lock);
	bool ok = GBADeserialize(gba, state);
	pthread_mutex_lock(&link->lock);
	/* SIOMLT_SEND is not part of the savestate. */
	gba->memory.io[REG_SIOMLT_SEND >> 1] = link->snapSend;
	struct GBASavedata* sd = &gba->memory.savedata;
	if (link->saveSnapshotSize && sd->data && GBASavedataSize(sd) == link->saveSnapshotSize
	    && memcmp(sd->data, link->saveSnapshot, link->saveSnapshotSize)) {
		memcpy(sd->data, link->saveSnapshot, link->saveSnapshotSize);
		sd->dirty |= 1; /* mSAVEDATA_DIRT_NEW (private to savedata.c) */
	}
	_applyStatusBits(link);
	if (!mTimingIsScheduled(timing, &link->event)) {
		mTimingSchedule(timing, &link->event, _latticeWait(mTimingCurrentTime(timing)));
	}
	link->lastTickAt = mTimingCurrentTime(timing);
	link->snapCycles = link->lastTickAt; /* the rebased clock now reads the snapshot instant */
	link->snapMaster = timing->masterCycles;
	_traceStateHash(link, "rest");
	if (!ok) {
		NL_LOG("Snapshot restore failed; fast link disabled");
		link->fastEnabled = false;
	}
	return ok;
}

void GBASIONetLinkSetFast(struct GBASIONetLink* link, bool enable) {
	pthread_mutex_lock(&link->lock);
	link->fastEnabled = enable;
	if (!enable) {
		link->consecutiveAborts = 0;
	}
	pthread_mutex_unlock(&link->lock);
}

void GBASIONetLinkFastStats(struct GBASIONetLink* link, uint32_t* blocks, uint32_t* aborts, bool* active) {
	pthread_mutex_lock(&link->lock);
	if (blocks) *blocks = link->fastBlocks;
	if (aborts) *aborts = link->fastAborts;
	if (active) *active = link->fastEnabled && link->fastActive;
	pthread_mutex_unlock(&link->lock);
}

static void _deinit(struct GBASIODriver* driver) {
	struct GBASIONetLink* link = (struct GBASIONetLink*) driver;
	GBASIONetLinkStop(link);
	if (link->sio) {
		_overrideKeys(link, false);
	}
	_unregisterCoreCb(link);
	free(link->snapshot);
	link->snapshot = NULL;
	free(link->saveSnapshot);
	link->saveSnapshot = NULL;
	free(link->specIwram);
	link->specIwram = NULL;
	free(link->specIo);
	link->specIo = NULL;
	link->saveSnapshotSize = 0;
	if (link->sio && mTimingIsScheduled(&link->sio->p->timing, &link->event)) {
		mTimingDeschedule(&link->sio->p->timing, &link->event);
	}
}

static bool _ready(struct GBASIONetLink* link) {
	/* Caller holds lock. Ready when at least two players and all of them in multi mode. */
	if (link->attached < 2) {
		return false;
	}
	uint8_t all = 0;
	if (link->host) {
		all = 1;
		int i;
		for (i = 1; i < NETLINK_MAX_PLAYERS; ++i) {
			if (link->peers[i].connected) {
				all |= 1 << i;
			}
		}
	} else {
		/* host broadcasts the mask of players in multi mode; attached tells how many there are */
		int i;
		for (i = 0; i < link->attached; ++i) {
			all |= 1 << i;
		}
		/* players may not be contiguous after someone leaves; accept any mask with popcount == attached */
		int count = 0;
		uint8_t m = link->multiMask;
		while (m) {
			count += m & 1;
			m >>= 1;
		}
		return count >= link->attached && link->selfMulti;
	}
	return (link->multiMask & all) == all;
}

static void _applyStatusBits(struct GBASIONetLink* link) {
	/* Caller holds lock */
	struct GBASIO* sio = link->sio;
	sio->siocnt = GBASIOMultiplayerSetReady(sio->siocnt, _ready(link));
	sio->siocnt = GBASIOMultiplayerSetSlave(sio->siocnt, link->id != 0 || link->attached < 2);
	if (link->id) {
		sio->rcnt |= 4;
	} else {
		sio->rcnt &= ~4;
	}
}

static bool _load(struct GBASIODriver* driver) {
	struct GBASIONetLink* link = (struct GBASIONetLink*) driver;
	pthread_mutex_lock(&link->lock);
	link->selfMulti = true;
	if (link->host) {
		link->multiMask |= 1;
		_hostBroadcastState(link);
	} else {
		link->multiMask |= 1 << link->id;
	}
	_applyStatusBits(link);
	pthread_mutex_unlock(&link->lock);
	if (!link->host && link->peers[0].connected) {
		struct NetLinkMsg mode = { .type = MSG_MODE, .a = 1 };
		_sendTo(link, link->peers[0].fd, &mode);
	}
	if (!mTimingIsScheduled(&link->sio->p->timing, &link->event)) {
		mTimingSchedule(&link->sio->p->timing, &link->event, _latticeWait(mTimingCurrentTime(&link->sio->p->timing)));
	}
	NL_LOG("Multiplayer mode entered (player %d)", link->id + 1);
	NL_TRACE("%c load frame=%u siocnt=%04x rcnt=%04x", link->host ? 'M' : 'C', link->sio->p->video.frameCounter, link->sio->siocnt, link->sio->rcnt);
	return true;
}

static bool _unload(struct GBASIODriver* driver) {
	struct GBASIONetLink* link = (struct GBASIONetLink*) driver;
	NL_TRACE("%c unload frame=%u pc=%08x fastPhase=%d", link->host ? 'M' : 'C', link->sio->p->video.frameCounter, link->sio->p->cpu->gprs[15], link->fastPhase);
	pthread_mutex_lock(&link->lock);
	link->selfMulti = false;
	link->transferActive = false;
	if (link->host) {
		link->multiMask &= ~1;
		_hostBroadcastState(link);
	} else {
		link->multiMask &= ~(1 << link->id);
	}
	pthread_mutex_unlock(&link->lock);
	if (!link->host && link->peers[0].connected) {
		struct NetLinkMsg mode = { .type = MSG_MODE, .a = 0 };
		_sendTo(link, link->peers[0].fd, &mode);
	}
	if (link->host && link->fastPhase != FAST_IDLE) {
		/* Mid-speculation mode change: let the next tick restore and go exact. */
		link->abortFlag = true;
		return true;
	}
	_overrideKeys(link, false);
	link->snapshotValid = false;
	if (mTimingIsScheduled(&link->sio->p->timing, &link->event)) {
		mTimingDeschedule(&link->sio->p->timing, &link->event);
	}
	return true;
}

static void _finishMaster(struct GBASIONetLink* link) {
	struct GBASIO* sio = link->sio;
	struct GBA* gba = sio->p;
	NL_TRACE("M done seq=%u v=%04x %04x %04x %04x siocnt=%04x cyc=%u vc=%d", link->seq, link->recv[0], link->recv[1], link->recv[2], link->recv[3], sio->siocnt, mTimingCurrentTime(&gba->timing) - link->snapCycles, gba->video.vcount);
	gba->memory.io[REG_SIOMULTI0 >> 1] = link->recv[0];
	gba->memory.io[REG_SIOMULTI1 >> 1] = link->recv[1];
	gba->memory.io[REG_SIOMULTI2 >> 1] = link->recv[2];
	gba->memory.io[REG_SIOMULTI3 >> 1] = link->recv[3];
	sio->rcnt |= 1;
	sio->siocnt = GBASIOMultiplayerClearBusy(sio->siocnt);
	sio->siocnt = GBASIOMultiplayerSetId(sio->siocnt, 0);
	link->transferActive = false;
	if (link->fastPhase != FAST_SPEC) {
		++link->transfers;
		if (link->activeFrame == gba->video.frameCounter && link->activeIdx < NETLINK_FAST_MAX) {
			link->curFrameChild[link->activeIdx] = link->recv[1];
			if (link->activeIdx + 1 > link->curFrameCount) {
				link->curFrameCount = link->activeIdx + 1;
			}
		}
	}
	if (link->transfers % 1000 == 0 && link->fastPhase != FAST_SPEC) {
		NL_LOG("%u transfers, avg latency %u us, max %u us, timeouts %u, fast blocks %u, aborts %u", link->transfers, link->avgLatencyUs, link->maxLatencyUs, link->timeouts, link->fastBlocks, link->fastAborts);
	}
	if (!link->localTransfer) {
		uint64_t latencyUs = (_nowNs() - link->startedNs) / 1000;
		link->lastLatencyUs = (uint32_t) latencyUs;
		link->avgLatencyUs = link->avgLatencyUs ? (link->avgLatencyUs * 7 + (uint32_t) latencyUs) / 8 : (uint32_t) latencyUs;
		if (latencyUs > link->maxLatencyUs) {
			link->maxLatencyUs = (uint32_t) latencyUs;
		}
		struct NetLinkMsg done = { .type = MSG_XFER_DONE, .seq = link->seq };
		memcpy(done.v, link->recv, sizeof(done.v));
		_broadcast(link, &done);
	}
	link->localTransfer = false;
	if (GBASIOMultiplayerIsIrq(sio->siocnt)) {
		GBARaiseIRQ(gba, GBA_IRQ_SIO, 0);
	}
}

static uint16_t _writeRegister(struct GBASIODriver* driver, uint32_t address, uint16_t value) {
	struct GBASIONetLink* link = (struct GBASIONetLink*) driver;
	struct GBASIO* sio = driver->p;
	if (address == REG_SIOMLT_SEND) {
		pthread_mutex_lock(&link->lock);
		link->sendLatch = value;
		link->sendWrittenSinceDone = true;
		pthread_mutex_unlock(&link->lock);
		return value;
	}
	if (address != REG_SIOCNT) {
		return value;
	}
	NL_TRACE("%c siocnt<-%04x (cur %04x)", link->host ? 'M' : 'C', value, sio->siocnt);
	pthread_mutex_lock(&link->lock);
	_applyStatusBits(link);
	bool start = false;
	if (link->host && (value & 0x0080) && !link->transferActive && _ready(link)) {
		start = true;
		link->transferActive = true;
		link->localTransfer = false;
		++link->seq;
		uint16_t sendVal = sio->p->memory.io[REG_SIOMLT_SEND >> 1];
		link->recv[0] = sendVal;
		link->recv[1] = link->recv[2] = link->recv[3] = 0xFFFF;
		link->gotMask = 1;
		int i;
		for (i = 1; i < NETLINK_MAX_PLAYERS; ++i) {
			if (!link->peers[i].connected) {
				link->gotMask |= 1 << i; /* nothing to wait for */
			}
		}
		int baud = GBASIOMultiplayerGetBaud(value);
		link->cyclesLeft = GBASIOCyclesPerTransfer[baud][link->attached - 1];
		link->startedNs = _nowNs();
		sio->p->memory.io[REG_SIOMULTI0 >> 1] = 0xFFFF;
		sio->p->memory.io[REG_SIOMULTI1 >> 1] = 0xFFFF;
		sio->p->memory.io[REG_SIOMULTI2 >> 1] = 0xFFFF;
		sio->p->memory.io[REG_SIOMULTI3 >> 1] = 0xFFFF;

		link->activeIdx = link->frameIdx;
		link->activeFrame = sio->p->video.frameCounter;
		++link->frameIdx;
		if ((link->fastPhase == FAST_SPEC && link->specFrozen) || link->abortFlag) {
			/* A restore is pending (waiting for a prescaler boundary): whatever the game
			 * does now is discarded, so never let it reach the network. */
			link->recv[1] = 0;
			link->localTransfer = true;
			link->gotMask = 0x0F;
		} else if (link->fastActive && !link->exactThisFrame && link->snapshotValid && sio->p->video.frameCounter == link->snapFrame) {
			if (link->fastPhase == FAST_IDLE) {
				/* First transfer after a snapshot: start a speculation segment. Audio
				 * sampling is suspended so the speculative run does not emit sound. */
				link->fastPhase = FAST_SPEC;
				link->specCount = 0;
				link->specOverflow = false;
				/* The audio sample event keeps running (removing it changes when the CPU
				 * stops between instructions, which shifts IRQ timing); the speculative
				 * samples are dropped on restore, and the frontend is asked not to block
				 * on audio meanwhile. */
				_nudge(link);
				NL_TRACE("M spec begin frame=%u idx=%d cyc=%u", sio->p->video.frameCounter, link->activeIdx, mTimingCurrentTime(&sio->p->timing) - link->snapCycles);
			}
			if (link->fastPhase == FAST_SPEC) {
				if (link->specCount < NETLINK_FAST_MAX) {
					link->specWords[link->specCount] = sendVal;
					link->recv[1] = link->activeIdx < link->prevFrameCount ? link->prevFrameChild[link->activeIdx] : 0;
					++link->specCount;
				} else {
					link->specOverflow = true;
				}
				link->localTransfer = true;
				link->gotMask = 0x0F;
			} else if (link->fastPhase == FAST_REAL) {
				if (link->realIndex < link->specCount) {
					if (sendVal != link->specWords[link->realIndex]) {
						NL_TRACE("M real mismatch idx=%d predicted=%04x actual=%04x", link->realIndex, link->specWords[link->realIndex], sendVal);
						link->abortFlag = true;
					}
					link->recv[1] = link->childWords[link->realIndex];
					++link->realIndex;
					link->localTransfer = true;
					link->gotMask = 0x0F;
				}
				/* else: more transfers than speculated; this one goes over the
				 * network and a new segment starts once it completes. */
			}
		}
		NL_TRACE("M xstart seq=%u send=%04x local=%d cyc=%u vc=%d pc=%08x", link->seq, sendVal, link->localTransfer, mTimingCurrentTime(&sio->p->timing) - link->snapCycles, sio->p->video.vcount, sio->p->cpu->gprs[15]);
		if (!link->localTransfer) {
			struct NetLinkMsg req = { .type = MSG_XFER_REQ, .a = (uint8_t) baud, .seq = link->seq, .b = _masterFrame16(link) };
			req.v[0] = link->recv[0];
			_broadcast(link, &req);
			NL_TRACE("M start seq=%u send=%04x siocnt_w=%04x", link->seq, link->recv[0], value);
		}
	}
	pthread_mutex_unlock(&link->lock);

	/* Keep baud/irq bits from the write, status bits from our bookkeeping. */
	uint16_t result = (value & 0xFF03) | (sio->siocnt & 0x007C);
	if (start || (link->host && link->transferActive)) {
		result = GBASIOMultiplayerFillBusy(result);
	} else if (!link->host) {
		result = (result & ~0x0080) | (sio->siocnt & 0x0080); /* children cannot start transfers */
	} else {
		result = GBASIOMultiplayerClearBusy(result);
	}
	return result;
}


static void _hostRestore(struct GBASIONetLink* link, struct mTiming* timing) {
	struct GBA* gba = link->sio->p;
	_restoreSnapshot(link, timing);
	/* Discard the audio produced by the speculative pass. */
	blip_clear(gba->audio.psg.left);
	blip_clear(gba->audio.psg.right);
	_unnudge(link);
	link->transferActive = false;
	link->localTransfer = false;
	link->abortFlag = false;
	link->lastFrameSeen = gba->video.frameCounter;
	link->frameIdx = link->segStartIdx;
	gba->keysActive = link->frozenKeys;
}

/* Take a segment snapshot at a safe point (inside the link tick, no transfer in flight). */
static void _hostStartSegment(struct GBASIONetLink* link) {
	struct GBA* gba = link->sio->p;
	link->frozenKeys = _liveKeys(link);
	_overrideKeys(link, true);
	gba->keysActive = link->frozenKeys;
	_takeSnapshot(link);
	link->snapshotValid = true;
	link->snapFrame = gba->video.frameCounter;
	link->segStartIdx = link->frameIdx;
}

static void _hostCommitSegment(struct GBASIONetLink* link) {
	link->consecutiveAborts = 0;
	link->blockSent = false;
	link->fastPhase = FAST_IDLE;
	link->specHashValid = false;
}

static void _hostAbortBlock(struct GBASIONetLink* link, struct mTiming* timing, const char* why) {
	NL_LOG("Fast block %u aborted (%s): falling back to exact mode for this frame", link->blockSeq, why);
	++link->fastAborts;
	++link->consecutiveAborts;
	if (link->consecutiveAborts >= FAST_MAX_ABORTS && link->fastActive) {
		NL_LOG("Fast link disabled after %d consecutive aborts", link->consecutiveAborts);
		link->fastActive = false;
	}
	if (link->blockSent) {
		struct NetLinkMsg abortMsg = { .type = MSG_BLOCK_ABORT, .seq = link->blockSeq };
		_broadcast(link, &abortMsg);
		link->blockSent = false;
	}
	_hostRestore(link, timing);
	link->fastPhase = FAST_IDLE;
	link->exactThisFrame = true;
}

/* Debug: remember the state at the end of the speculative pass (taken at the boundary tick). */
static void _captureSpecEnd(struct GBASIONetLink* link) {
	struct GBA* gba = link->sio->p;
	struct NetLinkHash h;
	_computeHashRel(link, &h, link->specRel, &link->specRelN);
	memcpy(link->specHash, &h, sizeof(h) < sizeof(link->specHash) ? sizeof(h) : sizeof(link->specHash));
	link->specHashValid = true;
	if (!link->specIwram) {
		link->specIwram = malloc(0x8000);
		link->specIo = malloc(0x400);
	}
	memcpy(link->specIwram, gba->memory.iwram, 0x8000);
	memcpy(link->specIo, gba->memory.io, 0x400);
}

static void _hostFrameBoundary(struct GBASIONetLink* link, struct mTiming* timing) {
	struct GBA* gba = link->sio->p;
	bool wantFast = link->fastEnabled && link->attached == 2 && link->snapshot && link->consecutiveAborts < FAST_MAX_ABORTS && link->selfMulti;
	switch (link->fastPhase) {
	case FAST_SPEC: {
		/* Speculative frame done: back to the frame start and exchange words. */
		int k = link->specCount;
		bool overflow = link->specOverflow;
		_hostRestore(link, timing);
		if (overflow || k == 0) {
			link->fastPhase = FAST_IDLE;
			link->exactThisFrame = true;
			return;
		}
		++link->blockSeq;
		link->respK = 0;
		link->respRecv = 0;
		link->respComplete = false;
		int fd = -1;
		int i;
		for (i = 1; i < NETLINK_MAX_PLAYERS; ++i) {
			if (link->peers[i].connected) {
				fd = link->peers[i].fd;
			}
		}
		struct NetLinkMsg req = { .type = MSG_BLOCK_REQ, .a = (uint8_t) k, .b = _masterFrame16(link), .seq = link->blockSeq };
		_sendTo(link, fd, &req);
		_sendWords(link, fd, link->blockSeq, link->specWords, k);
		link->blockSent = true;
		NL_TRACE("M block %u sent k=%d", link->blockSeq, k);
		uint64_t started = _nowNs();
		for (;;) {
			if (link->respComplete || link->stopping) {
				break;
			}
			if (_nowNs() - started > TRANSFER_TIMEOUT_NS) {
				break;
			}
			struct timespec ts;
			clock_gettime(CLOCK_REALTIME, &ts);
			ts.tv_nsec += 5000000;
			if (ts.tv_nsec >= 1000000000) {
				ts.tv_sec += 1;
				ts.tv_nsec -= 1000000000;
			}
			pthread_cond_timedwait(&link->respCond, &link->lock, &ts);
			struct NetLinkMsg ev;
			while (_pop(link, &ev)) {
				if (ev.type == EV_STATE) {
					_applyStatusBits(link);
				}
			}
		}
		if (link->respComplete && link->respK == k && link->testAbortEvery && link->blockSeq % link->testAbortEvery == 0) {
			_hostAbortBlock(link, timing, "test hook");
		} else if (link->respComplete && link->respK == k) {
			link->fastPhase = FAST_REAL;
			link->realIndex = 0;
			++link->fastBlocks;
			NL_TRACE("M block %u response ok, replaying", link->blockSeq);
		} else {
			_hostAbortBlock(link, timing, link->respComplete ? "length mismatch" : "no block answer");
		}
		return;
	}
	case FAST_REAL:
		if (link->realIndex != link->specCount) {
			_hostAbortBlock(link, timing, "transfer count differs from speculation");
			return;
		}
		if (_traceOn() && link->specHashValid) {
			struct NetLinkHash h;
			struct NetLinkHash sh;
			_computeHash(link, &h);
			memcpy(&sh, link->specHash, sizeof(sh));
			if (memcmp(&h, &sh, sizeof(h))) {
				_printHash(link, "spec-end", &sh);
				_printHash(link, "real-end", &h);
				int32_t rel[24];
				int n = 0;
				_computeHashRel(link, &h, rel, &n);
				int i;
				for (i = 0; i < n && i < link->specRelN; ++i) {
					if (rel[i] != link->specRel[i]) {
						NL_TRACE("M timdiff idx=%d spec=%d real=%d", i, link->specRel[i], rel[i]);
					}
				}
				int shown = 0;
				const uint32_t* a = (const uint32_t*) link->specIwram;
				const uint32_t* b = (const uint32_t*) gba->memory.iwram;
				for (i = 0; i < 0x2000 && shown < 12; ++i) {
					if (a[i] != b[i]) {
						NL_TRACE("M iwdiff off=%05x spec=%08x real=%08x", i * 4, a[i], b[i]);
						++shown;
					}
				}
				const uint16_t* ia = (const uint16_t*) link->specIo;
				const uint16_t* ib = (const uint16_t*) gba->memory.io;
				for (i = 0, shown = 0; i < 0x200 && shown < 8; ++i) {
					if (ia[i] != ib[i]) {
						NL_TRACE("M iodiff reg=%03x spec=%04x real=%04x", i * 2, ia[i], ib[i]);
						++shown;
					}
				}
			}
			link->specHashValid = false;
		}
		_hostCommitSegment(link);
		/* fall through: take the snapshot for the next frame */
		// fall through
	default:
		if ((gba->video.frameCounter & 127) == 0) {
			s_trace = -1; /* re-read NETLINK_TRACE / debug.cartucho.nltrace while running */
		}
		link->reportFrame = gba->video.frameCounter;
		memcpy(link->prevFrameChild, link->curFrameChild, sizeof(link->prevFrameChild));
		link->prevFrameCount = link->curFrameCount;
		link->curFrameCount = 0;
		link->frameIdx = 0;
		link->exactThisFrame = false;
		link->fastActive = wantFast;
		if (wantFast && !link->transferActive) {
			_hostStartSegment(link);
		} else {
			link->snapshotValid = false;
			_overrideKeys(link, false);
		}
		break;
	}
}

static void _childApplyDone(struct GBASIONetLink* link, struct mTiming* timing, struct GBASIO* sio, struct GBA* gba, const uint16_t* v, uint32_t seq) {
	gba->memory.io[REG_SIOMULTI0 >> 1] = v[0];
	gba->memory.io[REG_SIOMULTI1 >> 1] = v[1];
	gba->memory.io[REG_SIOMULTI2 >> 1] = v[2];
	gba->memory.io[REG_SIOMULTI3 >> 1] = v[3];
	sio->rcnt |= 1;
	sio->siocnt = GBASIOMultiplayerClearBusy(sio->siocnt);
	sio->siocnt = GBASIOMultiplayerSetId(sio->siocnt, link->id);
	++link->transfers;
	link->lastDoneAt = mTimingCurrentTime(timing);
	link->haveDone = true;
	link->sendWrittenSinceDone = false;
	NL_TRACE("C done seq=%u v=%04x %04x %04x %04x siocnt=%04x", seq, v[0], v[1], v[2], v[3], sio->siocnt);
	if (GBASIOMultiplayerIsIrq(sio->siocnt)) {
		GBARaiseIRQ(gba, GBA_IRQ_SIO, 0);
	}
}

static bool _childDeferredRestore(struct GBASIONetLink* link, struct mTiming* timing, int32_t* nextTickIn) {
	if (!link->childRestorePending) {
		return false;
	}
	int32_t wait = _restoreWait(timing, link->snapMaster);
	if (wait > 0 && link->restoreTries < RESTORE_MAX_TRIES) {
		++link->restoreTries;
		*nextTickIn = _scheduleDelay(timing, wait);
		return true;
	}
	link->restoreTries = 0;
	_restoreSnapshot(link, timing);
	link->childSnapshotValid = false;
	link->blockActive = false;
	link->reqPending = false;
	/* Back to exactly the reply-gating state the game was in at the snapshot: the
	 * next answer must wait for the game to load SIOMLT_SEND, as it would have. */
	link->sendWrittenSinceDone = link->snapSendWritten;
	link->haveDone = link->snapHaveDone;
	link->lastDoneAt = mTimingCurrentTime(timing) - link->snapSinceDone;
	link->childRestorePending = false;
	NL_TRACE("C block %u aborted: restored", link->childRestoreSeq);
	return false;
}

static void _childApplyEvents(struct GBASIONetLink* link, struct mTiming* timing, struct GBASIO* sio, struct GBA* gba) {
	struct NetLinkMsg ev;
	while (_pop(link, &ev)) {
		switch (ev.type) {
		case EV_STATE:
			_applyStatusBits(link);
			break;
		case EV_XFER_START:
			link->reqPending = true;
			link->reqSeq = ev.seq;
			break;
		case EV_XFER_DONE:
			_childApplyDone(link, timing, sio, gba, ev.v, ev.seq);
			break;
		case EV_BLOCK_START:
			if (link->snapshot && !link->blockActive && ev.seq == link->blockReqSeq) {
				_takeSnapshot(link);
				link->snapSendWritten = link->sendWrittenSinceDone;
				link->snapHaveDone = link->haveDone;
				link->snapSinceDone = link->haveDone ? (int32_t) (mTimingCurrentTime(timing) - link->lastDoneAt) : 0;
				link->childSnapshotValid = true;
				link->blockActive = true;
				link->blockIndex = 0;
				link->reqPending = true;
				link->reqSeq = ev.seq;
				NL_TRACE("C block %u start k=%d", ev.seq, link->blockK);
			}
			break;
		case EV_BLOCK_ABORT:
			if (link->childSnapshotValid && ev.seq == link->blockReqSeq) {
				/* Restore on a prescaler boundary; later events stay queued until then. */
				link->childRestorePending = true;
				link->childRestoreSeq = ev.seq;
				return;
			}
			break;
		default:
			break;
		}
	}
}

static void _childReply(struct GBASIONetLink* link, struct mTiming* timing) {
	if (!link->reqPending) {
		return;
	}
	struct GBA* gba = link->sio->p;
	int32_t since = link->haveDone ? mTimingCurrentTime(timing) - link->lastDoneAt : CHILD_REPLY_FALLBACK;
	if (!link->haveDone || link->sendWrittenSinceDone || since >= CHILD_REPLY_FALLBACK) {
		struct NetLinkMsg resp = { .type = MSG_XFER_RESP, .a = (uint8_t) link->id, .seq = link->reqSeq };
		resp.v[0] = gba->memory.io[REG_SIOMLT_SEND >> 1];
		gba->memory.io[REG_SIOMULTI0 >> 1] = 0xFFFF;
		gba->memory.io[REG_SIOMULTI1 >> 1] = 0xFFFF;
		gba->memory.io[REG_SIOMULTI2 >> 1] = 0xFFFF;
		gba->memory.io[REG_SIOMULTI3 >> 1] = 0xFFFF;
		link->sio->siocnt = GBASIOMultiplayerFillBusy(link->sio->siocnt);
		link->reqPending = false;
		if (!link->lockstepRef) {
			link->lockstepRef = true;
			link->refMasterFrame = link->masterFrame;
			link->refChildFrame = gba->video.frameCounter;
		}
		if (link->blockActive) {
			/* Fast link: complete the transfer locally with the master's word. */
			int i = link->blockIndex;
			link->blockChild[i] = resp.v[0];
			uint16_t v[4] = { link->blockMaster[i], resp.v[0], 0xFFFF, 0xFFFF };
			_childApplyDone(link, timing, link->sio, gba, v, link->reqSeq);
			++link->blockIndex;
			if (link->blockIndex < link->blockK) {
				link->reqPending = true;
			} else {
				link->blockActive = false;
				struct NetLinkMsg hdr = { .type = MSG_BLOCK_RESP, .a = (uint8_t) link->blockK, .seq = link->reqSeq };
				int fd = link->peers[0].connected ? link->peers[0].fd : -1;
				NL_TRACE("C block %u done, answering k=%d", link->reqSeq, link->blockK);
				pthread_mutex_unlock(&link->lock);
				_sendTo(link, fd, &hdr);
				_sendWords(link, fd, link->reqSeq, link->blockChild, link->blockK);
				pthread_mutex_lock(&link->lock);
			}
			return;
		}
		NL_TRACE("C resp seq=%u send=%04x since=%d written=%d", link->reqSeq, resp.v[0], since, link->sendWrittenSinceDone);
		int fd = link->peers[0].connected ? link->peers[0].fd : -1;
		pthread_mutex_unlock(&link->lock);
		_sendTo(link, fd, &resp);
		pthread_mutex_lock(&link->lock);
	}
}

static void _tick(struct mTiming* timing, void* context, uint32_t cyclesLate) {
	struct GBASIONetLink* link = context;
	struct GBASIO* sio = link->sio;
	struct GBA* gba = sio->p;
	struct NetLinkMsg ev;

	pthread_mutex_lock(&link->lock);
	uint32_t nowCycles = mTimingCurrentTime(timing);
	int32_t elapsed = (int32_t) (nowCycles - link->lastTickAt);
	if (elapsed < 0 || elapsed > POLL_PERIOD * 4) {
		elapsed = POLL_PERIOD + cyclesLate;
	}
	link->lastTickAt = nowCycles;
	_debugTick(link, nowCycles);
	int32_t childTickIn = _latticeWait(nowCycles);
	if (!link->host) {
		if (_childDeferredRestore(link, timing, &childTickIn)) {
			pthread_mutex_unlock(&link->lock);
			if (!mTimingIsScheduled(timing, &link->event)) {
				mTimingSchedule(timing, &link->event, childTickIn);
			}
			return;
		}
		_childApplyEvents(link, timing, sio, gba);
		if (link->childRestorePending && _childDeferredRestore(link, timing, &childTickIn)) {
			pthread_mutex_unlock(&link->lock);
			if (!mTimingIsScheduled(timing, &link->event)) {
				mTimingSchedule(timing, &link->event, childTickIn);
			}
			return;
		}
	}
	while (link->host && _pop(link, &ev)) {
		switch (ev.type) {
		case EV_STATE:
			_applyStatusBits(link);
			break;
		case EV_XFER_RESP:
			if (link->host && link->transferActive && ev.seq == link->seq && ev.a < NETLINK_MAX_PLAYERS) {
				link->recv[ev.a] = ev.v[0];
				link->gotMask |= 1 << ev.a;
			}
			break;
		case EV_XFER_START:
			if (!link->host) {
				/* Registers are left untouched until we answer: the game may
				 * still have to read the previous result in its IRQ handler. */
				link->reqPending = true;
				link->reqSeq = ev.seq;
			}
			break;
		case EV_XFER_DONE:
			if (!link->host) {
				gba->memory.io[REG_SIOMULTI0 >> 1] = ev.v[0];
				gba->memory.io[REG_SIOMULTI1 >> 1] = ev.v[1];
				gba->memory.io[REG_SIOMULTI2 >> 1] = ev.v[2];
				gba->memory.io[REG_SIOMULTI3 >> 1] = ev.v[3];
				sio->rcnt |= 1;
				sio->siocnt = GBASIOMultiplayerClearBusy(sio->siocnt);
				sio->siocnt = GBASIOMultiplayerSetId(sio->siocnt, link->id);
				++link->transfers;
				link->lastDoneAt = mTimingCurrentTime(timing);
				link->haveDone = true;
				link->sendWrittenSinceDone = false;
				NL_TRACE("C done seq=%u v=%04x %04x %04x %04x siocnt=%04x", ev.seq, ev.v[0], ev.v[1], ev.v[2], ev.v[3], sio->siocnt);
				if (GBASIOMultiplayerIsIrq(sio->siocnt)) {
					GBARaiseIRQ(gba, GBA_IRQ_SIO, 0);
				}
			}
			break;
		default:
			break;
		}
	}

	if (!link->host) {
		_childReply(link, timing);
		/* Frame lockstep: never run more than one frame ahead of the master
		 * (both games must advance together, as two consoles would). */
		while (!link->reqPending && link->lockstepRef && link->connected && !link->stopping
		       && _nowNs() - link->lastDoneWallNs < CHILD_ACTIVE_WINDOW_NS
		       && (int32_t) ((gba->video.frameCounter - link->refChildFrame) - (link->masterFrame - link->refMasterFrame)) > CHILD_MAX_AHEAD_FRAMES) {
			struct timespec ts;
			clock_gettime(CLOCK_REALTIME, &ts);
			ts.tv_nsec += 5000000;
			if (ts.tv_nsec >= 1000000000) {
				ts.tv_sec += 1;
				ts.tv_nsec -= 1000000000;
			}
			pthread_cond_timedwait(&link->respCond, &link->lock, &ts);
			_childApplyEvents(link, timing, sio, gba);
			if (link->childRestorePending) {
				break;
			}
			_childReply(link, timing);
		}
		_unnudge(link);
	}

	int32_t nextTickIn = _latticeWait(nowCycles);
	if (link->host) {
		uint32_t frame = gba->video.frameCounter;
		if (frame != link->lastFrameSeen) {
			link->lastFrameSeen = frame;
			link->boundaryPending = true;
			if (link->fastPhase == FAST_SPEC && _traceOn()) {
				_captureSpecEnd(link);
			}
		}
		if (link->boundaryPending || link->abortFlag) {
			bool needRestore = link->abortFlag || (link->boundaryPending && link->fastPhase == FAST_SPEC);
			int32_t wait = (needRestore && link->snapshotValid) ? _restoreWait(timing, link->snapMaster) : 0;
			if (wait > 0 && link->restoreTries < RESTORE_MAX_TRIES) {
				/* Not on a prescaler boundary yet (the CPU overshot the tick): keep running,
				 * results are discarded, and try again at the next aligned instant. */
				link->specFrozen = true;
				++link->restoreTries;
				nextTickIn = _scheduleDelay(timing, wait);
			} else {
				if (wait > 0) {
					NL_TRACE("M restore forced off-phase by %d cycles", RESTORE_ALIGN - wait);
				}
				link->restoreTries = 0;
				if (link->boundaryPending) {
					link->boundaryPending = false;
					_hostFrameBoundary(link, timing);
				}
				if (link->abortFlag) {
					_hostAbortBlock(link, timing, "word mismatch");
				}
				link->specFrozen = false;
			}
		}
		if (link->fastPhase == FAST_IDLE && !link->snapshotValid) {
			link->reportFrame = gba->video.frameCounter;
		}
		if (link->keysTracking) {
			if (gba->keysActive != link->frozenKeys) {
				link->pendingKeys = gba->keysActive;
				gba->keysActive = link->frozenKeys;
			}
		}
	}

	if (link->host && link->transferActive) {
		link->cyclesLeft -= elapsed;
		if (!link->localTransfer) {
			/* Exact mode: stall the emulated clock until every connected child
			 * has answered (or the timeout expires). */
			for (;;) {
				bool complete = true;
				int i;
				for (i = 1; i < NETLINK_MAX_PLAYERS; ++i) {
					if (link->peers[i].connected && !(link->gotMask & (1 << i))) {
						complete = false;
					}
				}
				if (complete || link->stopping) {
					break;
				}
				uint64_t waited = _nowNs() - link->startedNs;
				if (waited > TRANSFER_TIMEOUT_NS) {
					++link->timeouts;
					NL_LOG("Transfer %u: no answer after %llu ms (timeouts so far: %u)", link->seq,
					       (unsigned long long) (waited / 1000000ULL), link->timeouts);
					break;
				}
				struct timespec ts;
				clock_gettime(CLOCK_REALTIME, &ts);
				ts.tv_nsec += 5000000;
				if (ts.tv_nsec >= 1000000000) {
					ts.tv_sec += 1;
					ts.tv_nsec -= 1000000000;
				}
				pthread_cond_timedwait(&link->respCond, &link->lock, &ts);
				while (_pop(link, &ev)) {
					if (ev.type == EV_XFER_RESP && ev.seq == link->seq && ev.a < NETLINK_MAX_PLAYERS) {
						link->recv[ev.a] = ev.v[0];
						link->gotMask |= 1 << ev.a;
					} else if (ev.type == EV_STATE) {
						_applyStatusBits(link);
					}
				}
			}
		}
		if (link->transferActive && link->cyclesLeft <= 0) {
			bool wasLocal = link->localTransfer;
			_finishMaster(link);
			if (!wasLocal && link->fastActive && !link->exactThisFrame && link->snapshot
			    && (link->fastPhase == FAST_IDLE || (link->fastPhase == FAST_REAL && link->realIndex >= link->specCount))) {
				/* The game asked for more transfers than predicted: the replayed
				 * segment is complete, so speculate again from this point. */
				if (link->fastPhase == FAST_REAL) {
					_hostCommitSegment(link);
				}
				_hostStartSegment(link);
				NL_TRACE("M segment restart frame=%u idx=%d", gba->video.frameCounter, link->frameIdx);
			}
		}
	}
	pthread_mutex_unlock(&link->lock);

	if (!mTimingIsScheduled(timing, &link->event)) {
		mTimingSchedule(timing, &link->event, nextTickIn);
	}
}

void GBASIONetLinkStats(struct GBASIONetLink* link, uint32_t* timeouts, uint32_t* avgLatencyUs, uint32_t* maxLatencyUs) {
	pthread_mutex_lock(&link->lock);
	if (timeouts) *timeouts = link->timeouts;
	if (avgLatencyUs) *avgLatencyUs = link->avgLatencyUs;
	if (maxLatencyUs) *maxLatencyUs = link->maxLatencyUs;
	pthread_mutex_unlock(&link->lock);
}
