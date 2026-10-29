# Network link cable for mGBA (branch `netlink`)

This branch adds GBA multiplayer (SIO MULTI mode, 2–4 players) over the local
network to mGBA 0.10.5. One instance hosts (it becomes player 1 / master) and
the others join with its IP. Code: `src/gba/sio/netlink.c`,
`include/mgba/internal/gba/sio/netlink.h`, Qt dialog `src/platform/qt/NetLinkView.*`
(**Emulation → Network link cable…**).

## How it works

* **Exact mode** (any number of players): each transfer started by the master is
  sent to the children over TCP; the master stalls its *emulated* clock until
  every child answers, so games see hardware timing (Pokémon chains 9 transfers
  per frame and reports `LAG_MASTER` otherwise). Children answer from the core
  thread only once their game has serviced the previous serial IRQ, and keep in
  frame lockstep with the master (frame counter in every request / keepalive).
  Cost: one network round trip per transfer (~30 % speed on Wi-Fi for Pokémon).
* **Fast mode** (two players, default, exact mode as fallback): at each frame
  boundary the master snapshots the whole state (`GBASerialize` + save memory +
  SIO registers, keys frozen for the frame), runs the frame speculatively with
  placeholder child words, restores the snapshot, sends the block of K words in
  one round trip, the child executes the K transfers locally and answers its K
  words, and the master replays the frame with the real words, verifying that
  its own words match. If the game asks for more transfers than predicted the
  next one goes exact and a new speculation segment starts; on a mismatch the
  block is aborted and the frame finishes in exact mode. One or two round trips
  per frame instead of nine. Verified with complete Pokémon FireRed trades
  (8 200 blocks, 0 aborts) and with forced aborts (`NETLINK_TEST_ABORT_EVERY=7`).

Notes for anyone porting this: `GBAIOSerialize` does not store SIOMLT_SEND nor
the status bits kept in `sio->siocnt/rcnt`; save memory is not part of the
state; `mTimingTick` caches `masterCycles`, so a state restored inside a timing
callback is rebased onto the current clock; and the frame number advertised to
children must be the committed one, not the live counter.

## Protocol

16-byte messages (`NetLinkMsg {type, a, b, seq, v[4]}`) over TCP with
`TCP_NODELAY`: HELLO/WELCOME, STATE, MODE, XFER_REQ/RESP/DONE (exact mode),
PING (keepalive every 20 ms, carries the master frame), BLOCK_REQ/DATA/RESP/ABORT
(fast mode). Default port 29730.

## Debugging

`NETLINK_TRACE=1` prints every transfer, snapshot/restore fingerprints and the
speculative-vs-real state hashes to stderr (Android: property
`debug.cartucho.nltrace`).

## Licence

Same as mGBA: Mozilla Public License 2.0.
