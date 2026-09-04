// fiber_identity.h -- the tick service hook's fiber-identity gate.
//
// tickDrive() (src/shims.cpp) fires its registered service hook on
// EVERY call, from whatever fiber happens to call it -- a dispatched
// job's own tick loop on the protocol fiber, a wire motion obligation
// on the protocol fiber, or a MessageBus button-handler fiber calling
// tickDrive() directly. Before this fix the hook gated on a piece of
// STATE (whether a job was believed to be running), not on which fiber
// was actually calling -- so a second fiber's tickDrive() call during a
// live job satisfied that state check and ran the wire dispatcher a
// second time, concurrently, corrupting its shared line buffer mid-
// yield.
//
// The fix compares fiber IDENTITY instead: the hook may only ever run
// on the one fiber that is Protocol's own -- the fiber Protocol::run()
// itself executes on. This function is that comparison, extracted into
// a host-portable pure function (no pxt.h, no CODAL type -- both ids
// are opaque pointers, compared for identity only, never dereferenced)
// so tests/host/test_fiber_identity_gate.py can pin fake ids on both
// sides without a real fiber scheduler. The production caller
// (comms/protocol.cpp's own Protocol::serviceHookEntry(), which cannot
// itself be compiled host-side) supplies the real ids: the protocol
// fiber's own identity, captured once when its fiber body starts, and
// an injectable "current fiber" reader defaulting to a real CODAL
// global read.
#pragma once

namespace diffDrive {

// True iff `currentFiberId` is the SAME fiber `protocolFiberId` names.
// `protocolFiberId == nullptr` (the identity has not been captured yet)
// always answers false -- there is no protocol fiber to match before
// its own body has started.
inline bool shouldServiceHookRun(const void* protocolFiberId,
                                 const void* currentFiberId) {
  return protocolFiberId != nullptr && currentFiberId == protocolFiberId;
}

}  // namespace diffDrive
