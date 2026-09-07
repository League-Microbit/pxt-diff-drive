// run_registry.cpp -- the one shared RunRegistry instance.
//
// The table itself is a header-only template (run_registry.h), so a
// host test can stand one up with no firmware at all. This file exists
// only to give the FIRMWARE's shared instance a single definition and
// a single translation unit -- the writer (shims.cpp's registerRunName,
// called from a block program's onRun()) and the reader
// (wire_adapter.cpp's runName()/runSignature(), called from the
// protocol fiber) are in different files and must see the same table.
//
// A function-local static, not a namespace-scope object, and that is
// the whole reason this accessor is a function: registration arrives
// from a block program's TOP-LEVEL code, which on this target can run
// before another translation unit's namespace-scope constructors have
// (run.ts documents the same hazard from the TypeScript side, where an
// initialiser that DID run would wipe the handlers just registered).
// A function-local static is constructed on the first call, so the
// first registerRunName() cannot land on an unconstructed table.
#include "run_registry.h"

namespace diffDrive {

RunRegistry<>& runRegistry() {
  static RunRegistry<> registry;
  return registry;
}

}  // namespace diffDrive
