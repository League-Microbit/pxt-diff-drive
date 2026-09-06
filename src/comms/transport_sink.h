// transport_sink.h -- the ONE outbound seam between the v6 wire stack
// and a transport: `wireLineContentLength()` (how much of a written
// line is content) plus `TransportSink`, the single Wire::Sink every
// transport is reached through.
//
// Why one class instead of one per transport: a Sink's whole body is
// "decide how many of these bytes are content, then hand them to the
// transport, which appends its own delimiter". That decision is the
// same for serial, radio and WiFi -- three hand-copied Sinks were three
// chances for it to drift, and the terminator half of it is exactly the
// kind of off-by-one no on-target test can see. Here it is one function,
// host-portable by construction (no pxt.h, no CODAL -- only
// wire_handler.h's Sink interface and <cstddef>), so
// tests/host/test_transport_sink.py drives the real code rather than a
// re-implementation of it.
//
// The transport itself stays a template parameter, never an #include:
// src/DESIGN.md S1's layering table puts the transports below the wire
// grammar, and a header naming both concretely would tie the two
// together and drag pxt.h in with them. Protocol supplies the pairing
// (its own writer functions, protocol.h), which is where knowing about
// both already belongs.
#pragma once

#include <cstddef>
#include <cstdint>

#include "wire_handler.h"

namespace diffDrive {

// How many of `length` bytes at `data` are the line's CONTENT -- i.e.
// `length` minus one trailing '\n', if there is one.
//
// WireHandler::writeLine() terminates every line it writes, and every
// transport appends its OWN single delimiter, so passing the handler's
// byte straight through would double it. That is why a sink drops one
// byte here at all.
//
// It drops that byte only after CHECKING for it. The distinction is not
// theoretical: a line that arrives WITHOUT its terminator (a frame
// whose formatter ran out of buffer before it could append one) would
// otherwise lose its last real byte instead -- a plausible, wrong
// number rather than a visibly truncated one. A '\r' immediately before
// the '\n' is left in the content deliberately: the transport re-appends
// exactly one '\n', so a CRLF-terminated line goes out on the wire with
// the same bytes it came in with.
//
// Total-function by construction: `length == 0` returns 0 rather than
// underflowing, and a null `data` is treated as an empty line.
inline size_t wireLineContentLength(const char* data, size_t length) {
  if (data == nullptr || length == 0) return 0;
  return data[length - 1] == '\n' ? length - 1 : length;
}

// One Sink for every transport. `Write` is a plain function pointer
// rather than a virtual method or a member-pointer template argument so
// the three transports' differing write signatures (void writeLine(),
// bool sendLine()) are adapted by a one-line function each at the
// composition site instead of by three near-identical Sink classes
// here.
//
// Single writer: the protocol fiber. Every call into this sink arrives
// from Wire::WireHandler, which is only ever driven from
// Protocol::serviceOnce() -- on Protocol's own fiber, at any nesting
// depth. A caller-supplied line from another fiber (Protocol::emitLine())
// takes the emit ring instead and is written out by that same fiber.
// Nothing here serializes concurrent writers because there are none.
template <typename Transport>
class TransportSink : public Wire::Sink {
 public:
  using WriteFn = void (*)(Transport&, const uint8_t*, size_t);

  TransportSink(Transport& transport, WriteFn write)
      : transport_(transport), write_(write) {}

  void write(const char* data, size_t length) override {
    write_(transport_, reinterpret_cast<const uint8_t*>(data),
           wireLineContentLength(data, length));
  }

 private:
  Transport& transport_;
  WriteFn write_;
};

}  // namespace diffDrive
