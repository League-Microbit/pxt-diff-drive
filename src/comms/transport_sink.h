// transport_sink.h -- the ONE outbound seam between the v6 wire stack
// and a transport: `wireLineContentLength()` (how much of a written
// line is content) plus `TransportSink`, the single Wire::Sink every
// transport is reached through.
//
// One class, not one per transport: a Sink's whole body is "decide how
// many of these bytes are content, then hand them to the transport,
// which appends its own delimiter", and that is identical for serial,
// radio and WiFi. Three hand-copied Sinks were three chances for the
// terminator half to drift -- exactly the off-by-one no on-target test
// can see. Host-portable by construction (no pxt.h, no CODAL, only
// wire_handler.h's Sink interface and <cstddef>), so
// tests/host/test_transport_sink.py drives the real code.
//
// The transport stays a TEMPLATE PARAMETER, never an #include:
// src/DESIGN.md S1's layering table puts the transports below the wire
// grammar, so a header naming both concretely would tie them together
// and drag pxt.h in with them. Protocol supplies the pairing.
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
// It drops that byte only after CHECKING for it, and the distinction is
// not theoretical: a line arriving WITHOUT its terminator (a formatter
// that ran out of buffer before appending one) would otherwise lose its
// last real byte -- a plausible wrong number instead of a visibly
// truncated one. A '\r' immediately before the '\n' is left in the
// content deliberately, so a CRLF-terminated line goes out with the
// bytes it came in with.
//
// Total-function by construction: `length == 0` returns 0 rather than
// underflowing, and a null `data` is treated as an empty line.
inline size_t wireLineContentLength(const char* data, size_t length) {
  if (data == nullptr || length == 0) return 0;
  return data[length - 1] == '\n' ? length - 1 : length;
}

// One Sink for every transport. `Write` is a plain function pointer, so
// the transports' differing write signatures (void writeLine(), bool
// sendLine()) are adapted by a one-line function each at the
// composition site rather than by near-identical Sink classes here.
//
// Single writer: the protocol fiber. Every call arrives from
// Wire::WireHandler, driven only from Protocol::serviceOnce(), at any
// nesting depth; a line from another fiber (Protocol::emitLine()) takes
// the emit ring instead and is written out by that same fiber. Nothing
// here serializes concurrent writers because there are none.
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
