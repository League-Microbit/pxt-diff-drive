// radio_transport.cpp -- see radio_transport.h. Talks to uBit.radio
// directly, the same way serial_transport.cpp talks to uBit.serial and
// nezha_port.cpp talks to uBit.i2c: one small CODAL-facing leaf, no
// shaping/porting layers of its own.
//
// sendFragmented() below is a provenance-documented port of
// radio-robot's Platform::MicroBitRadioLink::sendFragmented() -- the
// fleet's own reference driver the RADIOBRIDGE relay hardware is built
// against (src/DESIGN.md §2 has the authoritative upstream repo/path
// statement). See radio_transport.h's top comment for the framing this
// implements and its RX/TX/ACK scope.
#include "radio_transport.h"

#include <cstring>

#include "pxt.h"

using namespace pxt;

namespace diffDrive {

namespace {
constexpr uint8_t kLineDelimiter = 0x0A;
RadioTransport* gRadioRx = nullptr;  // MessageBus trampoline target
void radioDatagramTrampoline(MicroBitEvent) {
  if (gRadioRx != nullptr) gRadioRx->onDatagram();
}
}  // namespace

void RadioTransport::ensureRadioReady() {
  if (radioReady_) return;
  radioReady_ = true;
  // Call order mirrors the reference driver's own begin()
  // (microbit_radio_link.cpp): enable, then frequency band, then
  // group, then transmit power. CODAL does not default to band 0 --
  // it must be set explicitly, or a robot and the relay could sit on
  // different frequencies and never hear each other.
  uBit.radio.enable();
  // channel_, not kChannel: the constant is only the default this field
  // was initialised from (see its declaration), so a setChannel() call
  // made before the radio came up is honoured here.
  uBit.radio.setFrequencyBand(channel_);
  uBit.radio.setGroup(group_);
  uBit.radio.setTransmitPower(kTransmitPower);
  // Reference-style RX: listen for the datagram event and recv() only
  // there (see header comment on onDatagram).
  gRadioRx = this;
  uBit.messageBus.listen(MICROBIT_ID_RADIO, MICROBIT_RADIO_EVT_DATAGRAM,
                         radioDatagramTrampoline);
}

void RadioTransport::setChannel(uint8_t channel) {
  channel_ = channel;
  if (radioReady_) {
    // Radio already up. UNVERIFIED whether this actually re-tunes live
    // hardware -- see this method's own doc comment (radio_transport.h)
    // for the source reading behind the doubt and what would settle it.
    // Protocol::setupRadio() never reaches this branch: it configures
    // before enabling.
    uBit.radio.setFrequencyBand(channel_);
  }
}

void RadioTransport::setGroup(uint8_t group) {
  group_ = group;
  if (radioReady_) {
    // Radio already up (a prior sendLine()/tryReceiveLine() already
    // lazily called ensureRadioReady()): re-apply immediately so this
    // call takes effect without waiting for anything else to happen.
    // If the radio has NOT come up yet, group_ is simply stored above --
    // ensureRadioReady() reads it (not a hardcoded constant) the first
    // time it actually runs. See this method's own doc comment
    // (radio_transport.h) for why uBit.radio.enable() is never called
    // from here.
    uBit.radio.setGroup(group_);
  }
}

void RadioTransport::onDatagram() {
  PacketBuffer p = uBit.radio.datagram.recv();
  const int plen = p.length();
  if (plen < static_cast<int>(kFrameHeaderBytes)) return;
  const uint8_t* d = p.getBytes();
  const uint8_t flags = d[1];
  if (!(flags & kFlagStart) || !(flags & kFlagEnd)) return;  // multi-frag: drop
  size_t len = d[2];
  if (static_cast<int>(kFrameHeaderBytes + len) > plen) return;
  if (len > 0 && d[kFrameHeaderBytes + len - 1] == kLineDelimiter) --len;
  if (!radioRxLineFits(len, sizeof(rxLine_))) {
    // Over-length: REJECT the whole frame -- never truncate it to a
    // shorter, still-parseable prefix and deliver that prefix as if it
    // were the complete line (radioRxLineFits()'s own doc comment,
    // radio_transport.h, explains why truncate-and-accept was the
    // actual hazard). rxReady_/rxLine_ are left untouched, exactly as
    // an already-dropped MORE-flagged fragment above leaves them.
    ++rxOversizeDropped_;
    return;
  }
  if (rxReady_) return;  // previous line unconsumed: drop (reference behavior)
  if (len > 0) memcpy(rxLine_, d + kFrameHeaderBytes, len);
  rxLen_ = len;
  rxReady_ = true;
}

void RadioTransport::sendFragmented(const uint8_t* payload,
                                    size_t payloadLen) {
  // MICROBIT_RADIO_MAX_PACKET_SIZE is whatever this build's CODAL
  // target actually compiles with -- see this file's header comment
  // and sprint.md Open Question 1. Computed locally (not as a
  // class-level constant) so this translation unit has no static
  // initialization-order dependency on the macro.
  constexpr int kMaxFrame = MICROBIT_RADIO_MAX_PACKET_SIZE;
  constexpr int kMtu = kMaxFrame - kFrameHeaderBytes;
  static_assert(kMtu > 0,
               "MICROBIT_RADIO_MAX_PACKET_SIZE too small to hold the "
               "[SEQ][FLAGS][LEN] fragment header plus any payload");

  size_t off = 0;
  bool first = true;
  uint8_t* frame = frameBuf_;  // member scratch -- see header comment
  static_assert(sizeof(frameBuf_) >= kFrameHeaderBytes + kMtu,
                "frameBuf_ must hold one full on-air frame");

  do {
    size_t chunk = payloadLen - off;
    if (chunk > static_cast<size_t>(kMtu)) chunk = static_cast<size_t>(kMtu);

    uint8_t flags = 0;
    if (first) flags |= kFlagStart;
    if (off + chunk < payloadLen) {
      flags |= kFlagMore;
    } else {
      flags |= kFlagEnd;
    }

    frame[0] = txSeq_++;
    frame[1] = flags;
    frame[2] = static_cast<uint8_t>(chunk);
    if (chunk > 0) {
      memcpy(frame + kFrameHeaderBytes, payload + off, chunk);
    }
    uBit.radio.datagram.send(frame,
                             static_cast<int>(kFrameHeaderBytes + chunk));

    off += chunk;
    first = false;
  } while (off < payloadLen);
}

bool RadioTransport::tryReceiveLine(uint8_t* outBuf, size_t outCap,
                                    size_t* outLen) {
  ensureRadioReady();
  if (!rxReady_) return false;
  size_t len = rxLen_;
  if (len > outCap) len = outCap;
  if (len > 0) memcpy(outBuf, rxLine_, len);
  *outLen = len;
  rxReady_ = false;
  return true;
}

bool RadioTransport::sendLine(const uint8_t* data, size_t len) {
  ensureRadioReady();

  // Re-entrancy guard (sprint 004 ticket 002): payloadBuf_/frameBuf_
  // are shared scratch now reached by two fibers (see header comment).
  // A caller that finds sending_ already true returns immediately,
  // WITHOUT touching either buffer -- the in-flight caller owns them
  // until it clears sending_ on its own way out below.
  if (sending_) return false;
  sending_ = true;

  // `data`/`len` plus one trailing '\n' delimiter -- the ONE
  // terminator every outbound line uses here, exactly as
  // SerialTransport::writeLine() appends for the serial side. Truncates
  // rather than overflows on an over-length caller, mirroring
  // SerialTransport's own defensive truncation.
  uint8_t* payload = payloadBuf_;  // member scratch -- see header comment
  size_t n = (len < sizeof(payloadBuf_) - 1) ? len : sizeof(payloadBuf_) - 1;
  if (n > 0) {
    memcpy(payload, data, n);
  }
  payload[n] = kLineDelimiter;
  sendFragmented(payload, n + 1);

  sending_ = false;
  return true;
}

}  // namespace diffDrive
