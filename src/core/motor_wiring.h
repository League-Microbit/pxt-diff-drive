// motor_wiring.h -- the decision behind `configure motor`: given what the
// two wheels are plugged into now, and what the caller just asked for,
// what should the PAIR become?
//
// Its own header, rather than logic inside shims.cpp's configureMotor(),
// for the reason encoder_glitch_armor.h was extracted: shims.cpp includes
// pxt.h and cannot be host-compiled, so anything living there is reachable
// only by flashing a board. This file is host-portable and host-tested
// directly (tests/host/test_motor_wiring.py).
//
// That is not a hypothetical. The first version of configureMotor()
// REFUSED any request naming the port the other wheel was on, to keep the
// pair from ever sharing one -- which silently rejected exactly the call a
// swap is made of. With the shipped defaults (left M1, right M2) both
// lines of a left->M2 / right->M1 swap did nothing at all, and since the
// refusal came before the sign was read, the direction dropdown looked
// inert too. It reached a robot before anyone noticed, because no test
// could see it.
#pragma once

#include <cstdint>

namespace diffDrive {

// One side's wiring: which 1-based brick port, and which way round.
struct MotorWiring {
  uint8_t port;   // [M1..M4]
  int8_t sign;    // [+1/-1] fwdSign: multiplies duty AND encoder
};

// Both sides. `target` is the one the caller named.
struct WiringPair {
  MotorWiring target;
  MotorWiring other;
};

constexpr bool isValidMotorPort(int port) { return port >= 1 && port <= 4; }
constexpr bool isValidMotorSign(int sign) { return sign == 1 || sign == -1; }

// Resolve one `configure motor` request into the resulting pair.
//
// Rules, in order:
//   1. An out-of-range port or sign changes NOTHING. A caller cannot
//      half-apply a bad request, and in particular cannot move the other
//      wheel off the back of one.
//   2. The target takes the requested port and sign.
//   3. If that port is where the OTHER wheel was, the two exchange: the
//      other wheel moves to the port the target just vacated, keeping its
//      own sign. This is what makes a swap expressible as one call --
//      and a mirror-image wheel pair, which drives the robot backwards,
//      is the reason the block exists.
//
// The pair is never left sharing a port, and the function is idempotent:
// feeding its own output back in changes nothing.
inline WiringPair applyWiringRequest(MotorWiring target, MotorWiring other,
                                     int port, int sign) {
  if (!isValidMotorPort(port) || !isValidMotorSign(sign))
    return WiringPair{target, other};

  const uint8_t vacated = target.port;
  const uint8_t wanted = static_cast<uint8_t>(port);

  WiringPair out{MotorWiring{wanted, static_cast<int8_t>(sign)}, other};
  if (other.port == wanted) out.other.port = vacated;
  return out;
}

}  // namespace diffDrive
