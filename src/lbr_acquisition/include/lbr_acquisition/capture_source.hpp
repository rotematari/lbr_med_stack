#pragma once

// PHD-FRI-RECORDER: one explicit provenance identity follows a capture from
// producer through container, manifest, outcome, and strict readback.
#include <stdexcept>

namespace lbr_acquisition {
enum class CaptureSource { Synthetic, FriSdk };

inline const char *capture_source_name(CaptureSource source) {
  switch (source) {
  case CaptureSource::Synthetic: return "synthetic";
  case CaptureSource::FriSdk: return "fri-sdk";
  }
  throw std::invalid_argument("unsupported capture source");
}

inline const char *capture_profile(CaptureSource source) {
  switch (source) {
  case CaptureSource::Synthetic: return "offline-fixture-v1";
  case CaptureSource::FriSdk: return "fri-capture-v1";
  }
  throw std::invalid_argument("unsupported capture source");
}

inline const char *capture_outcome_profile(CaptureSource source) {
  switch (source) {
  case CaptureSource::Synthetic: return "offline-capture-outcome-v1";
  case CaptureSource::FriSdk: return "fri-capture-outcome-v1";
  }
  throw std::invalid_argument("unsupported capture source");
}
} // namespace lbr_acquisition
