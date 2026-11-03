#pragma once
#include <Arduino.h>
#include "PositionHistory.h"

// =================================================================================
// PositionHistoryService - the ONE place that fills the ring.
// ---------------------------------------------------------------------------------
// Implemented in PositionHistoryService.cpp, FLASHMEM for a reason: RAM1 has only ~684
// bytes of ITCM headroom before the next 32 KB DTCM bank cliff (knowledge/MEMORY_BASELINE.md).
//
// Call once per 100 Hz AutosteerProcessor cycle, AFTER headingBridge.update():
//
//     positionHistoryService(millis(), currentAngle, steerAngleOutQuality);
//
// The fix gate is inside PositionHistory::push() - it drops a record whose position counter
// equals the newest one - so this needs no state of its own and 9 of 10 calls are cheap
// duplicates. Why the gate belongs there: ONE rule for "what is a new fix", the same rule the
// ring documents, and duplicatesSkipped() turns the call rate into a measured number instead
// of a comment.
//
// Returns true when a NEW fix was stored.
//
// See knowledge/position_ring_buffer_architecture.md (sections 4 and 7) for the field list
// and the ordering argument.
// =================================================================================
bool positionHistoryService(uint32_t now, float wheelDeg, float wheelQualPct);
