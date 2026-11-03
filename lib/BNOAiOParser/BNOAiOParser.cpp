// Firmware_Teensy_AiO-New-Dawn is copyright 2025 by the AOG Group
// Firmware_Teensy_AiO-New-Dawn is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// Firmware_Teensy_AiO-New-Dawn is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Firmware_Teensy_AiO-New-Dawn. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

#include "BNOAiOParser.h"
#include "EventLogger.h"

BNOAiOParser::BNOAiOParser() : 
    state(WAIT_HEADER1),
    bufferIndex(0),
    yawX10(0),
    pitchX10(0),
    rollX10(0),
    yawX100(0),
    prevYaw(0),
    angVel(0),
    angCounter(0),
    lastValidTime(0),
    dataValid(false),
    isSwapXY(false)
{
}

void BNOAiOParser::processByte(uint8_t byte)
{
    switch (state)
    {
    case WAIT_HEADER1:
        if (byte == HEADER1)
        {
            buffer[0] = byte;
            bufferIndex = 1;
            state = WAIT_HEADER2;
        }
        break;
        
    case WAIT_HEADER2:
        if (byte == HEADER2)
        {
            buffer[1] = byte;
            bufferIndex = 2;
            state = COLLECT_DATA;
        }
        else
        {
            // Not a valid header sequence, reset
            state = WAIT_HEADER1;
        }
        break;
        
    case COLLECT_DATA:
        buffer[bufferIndex++] = byte;
        
        // Check if we have all data bytes (18 total including headers)
        if (bufferIndex >= DATA_SIZE)
        {
            state = WAIT_CHECKSUM;
        }
        break;
        
    case WAIT_CHECKSUM:
        buffer[bufferIndex] = byte;
        
        // Validate checksum and parse if valid
        if (validateChecksum())
        {
            parsePacket();
            dataValid = true;
            lastValidTime = millis();
        }
        else
        {
            // Checksum failed - could be noise, don't log every failure
            // LOG_WARNING(EventSource::IMU, "BNO checksum failed");
        }
        
        // Reset for next packet
        resetParser();
        break;
    }
    
    // Safety check - reset if buffer overrun
    if (bufferIndex >= PACKET_SIZE)
    {
        resetParser();
    }
}

void BNOAiOParser::resetParser()
{
    state = WAIT_HEADER1;
    bufferIndex = 0;
}

bool BNOAiOParser::validateChecksum()
{
    // Calculate checksum on payload bytes (after headers, before checksum)
    // BNO RVC format: AA AA [15 bytes of data] [checksum]
    // We sum bytes 2 through 16 (0-indexed)
    uint8_t sum = 0;
    for (uint8_t i = 2; i < 18; i++)  // Sum 16 bytes starting from index 2
    {
        sum += buffer[i];
    }
    
    // Compare with received checksum at position 18
    return (sum == buffer[18]);
}

void BNOAiOParser::parsePacket()
{
    // Extract raw values from buffer
    // Buffer layout after headers:
    // [2] = Index (not used)
    // [3-4] = Yaw (little endian)
    // [5-6] = Pitch (little endian)
    // [7-8] = Roll (little endian)
    // Rest of packet not used for basic RVC mode
    
    int16_t rawYaw = buffer[3] | (buffer[4] << 8);
    int16_t rawPitch = buffer[5] | (buffer[6] << 8);
    int16_t rawRoll = buffer[7] | (buffer[8] << 8);
    
    // Store raw yaw for angular velocity calculation
    yawX100 = rawYaw;
    
    // Calculate angular velocity (same as original BNO_RVC)
    if (angCounter < 20)
    {
        angVel += (rawYaw - prevYaw);
        angCounter++;
        prevYaw = rawYaw;
    }
    else
    {
        angCounter = 0;
        prevYaw = 0;
        angVel = 0;
    }
    
    // ---- Units (checked 2026-10-05; MTZ8302's 2026-05-07 fix is the authoritative behaviour)
    // The three RVC fields do NOT share a unit in this stream:
    //   yaw   : 0.01 deg  -> scaled by 0.1 here, so yawX10 IS in 0.1 deg units (getYaw() = /10 -> deg)
    //   pitch : 0.1  deg  -> taken AS IS into pitchX10            (getPitch() = /10 -> deg)
    //   roll  : 0.1  deg  -> taken AS IS into rollX10             (getRoll()  = /10 -> deg)
    // getYaw()/getPitch()/getRoll() therefore return DEGREES, and so does every value downstream
    // (IMUProcessor::currentData.roll/pitch, HeadingBridge::Status.imuRollDeg, the roll channel,
    // WheelAngleFusion::rollDeg, the roll bins). Only the wire formats need the x10 form, and the
    // senders convert there: `sendIMUData()` sends `currentData.roll * 10` because AgOpenGPS
    // multiplies the PGN-211 field by 0.1, while the PANDA field carries whole degrees.
    // NOTE: the comment that used to stand here claimed the raw values were 0.01 deg for ALL
    // THREE fields - true for yaw only. That comment is what made a 2026-10-05 attempt "correct"
    // the pitch/roll scaling (which would have made them 10x too small); it was reverted. If you
    // ever doubt this, compare a known tilt: the value AOG shows must equal the geometry.
    yawX10 = (int16_t)(rawYaw * 0.1f);
    if (yawX10 < 0) yawX10 += 3600;  // Ensure positive 0-3600 range
    
   // pitchX10 = (int16_t)(rawPitch * 0.1f);   // NOT needed - the field is already x10 (see above)
   // rollX10 = (int16_t)(rawRoll * 0.1f);
    // *** DO NOT "FIX" THIS AGAIN ***
    // MTZ8302 fixed this on 2026-05-07 and the result is verified on the machines: with the RAW
    // value here (getPitch()/getRoll() divide by 10) the roll and pitch that reach AgOpenGPS are
    // CORRECT - the roll has been confirmed in AOG repeatedly on the F9P machine, whose board is
    // firmly mounted with the axes nearly cleanly aligned (mounting offset <= 5 deg).
    // Note the two field groups do NOT share a unit in this RVC stream: the yaw above IS scaled
    // by 0.1, the pitch/roll are not (both are verified correct as they are).
    //
    // 2026-10-05: an attempt to "restore" the * 0.1f for pitch/roll - concluding "10x too large"
    // from the boot line and from the width of the roll spread in the logs - was WRONG and has
    // been reverted. Two mistakes in that reasoning, both worth remembering:
    //   * the boot line is the FIRST packet, printed ~10 ms after "Initializing BNO085 RVC mode",
    //     so the fusion has not converged - it is not a measurement (the values looked
    //     "impossible", which is exactly what an unconverged first packet looks like);
    //   * "the route has <= 5 deg of tilt" was used as a terrain bound for a spread measured in a
    //     column that is not necessarily that angle (axis configuration!).
    pitchX10 = (int16_t)(rawPitch);//bugfix MTZ8302 2026-05-07
    rollX10 = (int16_t)(rawRoll);
    
    // Handle axis swap if configured
    if (isSwapXY)
    {
        int16_t temp = pitchX10;
        pitchX10 = rollX10;
        rollX10 = temp;
    }
}

// FLASHMEM: called ~1 Hz from IMUProcessor::processBNO085Data() as the raw-IMU log line; it
// must not add to ITCM, where only a few hundred bytes of padding are left.
void FLASHMEM BNOAiOParser::printDebug()
{
    // Units (checked 2026-10-05): getYaw/getPitch/getRoll return DEGREES (see the units note in
    // parsePacket), the get*X10 accessors return the stored 0.1 deg representation. swapXY says
    // whether the firmware swaps the two tilt axes; operator 2026-10-05: the F9P machine uses
    // "x", i.e. the swap is OFF, so currentData.roll is this roll.
    LOG_DEBUG(EventSource::IMU,
              "BNO: yaw=%.1f pitch=%.1f roll=%.1f rate=%.1f valid=%d swapXY=%d x10=%d/%d/%d",
              getYaw(), getPitch(), getRoll(), getYawRate(), isDataValid(),
              isSwapXY ? 1 : 0, getYawX10(), getPitchX10(), getRollX10());
}
