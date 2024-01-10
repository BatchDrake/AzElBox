//
//
// Copyright (c) 2023 Gonzalo J. Carracedo <BatchDrake@gmail.com>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of IncrementalEmulator nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//

#include "MotorState.h"
#include <pcf8574.h>
#include <cmath>

#include "helpers.h"

PCF8574      g_homing(0x20);

//////////////////////////// Abstract commands /////////////////////////////////
void
AbstractMotorCommand::init()
{

}

MotorDirection
AbstractMotorCommand::direction()
{
  return Disabled;
}

uint8_t
AbstractMotorCommand::speed()
{
  return 0;
}

bool
AbstractMotorCommand::stop()
{
  return true;
}

void
AbstractMotorCommand::fini()
{

}

/////////////////////////// Supported commands /////////////////////////////////
bool
GotoCommand::setLocation(int32_t location)
{
  // Turn this location into pulses
  int32_t diff = location - (int32_t) state()->currLocation;
  int32_t cycleLen = state()->cycleLen();
  diff %= cycleLen;

  // If the number of pulses is past 180, this means that we can get to the
  // desired location faster in the opposite direction
  if (diff < -cycleLen / 2)
    diff += cycleLen;
  else if (diff > cycleLen / 2)
    diff -= cycleLen;

  // Adjust sign
  if (diff < 0) {
    diff = -diff;
    m_direction = Backward;
  } else {
    m_direction = Forward;
  }

  m_numPulses   = diff;
  m_accelPulses = state()->accelLen();
  m_brakePulses = state()->brakeLen();
  m_maxSpeed    = state()->maxSpeed();
  m_slowSpeed   = state()->slowSpeed();

  return true;
}

void
GotoCommand::init()
{

}

MotorDirection
GotoCommand::direction()
{
  return m_direction;
}

uint8_t
GotoCommand::speed()
{
  unsigned fromStart = state()->pulses();
  unsigned toFinish = m_numPulses - fromStart - 1;
  uint8_t  extra = m_maxSpeed - m_slowSpeed;

  if (m_maxSpeed == m_slowSpeed)
    return m_maxSpeed;
  
  if (toFinish <= m_brakePulses) {
    float t = (float) toFinish / (float) m_brakePulses;
    return m_slowSpeed + extra * sin(.5 * M_PI * t);
  } else if (fromStart <= m_accelPulses) {
    float t = (float) fromStart / (float) m_accelPulses;
    return m_slowSpeed + extra * sin(.5 * M_PI * t);
  } else {
    return m_maxSpeed;
  }
}

bool
GotoCommand::stop()
{
  return m_numPulses == 0 || state()->pulses() >= m_numPulses;
}

//////////////////////////////// Move Commands /////////////////////////////////
void
MoveCommand::setDirection(MotorDirection dir)
{
  m_direction = dir;
}

void
MoveCommand::setButton(int button)
{
  m_button = button;
}

uint8_t
MoveCommand::speed()
{
  return state()->maxSpeed();
}

MotorDirection
MoveCommand::direction()
{
  return m_direction;
}

bool
MoveCommand::stop()
{
  if (m_button < 0)
    return true;
  
  return digitalRead(m_button) == HIGH;
}

///////////////////////////// Virtual Homing commands //////////////////////////
void
VirtualHomeCommand::setLocation(int32_t pos)
{
  int32_t cycleLen = state()->cycleLen();
  pos %= cycleLen;

  if (pos < 0)
    pos += cycleLen;
  
  m_desiredPos = pos;
}

void
VirtualHomeCommand::init()
{
  state()->currLocation = m_desiredPos;
}

/////////////////////////// Motor State /////////////////////////////////
bool
MotorState::init(MotorProperties const &prop)
{
  m_properties = prop;

  m_gotoCmd.setState(this);
  m_moveCmd.setState(this);
  m_virtualHomeCmd.setState(this);

  if (prop.overCurrentPin > 0) {
    m_currentReadings = -.1 / log(1 - prop.overCurrentAlpha);
    m_current         = readCurrent();
  }

  m_slowSpeed  = prop.maxSpeed * prop.slowFrac;
  m_pendingCmd = nullptr;
  m_cmd        = nullptr;
  m_state      = Idle;

  initialized  = true;

  return true;
}

uint64_t
MotorState::currentTimeout()
{
  return (MOTOR_TIMEOUT_US * m_properties.maxSpeed) / m_currSpeed;
}

uint8_t
MotorState::currSpeed() const
{
  // Curr speed goes from 255 * slowFrac to 255. The range is (1 - slowFrac) * 255
  // Relative speed is m_currSpeed - slowFrac * 255

  int32_t speed = (m_currSpeed - m_slowSpeed) / (1. - m_properties.slowFrac);
  if (speed > 255)
    speed = 255;
  else if (speed < 0)
    speed = 0;
  
  return speed;
}

void
MotorState::drive(MotorDirection dir, bool adjust)
{
  if (!adjust) {
    ledcWrite(m_properties.forwardPWM, 0);
    ledcWrite(m_properties.backwardPWM, 0);
  }

  switch (dir) {
    case Forward:
      ledcWrite(m_properties.forwardPWM, m_currSpeed);
      break;
    
    case Backward:
      ledcWrite(m_properties.backwardPWM, m_currSpeed);
      break;

    case Locked:
      ledcWrite(m_properties.forwardPWM,  m_properties.maxSpeed);
      ledcWrite(m_properties.backwardPWM, 255);
      break;
  }
}

void
MotorState::abort()
{
  if (m_cmd != nullptr)
    m_abrtReq = true;
}

static const char *g_reasonStrings[] = {
  "SUCCESS", "ABORTED", "TIMEOUT",
  "LIMIT", "OVERCURRENT", "NOCOMMAND",
  "STILLRUNNING"};

void
MotorState::checkStopCondition()
{
  MotorStopReason reason = MotorStopReason::Success;

  if (m_cmd == nullptr)
    return;
  
  // Check overcurrent
  if (m_checkOverCurrent) {
    if (m_current > m_properties.maxCurrent) {
      reason = MotorStopReason::OverCurrent;
      goto do_finish;  
    }
  }

  // Check abort request
  if (m_abrtReq) {
    m_abrtReq = false;
    reason = MotorStopReason::Aborted;
    goto do_finish;
  }

  // In the running state, this is called after a timeout
  if (m_state == MotorCommandState::Running) {
    uint64_t currEdge = micros();
    uint64_t ellapsed = currEdge - m_lastEdge;

    if (ellapsed > currentTimeout()) {
      reason = MotorStopReason::Timeout;
      goto do_finish;
    }

    if (m_currSpeed == 0 || m_direction == Disabled) {
      // This is equivalent to a natural stop.
      goto do_finish;  
    }
  }

  // TODO: Check limits

  // Check natural stop
  if (m_cmd->stop())
    goto do_finish;
  
  // Nothing to do here.
  return;

do_finish:
  m_reason = reason;
  
  if (reason == MotorStopReason::Success)
    m_cmd->fini();

  m_cmd   = nullptr;

  drive(Disabled);
  m_currSpeed  = 0;
  m_currPulses = 0;

  m_state = Finalized;

  LOG("I:FINALIZED[%s]:%s\r\n", name(), g_reasonStrings[reason]);
}

void
MotorState::updateMotorSpeed()
{
  m_currSpeed = m_cmd->speed();
  m_direction = m_cmd->direction();
}

void
MotorState::iteration()
{
  bool currPulse = digitalRead(m_properties.sensorPin);
  uint64_t currEdge, ellapsed;

  // Update overcurrent (if applies)
  if (m_currentReadings > 0)
    if (--m_currentReadings == 0)
      m_checkOverCurrent = true;
  
  if (m_properties.overCurrentPin > 0) {
    float alpha = m_properties.overCurrentAlpha;
    m_current += alpha * readCurrent() - (1. - alpha) * m_current;
  }

  switch (m_state) {
    case Idle:
      // Idle state. We iterate until a new command has been queued.
      if (m_cmdFlag && m_pendingCmd != nullptr) {
        // Exchange
        m_cmd        = m_pendingCmd;
        m_pendingCmd = nullptr;
        m_cmdFlag    = false;

        // Prepare state
        m_reason     = MotorStopReason::Success;
        m_currPulses = 0;
        m_abrtReq    = false;
        
        // Initialized and acknowledged
        m_cmd->init();

        m_state      = Acknowledged;
      }
      break;

    case Acknowledged:
      // Acknowledged state. Determine speed and stop condition.
      updateMotorSpeed();
      checkStopCondition();

      if (m_state != Finalized) {
        m_state      = Running;
        m_prevState  = currPulse;
        startTimeMs  = millis();
        m_lastEdge   = micros();
        drive(m_direction);
      }

      break;

    case Running:
      currEdge = micros();
      ellapsed = currEdge - m_lastEdge;

      if (currPulse != m_prevState) {
        // Leaving pulse. Updaye current location
        if (currPulse && ellapsed > MIN_PULSE_TIME_US) {
          ++m_currPulses;
          if (m_direction == Forward)
            ++currLocation;
          else if (m_direction == Backward)
            --currLocation;

          checkStopCondition();

          if (m_state != Finalized) {
            updateMotorSpeed();
            drive(m_direction, true);
          }
        }

        m_prevState = currPulse;
        m_lastEdge  = currEdge;
      } else if (m_abrtReq || ellapsed > currentTimeout()) {
        checkStopCondition();
      }

      break;
  }

  taskYIELD();
}

MotorStopReason
MotorState::popStopReason()
{
  if (m_state == Idle)
    return NoCommand;

  if (m_state != Finalized)
    return StillRunning;

  m_state = Idle;
  return m_reason;
}

void
MotorState::goTo(uint16_t location)
{
  if (m_state != Idle) {
    LOG("E:%s:Motor is busy\r\n", name());
    return;
  }

  m_gotoCmd.setLocation(location);

  m_pendingCmd = &m_gotoCmd;
  m_cmdFlag    = true;
}

void
MotorState::virtualHome(int16_t location)
{
  if (m_state != Idle) {
    LOG("E:%s:Motor is busy\r\n", name());
    return;
  }

  m_virtualHomeCmd.setLocation(location);
  m_pendingCmd = &m_virtualHomeCmd;
  m_cmdFlag    = true;
}

void
MotorState::move(MotorDirection direction, int button)
{
  if (m_state != Idle) {
    LOG("E:%s:Motor is busy (speed: %d)\r\n", name(), currSpeed());
    return;
  }

  m_moveCmd.setButton(button);
  m_moveCmd.setDirection(direction);

  m_pendingCmd = &m_moveCmd;
  m_cmdFlag    = true;
}

// Check if current is available
bool
MotorState::haveCurrent() const
{
  return m_checkOverCurrent;
}

// Get current
float
MotorState::current() const
{
  return m_current;
}

void
MotorState::setMaxCurrent(float maxC)
{
  m_properties.maxCurrent = maxC;
}

void
MotorState::setSlowSpeedFrac(float frac)
{
  m_properties.slowFrac = frac;
  m_slowSpeed  = m_properties.maxSpeed * m_properties.slowFrac;
}
