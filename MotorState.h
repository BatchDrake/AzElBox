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

#ifndef _MOTOR_STATE_H
#define _MOTOR_STATE_H

#include <cstdint>

#define MIN_COUNTS                1000
#define MIN_FWD_PULSE_TIME_US   100000
#define MIN_BWD_PULSE_TIME_US    30000
#define MOTOR_TIMEOUT_US       2000000

enum MotorDirection {
  Disabled,
  Forward,
  Backward,
  Locked
};

enum MotorCommandState {
  Idle,           // No command, pending pointer is null. This is where we can send new commands.
  Acknowledged,   // Init called, motor state updated, next state determined
  Running,        // Motor is moving, angles updated after every tick, which is used to call stop() and speed()
  Finalized       // Motor is stopped, status flags updated accordingly. The UI can pop the state, which causes a transition to Idle.
};

enum MotorStopReason {
  Success,
  Aborted,
  Timeout,
  LimitSwitch,
  OverCurrent,
  NoCommand,
  StillRunning
};

struct MotorState;

class AbstractMotorCommand {
    MotorState *m_state = nullptr;
    
  protected:
    inline MotorState *
    state()
    {
      return m_state;
    }

  public:
    inline void setState(MotorState *state) { m_state = state; }

    virtual void init();
    virtual MotorDirection direction();
    virtual uint8_t speed();
    virtual bool stop();
    virtual void fini();
};

class GotoCommand : public AbstractMotorCommand {
    uint32_t m_numPulses;
    MotorDirection m_direction;
    uint32_t m_accelPulses;
    uint32_t m_brakePulses;
    uint32_t m_maxSpeed;
    uint32_t m_slowSpeed;

  public:
    bool setLocation(int32_t);

    virtual void init() override;
    virtual MotorDirection direction() override;
    virtual uint8_t speed() override;
    virtual bool stop() override;
};

class AbortCommand : public AbstractMotorCommand {
  public:
    virtual void init() override;
};

class MoveCommand : public AbstractMotorCommand {
    int            m_button = -1;
    MotorDirection m_direction = Disabled;

  public:
    void setDirection(MotorDirection);
    void setButton(int);

    virtual MotorDirection direction() override;
    virtual uint8_t speed() override;
    virtual bool stop() override;
};

class ParkingCommand : public AbstractMotorCommand {
    MotorDirection m_direction;
    unsigned int   m_maxPulses = 360;

  public:
    void setDirection(MotorDirection);
    void setMaxPulses(unsigned int);

    virtual MotorDirection direction() override;
    virtual uint8_t speed() override;
    virtual bool stop() override;
};

class VirtualHomeCommand : public AbstractMotorCommand {
    int32_t m_desiredPos = 0;

  public:
    void setLocation(int32_t);
    virtual void init() override;
};

struct MotorProperties {
  // Mandatory, need to be specified always
  unsigned int sensorPin        = 0;
  unsigned int forwardPWM       = 0;
  unsigned int backwardPWM      = 0;

  int          overCurrentPin   = -1;
  float        overCurrentSlope = 10.8;
  int32_t      overCurrentZero  = 0;
  float        overCurrentAlpha = 1e-2;
  float        maxCurrent       = 2.;
  const char  *name             = "M1";

  // Sane defaults
  uint32_t maxSpeed    = 255;
  float    slowFrac    = .55;
  uint32_t accelPulses = 1;
  uint32_t brakePulses = 10;
  uint32_t cycleLen    = 720; // Pulses of 0.5º in a 360º turn

  // Homing pins
  int homeLow  = -1;
  int homeHigh = -1;
};

struct MotorState {
    int32_t  currLocation = 0;
    uint64_t startTimeMs;       // milliseconds at start
    uint64_t endTimeMs;         // milliseconds at end
    bool     initialized = false;

  private:
    // Command allocation
    GotoCommand        m_gotoCmd;
    MoveCommand        m_moveCmd;
    VirtualHomeCommand m_virtualHomeCmd;
    ParkingCommand     m_parkingCmd;

    // State machine
    bool                  m_cmdFlag    = false;
    AbstractMotorCommand *m_pendingCmd = nullptr;
    AbstractMotorCommand *m_cmd        = nullptr;
    MotorCommandState     m_state      = Idle;
    MotorStopReason       m_reason     = Success;
    bool                  m_abrtReq    = false;

    // Overcurrent detection
    bool                  m_checkOverCurrent = false;
    int64_t               m_currentReadings  = -1;
    float                 m_current    = 0;

    // Pulse counter state
    bool                  m_prevState;
    uint64_t              m_lastEdge; // Timeout of the last edge
    uint32_t              m_currPulses = 0; // Current number of pulses
    
    // Homing switch state
    bool                  m_prevHomeLow;
    bool                  m_prevHomeHigh;
    bool                  m_homedLow  = false;
    bool                  m_homedHigh = false;

    // Motor speed state
    MotorDirection  m_direction = Disabled;
    uint8_t         m_currSpeed = 255; // Current PWM speed

    MotorProperties m_properties;
    uint8_t         m_slowSpeed;
    
    void updateMotorSpeed();
    void checkStopCondition();

    // Actuate motor
    void drive(MotorDirection dir, bool adjust = false);

    // Get instantaneous current
    inline float
    readCurrent() const
    {
      int32_t reading = analogReadMilliVolts(m_properties.overCurrentPin);
      return m_properties.overCurrentSlope * (float) (reading - m_properties.overCurrentZero);
    }

  public:
    inline MotorCommandState state() const     { return m_state; }
    inline MotorDirection    direction() const { return m_direction; }
    inline uint8_t           speed() const     { return m_currSpeed; }
    inline MotorStopReason   reason() const    { return m_reason; }
    inline uint32_t          maxSpeed() const  { return m_properties.maxSpeed; }
    inline uint32_t          slowSpeed() const { return m_slowSpeed; }
    inline uint32_t          pulses() const    { return m_currPulses; }
    inline uint32_t          brakeLen() const  { return m_properties.brakePulses; }
    inline uint32_t          accelLen() const  { return m_properties.accelPulses; }
    inline uint32_t          cycleLen() const  { return m_properties.cycleLen; }
    inline bool              canHomeLow() const  { return m_properties.homeLow  > 0; }
    inline bool              canHomeHigh() const { return m_properties.homeHigh > 0; }
    inline bool              homedLow() const  { return m_homedLow; }
    inline bool              homedHigh() const { return m_homedHigh; }
    inline const char *      name() const      { return m_properties.name; }

    // Sets the motor back to idle
    MotorStopReason   popStopReason();

    bool init(MotorProperties const &prop);

    // Calculate current (reasonable) command timeout
    uint64_t currentTimeout();

    // Get normalized speed, according to the slowest speed
    uint8_t currSpeed() const;

    // Set minimum (PWM) speed
    void setSlowSpeedFrac(float);

    // Go to an angle
    void goTo(uint16_t);

    // Run until button released
    void move(MotorDirection direction, int button);

    // Cancel a movement
    void abort();

    // Perform a virtual homing command
    void virtualHome(int16_t);

    // Start a parking movement
    void parking(MotorDirection, uint16_t pulses = 360);

    // Check if current is available
    bool haveCurrent() const;

    // Get current
    float current() const;

    // Set overcurrent
    void setMaxCurrent(float);

    // Polling task iteration
    void iteration();
};

// Full rotor context
struct RotorContext {
  MotorState azimuth;
  MotorState elevation;
};

#endif // _MOTOR_STATE_H
