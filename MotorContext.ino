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

#include <string>
#include <vector>
#include <sstream>
#include <cstring>

#include "MotorState.h"
#include "OLEDUI.h"
#include "helpers.h"

#define MAX_CURRENT        1.7
#define PSH_AZ_DEC         4
#define PSH_AZ_INC         0
#define PSH_EL_DEC         2
#define PSH_EL_INC         15

#define AZ_MOTOR_FWD       16
#define AZ_MOTOR_BWD       17
#define SENSOR_AZ          32

#define EL_MOTOR_FWD       18
#define EL_MOTOR_BWD       19
#define SENSOR_EL          33

#define AZ_CURRENT         35
#define EL_CURRENT         34

#define AZ_HOME_MIN        26 // EL_MAX = 25
#define AZ_HOME_MAX        14 // AZ_MIN = 26
#define EL_HOME_MIN        27 // EL_MIN = 27
#define EL_HOME_MAX        25 // AZ_MAX = 14

#define AZ_PWM_FWD_CHANNEL 0
#define AZ_PWM_BWD_CHANNEL 1
#define EL_PWM_FWD_CHANNEL 2
#define EL_PWM_BWD_CHANNEL 3

#define PWM_FREQUENCY      1000
#define PWM_RESOLUTION     8

TaskHandle_t g_pollTaskHandle;
RotorContext g_context;
OLEDUI       g_ui;
bool         g_reports = false;

SemaphoreHandle_t g_serialMutex;

struct CommandDesc {
  const char *command;
  int args;
  bool (*callback) (std::vector<std::string> const &args);
};

void
acquireSerial()
{
  xSemaphoreTake(g_serialMutex, portMAX_DELAY);
}

void
releaseSerial()
{
  xSemaphoreGive(g_serialMutex);
}

bool
cmdAZ(std::vector<std::string> const &args)
{
  float fAngle;

  if (sscanf(args[1].c_str(), "%f", &fAngle) < 1) {
    LOG("E:Invalid angle\r\n");
    return false;
  }

  LOG("I:MOVE[AZ]:%g\r\n", fAngle);
  g_context.azimuth.goTo(2 * fAngle);
  return true;
}

bool
cmdEL(std::vector<std::string> const &args)
{
  float fAngle;

  if (sscanf(args[1].c_str(), "%f", &fAngle) < 1) {
    LOG("E:Invalid angle\r\n");
    return false;
  }

  LOG("I:MOVE[EL]:%g\r\n", fAngle);
  g_context.elevation.goTo(2 * fAngle);
  return true;
}

bool
cmdGOTO(std::vector<std::string> const &args)
{
  float fAz, fEl;

  if (sscanf(args[1].c_str(), "%f", &fAz) < 1 
    || sscanf(args[2].c_str(), "%f", &fEl) < 1) {
    LOG("E:Invalid angles\r\n");
    return false;
  }

  LOG("I:GOTO:%g:%g\r\n", fAz, fEl);

  g_context.azimuth.goTo(2 * fAz);
  g_context.elevation.goTo(2 * fEl);

  return true;
}

bool
cmdADVANCE(std::vector<std::string> const &args)
{
  float fDeltaAz, fDeltaEl;

  if (sscanf(args[1].c_str(), "%f", &fDeltaAz) < 1 
    || sscanf(args[2].c_str(), "%f", &fDeltaEl) < 1) {
    LOG("E:Invalid angles\r\n");
    return false;
  }

  LOG("I:ADVANCE:%g:%g\r\n", fDeltaAz, fDeltaEl);

  g_context.azimuth.advance(2 * fDeltaAz);
  g_context.elevation.advance(2 * fDeltaEl);

  return true;
}

bool
cmdOVERCURRENT(std::vector<std::string> const &args)
{
  float current;
  MotorState *state = nullptr;

  if (sscanf(args[2].c_str(), "%f", &current) < 1 || current < 0) {
    LOG("E:Invalid overcurrent specification\r\n");
    return false;
  }

  if (args[1] == "AZ") {
    state = &g_context.azimuth;
  } else if (args[1] == "EL") {
    state = &g_context.elevation;
  } else {
    LOG("E:Invalid motor specification\r\n");
    return false;
  }

  state->setMaxCurrent(current);

  LOG("I:OVERCURRENT[%s]:%6.4f\r\n", args[1].c_str(), current);

  return true;
}

bool
cmdMINSPEED(std::vector<std::string> const &args)
{
  float percent;
  MotorState *state = nullptr;

  if (sscanf(args[2].c_str(), "%f", &percent) < 1 || percent < 0 || percent > 100) {
    LOG("E:Invalid slow speed rate specification\r\n");
    return false;
  }

  if (args[1] == "AZ") {
    state = &g_context.azimuth;
  } else if (args[1] == "EL") {
    state = &g_context.elevation;
  } else {
    LOG("E:Invalid motor specification\r\n");
    return false;
  }

  state->setSlowSpeedFrac(percent * 1e-2);

  LOG("I:MINSPEED[%s]:%6.4f\r\n", args[1].c_str(), percent);

  return true;
}

bool
cmdPARKING(std::vector<std::string> const &args)
{
  MotorState *state = nullptr;
  MotorDirection direction = Disabled;
  int16_t pulses;
  float maxDeg;

  if (sscanf(args[2].c_str(), "%f", &maxDeg) < 1 || fabs(maxDeg) > 360) {
    LOG("E:Invalid search angle\r\n");
    return false;
  }

  if (args[1] == "AZ") {
    state = &g_context.azimuth;
  } else if (args[1] == "EL") {
    state = &g_context.elevation;
  } else {
    LOG("E:Invalid motor specification\r\n");
    return false;
  }

  pulses = 2 * maxDeg;
  
  if (pulses > 0) {
    direction = Forward;
  } else if (pulses < 0) {
    direction = Backward;
    pulses    = -pulses;
  }

  state->parking(direction, (uint16_t) pulses);

  return true;
}

//
// FLAGS AZ SET/UNSET/TEST 0
//
bool
cmdFLAGS(std::vector<std::string> const &args)
{
  MotorState *state = nullptr;
  MotorDirection direction = Disabled;
  unsigned flag;

  if (args[1] == "AZ") {
    state = &g_context.azimuth;
  } else if (args[1] == "EL") {
    state = &g_context.elevation;
  } else {
    LOG("E:Invalid motor specification\r\n");
    return false;
  }

  if (sscanf(args[3].c_str(), "%u", &flag) < 1) {
    LOG("E:Invalid flag\r\n");
    return false;
  }

  if (args[2] == "SET")
    state->setFlag(static_cast<MotorStateFlag>(flag));
  else if (args[2] == "UNSET")
    state->unsetFlag(static_cast<MotorStateFlag>(flag));
  else if (args[2] == "TEST")
    LOG("I:FLAG[%s]:%d:%d\r\n", args[1].c_str(), flag, state->testFlag(static_cast<MotorStateFlag>(flag)));

  LOG("I:FLAGS[%s]:0x%x\r\n", args[1].c_str(), state->getFlags());
  
  return true;
}

bool
cmdSWITCHES(std::vector<std::string> const &args)
{
  bool azMin = digitalRead(AZ_HOME_MIN) == HIGH;
  bool azMax = digitalRead(AZ_HOME_MAX) == HIGH;
  bool elMin = digitalRead(EL_HOME_MIN) == HIGH;
  bool elMax = digitalRead(EL_HOME_MAX) == HIGH;

  LOG("I:SWITCHES:%d,%d,%d,%d\r\n", azMin, azMax, elMin, elMax);

  return true;
}

bool
cmdABORT(std::vector<std::string> const &args)
{
  g_context.azimuth.abort();
  g_context.elevation.abort();

  return true;
}

bool
cmdPOS(std::vector<std::string> const &args)
{
  LOG("I:POS:");

  if (g_context.azimuth.initialized) {
    float az = .5 * g_context.azimuth.currLocation;
    LOG("%g:", az);
  } else {
    LOG("NA:");
  }

  if (g_context.elevation.initialized) {
    float el = .5 * g_context.elevation.currLocation;
    LOG("%g\r\n", el);
  } else {
    LOG("NA\r\n");
  }

  return true;
}

bool
cmdREPORT(std::vector<std::string> const &args)
{
  g_reports = args[1] == "on" || args[1] == "ON";

  LOG("I:Reports are %s\r\n", g_reports ? "ON" : "OFF");

  return true;
}

bool
cmdVH(std::vector<std::string> const &args)
{
  float fAz, fEl;

  if (sscanf(args[1].c_str(), "%f", &fAz) < 1 
    || sscanf(args[2].c_str(), "%f", &fEl) < 1) {
    LOG("E:Invalid angles\r\n");
    return false;
  }

  LOG("I:VH:%g:%g\r\n", fAz, fEl);

  g_context.azimuth.virtualHome(2 * fAz);
  g_context.elevation.virtualHome(2 * fEl);

  return true;
}

void
pollTask(void *opaque)
{
  RotorContext *context = &g_context;
  
  delay(1000);
  LOG("Polling task started!\r\n");

  for (;;) {
    context->azimuth.iteration();
    context->elevation.iteration();
  }
}

const CommandDesc g_commands[] =
{
  {"AZ",          1, cmdAZ},
  {"EL",          1, cmdEL},
  {"GOTO",        2, cmdGOTO},
  {"ADVANCE",     2, cmdADVANCE},
  {"OVERCURRENT", 2, cmdOVERCURRENT},
  {"PARKING",     2, cmdPARKING},
  {"ABORT",       0, cmdABORT},
  {"POS",         0, cmdPOS},
  {"SWITCHES",    0, cmdSWITCHES},
  {"VH",          2, cmdVH},
  {"MINSPEED",    2, cmdMINSPEED},
  {"REPORT",      1, cmdREPORT},
  {"FLAGS",       3, cmdFLAGS},
  {nullptr,       0, nullptr}
};

void
setup() {
  Serial.begin(115200);

  do {
  g_serialMutex = xSemaphoreCreateMutex();

    if (g_serialMutex == nullptr) {
      LOG("Critical: Failed to create mutex\n");
      delay(1000);
    }
  } while (g_serialMutex == nullptr);

  pinMode(AZ_MOTOR_FWD, OUTPUT);
  pinMode(AZ_MOTOR_BWD, OUTPUT);

  pinMode(SENSOR_AZ, INPUT_PULLUP);

  pinMode(EL_MOTOR_FWD, OUTPUT);
  pinMode(EL_MOTOR_BWD, OUTPUT);

  pinMode(SENSOR_EL, INPUT_PULLUP);

  pinMode(AZ_CURRENT, INPUT);
  pinMode(EL_CURRENT, INPUT);

  pinMode(PSH_AZ_DEC, INPUT_PULLUP);
  pinMode(PSH_AZ_INC, INPUT_PULLUP);
  pinMode(PSH_EL_DEC, INPUT_PULLUP);
  pinMode(PSH_EL_INC, INPUT_PULLUP);

  pinMode(AZ_HOME_MIN, INPUT_PULLUP);
  pinMode(AZ_HOME_MAX, INPUT_PULLUP);
  pinMode(EL_HOME_MIN, INPUT_PULLUP);
  pinMode(EL_HOME_MAX, INPUT_PULLUP);

  ledcSetup(AZ_PWM_FWD_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(AZ_PWM_BWD_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(AZ_MOTOR_FWD, AZ_PWM_FWD_CHANNEL);
  ledcAttachPin(AZ_MOTOR_BWD, AZ_PWM_BWD_CHANNEL);

  ledcSetup(EL_PWM_FWD_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(EL_PWM_BWD_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(EL_MOTOR_FWD, EL_PWM_FWD_CHANNEL);
  ledcAttachPin(EL_MOTOR_BWD, EL_PWM_BWD_CHANNEL);

  MotorProperties properties;
  properties.name        = "AZ";
  properties.sensorPin   = SENSOR_AZ;
  properties.backwardPWM = AZ_PWM_BWD_CHANNEL;
  properties.forwardPWM  = AZ_PWM_FWD_CHANNEL;
  properties.cycleLen    = 720;
  properties.slowFrac    = .55;
  properties.overlookFirst = true;

  properties.overCurrentAlpha = 1e-4;
  properties.overCurrentPin   = AZ_CURRENT;
  properties.overCurrentSlope = 108.;
  properties.overCurrentZero  = 2524;
  properties.maxCurrent       = 3.;

  properties.homeLow          = AZ_HOME_MIN;
  properties.homeHigh         = AZ_HOME_MAX;
  g_context.azimuth.init(properties);

  properties.name        = "EL";
  properties.sensorPin   = SENSOR_EL;
  properties.backwardPWM = EL_PWM_BWD_CHANNEL;
  properties.forwardPWM  = EL_PWM_FWD_CHANNEL;
  properties.cycleLen    = 720;
  properties.slowFrac    = 1;
  properties.pulseFwdUs  = (2 * DEFAULT_PULSE_TIME_US) / 3;
  properties.overlookFirst = true;
  
  properties.overCurrentAlpha = 1e-4;
  properties.overCurrentPin   = EL_CURRENT;
  properties.overCurrentSlope = 108.;
  properties.overCurrentZero  = 2529;
  properties.maxCurrent       = 5.;
  
  properties.homeLow          = EL_HOME_MIN;
  properties.homeHigh         = EL_HOME_MAX;
  g_context.elevation.init(properties);

  xTaskCreatePinnedToCore(
    pollTask,
    "pollTask",
    10000,
    NULL,
    0,
    &g_pollTaskHandle,
    0);
  
  g_ui.init();
}

int g_moves = 0;

unsigned int retry = 0;
bool g_err_notified = false;

static int g_locations [] = {
  30,
  330,
  0
};

#define CMD_LINE_SZ 100
char g_command[CMD_LINE_SZ];
char g_cmdptr = 0;

void
parseCommand()
{
  std::vector<std::string> args;
  std::stringstream ss(g_command);
  std::string tmp;

  while (std::getline(ss, tmp, ' '))
    args.push_back(tmp);
  
  if (args.size() > 0) {
    int i = 0;

    while (g_commands[i].command != nullptr) {
      if (strcasecmp(args[0].c_str(), g_commands[i].command) == 0)
        break;
      ++i;
    }

    if (g_commands[i].command == nullptr) {
      LOG("E:%s:Unknown command\r\n", args[0].c_str());
      return;
    } else if (g_commands[i].args != args.size() - 1) {
      LOG(
        "E:%s:Invalid number of arguments (expected %d)\r\n",
        args[0].c_str(),
        g_commands[i].args);
      return;
    }

    (void) g_commands[i].callback(args);
  }
}

void
readCommand()
{
  while (Serial.available() > 0) {
    char b = Serial.read();
    g_command[g_cmdptr] = toupper(b);
    if (g_cmdptr == CMD_LINE_SZ - 1) {
      LOG("E:Command line too big\r\n");
      g_cmdptr = 0;
    } else if (b == '\n' || b == '\r') {
      g_command[g_cmdptr] = '\0';
      if (g_cmdptr > 0) {
        parseCommand();
        g_cmdptr = 0;
      }
    } else {
      ++g_cmdptr;
    }
  }
}

struct PollButton {
  MotorState    *motor;
  uint8_t        pin;
  MotorDirection dir;
  bool           state;
};

void
pollButtons()
{
  static bool prevState = false;
  bool state = false;

  PollButton buttons[4] = {
    {&g_context.azimuth,   PSH_AZ_DEC, Backward, false},
    {&g_context.azimuth,   PSH_AZ_INC, Forward, false},
    {&g_context.elevation, PSH_EL_DEC, Backward, false},
    {&g_context.elevation, PSH_EL_INC, Forward, false},
  };

  for (int i = 0; i < 4; ++i) {
    buttons[i].state = digitalRead(buttons[i].pin) == LOW;
    state |= buttons[i].state;
  }

  // Transition?
  if (state && prevState != state) {
    bool allOk = true;
    if (g_context.azimuth.state() == Running) {
      allOk = false;
      g_context.azimuth.abort();
    }

    if (g_context.elevation.state() == Running) {
      allOk = false;
      g_context.elevation.abort();
    }

    // Nothing aborted? Ok, command.
    if (allOk)
      for (int i = 0; i < 4; ++i)
        if (buttons[i].state)
          buttons[i].motor->move(buttons[i].dir, buttons[i].pin);    
  }

  prevState = state;
}

void
printReports()
{
  LOG(
    "I:REPORT[AZ]:%g:%g:%d:%d:%d:%d\r\n",
    .5 * g_context.azimuth.currLocation,
    g_context.azimuth.current(),
    g_context.azimuth.state(),
    g_context.azimuth.reason(),
    g_context.azimuth.homedLow(),
    g_context.azimuth.homedHigh());
  
  LOG(
    "I:REPORT[EL]:%g:%g:%d:%d:%d:%d\r\n",
    .5 * g_context.elevation.currLocation,
    g_context.elevation.current(),
    g_context.elevation.state(),
    g_context.elevation.reason(),
    g_context.elevation.homedLow(),
    g_context.elevation.homedHigh());
}

void
loop() {
  static int n = 0;

  g_ui.drawRotorContext(g_context);
  g_ui.clock();

  pollButtons();
  delay(50);

  readCommand();
  
  if (g_reports)
    printReports();
  
  if (++n == 10)
    n = 0;
  
}
