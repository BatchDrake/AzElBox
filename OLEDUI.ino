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


#include <Wire.h>
#include <GyverOLED.h>
#include "OLEDUI.h"
#include "MotorState.h"

const uint8_t g_AzElGlyphs[] PROGMEM = {
  #include "glyphs.h" // Yes
};

GyverOLED<SSH1106_128x64> g_oled;

void
OLEDUI::init()
{
  g_oled.init();
  Wire.setClock(800000L);
}

void
OLEDUI::drawMotorState(
  MotorState &motor,
  unsigned int x0,
  unsigned int y0)
{
  bool dotFive = false;
  uint8_t glyphs[4];
  int32_t pos = motor.currLocation;

  if (motor.initialized) {
    while (pos < 0)
      pos += 720;
    
    dotFive = pos & 1;
    pos >>= 1;

    glyphs[0] = glyphs[1] = glyphs[2] = 0;
    for (unsigned i = 0; i < 3; ++i) {
      if (i > 0 && pos == 0)
        break;

      glyphs[2 - i] = 4 + (pos % 10);
      pos /= 10;
    }

    // Draw fraction
    glyphs[3] = dotFive ? 9 : 4;
  } else {
    for (unsigned i = 0; i < 4; ++i)
      glyphs[i] = 17;
  }

  // Draw point
  g_oled.drawBitmap(x0 + 4 * 20, y0, g_AzElGlyphs + 14 * 60, 20, 24);

  // Draw fraction
  g_oled.drawBitmap(x0 + 5 * 20, y0, g_AzElGlyphs + glyphs[3] * 60, 20, 24);

  // Draw glyphs of the current position
  for (unsigned i = 0; i < 3; ++i)
    if (glyphs[i] > 0)
      g_oled.drawBitmap(x0 + (i + 1) * 20, y0, g_AzElGlyphs + ((int) glyphs[i]) * 60, 20, 24);

  auto state = motor.state();
  auto reason = motor.reason();

  // Draw motor state
  if (!motor.initialized) {
    g_oled.setCursorXY(x0, y0 + 0);
    g_oled.print("NC");
  } else {
    const char *err = nullptr;
    unsigned int ticks;

    switch (state) {
      case Finalized:
        reason = motor.popStopReason();
        
      case Idle:
        switch (reason) {
          case Timeout:
            err = "ERR";
            break;

          case LimitSwitch:
            err = "STP";
            break;

          case Aborted:
            err = "ABT";
            break;

          case OverCurrent:
            err = "OVR";
            break;
        }

        if (err != nullptr && m_blink) {
          g_oled.setCursorXY(x0, y0 + 0);
          g_oled.print(err);
        }

        break;

      case Running:
        ticks = motor.currSpeed() / 32;
        for (unsigned i = 0; i < (1 + ticks); ++i)
          g_oled.line(x0 + i * 2, y0, x0 + i * 2, y0 + 7, 1);
        break;
    }
  }
  
  g_oled.setCursorXY(x0, y0 + 14);
  g_oled.print(motor.name());
  g_oled.print(":");
}

void
OLEDUI::drawRotorContext(RotorContext &ctx)
{
  g_oled.clear();
  g_oled.home();

  drawMotorState(ctx.azimuth, 0,  8);
  drawMotorState(ctx.elevation, 0, 40);

  g_oled.update();
}

void
OLEDUI::clock()
{
  m_blink = !m_blink;
}

/*
void
scani2c()
{
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
}*/
