/*
  xdrv_77_serial_uhf.ino - UART UHF reader support

  Copyright (C) 2024 Leonid Myravjev

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_UHF_SERIAL

#define XDRV_77           77

#include <TasmotaSerial.h>

struct {
  bool active = false;
  uint8_t tx = 0;                      // GPIO for Serial Tx
  uint8_t rx = 0;                      // GPIO for Serial Rx
  //TasmotaSerial serial;                     // TasmotaSerial instance to communicate with SC18IM704
} UHF_Serial;

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/


void UHFSerialInit(void) {
  uint32_t bus=0;

  if (PinUsed(GPIO_UHF_SER_TX, bus) && PinUsed(GPIO_UHF_SER_RX, bus)) {
    UHF_Serial.tx = Pin(GPIO_UHF_SER_TX, bus);
    UHF_Serial.rx = Pin(GPIO_UHF_SER_RX, bus);
    UHF_Serial.active = true;
  }
}

void UHFSerialSecond(void) {

}


bool Xdrv77(uint32_t function) {
  bool result = false;
  
  if (!UHF_Serial.active) {
    if (function==FUNC_INIT) UHFSerialInit();
  } else {
    switch (function) {
      case FUNC_EVERY_SECOND:
        UHFSerialSecond();
        break;
      case FUNC_INIT:
        UHFSerialInit();
        break;
    }
  }
  return result;
}

#endif  // USE_UHF_SERIAL

