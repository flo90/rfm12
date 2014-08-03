/*
    This file is part of the rfm12 driver project.
    Copyright (C) 2013  Florian Menne (florianmenne@t-online.de)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see [http://www.gnu.org/licenses/].
*/

#include "rfm12.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include <inttypes.h>
#include <stdbool.h>

#include "rfm12phy.h"
#include "rfm12mac.h"

void rfm12_init(RFM12_PHY_FUNCPTR_t pfuncptr, uint16_t pownAddr)
{
  rfm12_phy_init(pfuncptr);
  
  rfm12_mac_init();
  
  rfm12_mac_setAddr(pownAddr);
  
  rfm12_phy_setConf( true, true, FREQBAND_433MHz, CAP12_5pf);
  
  //enable everything except additional module features
  rfm12_phy_setPowerManagement( true, true, true, false, false, false);
  
  rfm12_phy_setDataFilter(true, false, false, 4);
  
  rfm12_phy_setAFC(AUTOMODE_RECV, RANGELIMIT_3toMINUS4, false, true, true, true);
  
  rfm12_mac_setChannel(1, VDI_FAST, LNA_0, RSSI_MINUS97, OUTPWR_0);
  
  rfm12_phy_modeRX();
  
}