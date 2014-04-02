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
#include "rfm12llc.h"

void rfm12_init(uint16_t (*pRFM12_phy_exchangeWord)(uint16_t word), void (*pRFM12_phy_SPISelect) (void), void (*pRFM12_phy_SPIDeselect)(void), void (**prfm12_phy_int_vect)(void), bool (*prfm12_llc_nextLayerReceiveCallback)(uint8_t data, bool lastbyte), uint8_t (*prfm12_llc_nextLayerTransmitCallback)(void), uint16_t pownAddr)
{
  rfm12_phy_init(pRFM12_phy_exchangeWord, pRFM12_phy_SPISelect, pRFM12_phy_SPIDeselect, prfm12_phy_int_vect);
  
  rfm12_mac_init();
  
  rfm12_llc_init(prfm12_llc_nextLayerReceiveCallback, prfm12_llc_nextLayerTransmitCallback, pownAddr);
  
  
  
  rfm12_phy_setConf( true, true, FREQBAND_433MHz, CAP12_5pf);
  
  //enable everything except additional module features
  rfm12_phy_setPowerManagement( true, true, true, false, false, false);
  
  rfm12_phy_setDataFilter(true, false, false, 4);
  
  rfm12_phy_setAFC(AUTOMODE_RECV, RANGELIMIT_3toMINUS4, false, true, true, true);
  
  rfm12_mac_setChannel(1, VDI_FAST, LNA_0, RSSI_MINUS97, OUTPWR_0);
  
}