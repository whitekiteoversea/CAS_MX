/*
This file is part of CanFestival, a library implementing CanOpen Stack.

Copyright (C): Edouard TISSERANT and Francis DUPIN
AVR Port: Andreas GLAUSER and Peter CHRISTEN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/


#ifndef __CAN_CANFESTIVAL__
#define __CAN_CANFESTIVAL__

#include "applicfg.h"
#include "data.h"

// ���� STM32F4 �� SRAM �� Flash �ĵ�ַ��Χ
#define SRAMADDR_START       0x20000000
#define SRAMADDR_END         0x2002FFFF
#define FLASHADDR_START      0x08000000
#define FLASHADDR_END        0x080FFFFF

// ---------  to be called by user app ---------
void initTimer(void);
UNS8 canSend(CAN_PORT notused, Message *m);
UNS8 canChangeBaudRate(CAN_PORT port, char* baud);

unsigned char isValidAddress(void *ptr);

#endif
