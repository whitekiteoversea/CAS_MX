#include "canfestival_master.h"

/**************************************************************************/
/* Declaration of mapped variables                                        */
/**************************************************************************/
INTEGER32 D0 = 0x0;		/* Mapped at index 0x2000, subindex 0x00 */
UNS32 D1 = 0x0;		/* Mapped at index 0x2001, subindex 0x00 */
UNS32 D2 = 0x0;		/* Mapped at index 0x2002, subindex 0x00 */
INTEGER16 Current_actual_value = 0x0;		/* Mapped at index 0x6078, subindex 0x00 */
INTEGER32 Target_velocity = 0x0;		/* Mapped at index 0x60FF, subindex 0x00 */


#define Transmission_Type 0x25  // 0xff:是event定时器模式

/**************************************************************************/
/* Declaration of value range types                                       */
/**************************************************************************/

#define valueRange_EMC 0x9F /* Type for index 0x1003 subindex 0x00 (only set of value 0 is possible) */
UNS32 Master_valueRangeTest (UNS8 typeValue, void * value)
{
  switch (typeValue) {
    case valueRange_EMC:
      if (*(UNS8*)value != (UNS8)0) return OD_VALUE_RANGE_EXCEEDED;
      break;
  }
  return 0;
}

/**************************************************************************/
/* The node id                                                            */
/**************************************************************************/
/* node_id default value.*/
UNS8 Master_bDeviceNodeId = 0x00;

/**************************************************************************/
/* Array of message processing information */

const UNS8 Master_iam_a_slave = 0;

TIMER_HANDLE Master_heartBeatTimers[1];

/*
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

                               OBJECT DICTIONARY

$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
*/

/* index 0x1000 :   Device Type. */
                    UNS32 Master_obj1000 = 0x0;	/* 0 */
                    subindex Master_Index1000[] =
                     {
                       { RO, uint32, sizeof (UNS32), (void*)&Master_obj1000  }
                     };

/* index 0x1001 :   Error Register. */
                    UNS8 Master_obj1001 = 0x0;	/* 0 */
                    subindex Master_Index1001[] =
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Master_obj1001  }
                     };

/* index 0x1003 :   Pre-defined Error Field */
                    UNS8 Master_highestSubIndex_obj1003 = 0; /* number of subindex - 1*/
                    UNS32 Master_obj1003[] =
                    {
                      0x0	/* 0 */
                    };
                    subindex Master_Index1003[] =
                     {
                       { RW, valueRange_EMC, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1003  },
                       { RO, uint32, sizeof (UNS32), (void*)&Master_obj1003[0]  }
                     };

/* index 0x1005 :   SYNC COB ID. */
                    UNS32 Master_obj1005 = 0x40000080;	/* 128 */
                    subindex Master_Index1005[] =
                     {
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1005  }
                     };

/* index 0x1006 :   Communication / Cycle Period. */
                    UNS32 Master_obj1006 = 1000;	// Synchro Cycle Period = 1ms
                    subindex Master_Index1006[] =
                     {
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1006  }
                     };

/* index 0x100C :   Guard Time */
                    UNS16 Master_obj100C = 0x0;   /* 0 */

/* index 0x100D :   Life Time Factor */
                    UNS8 Master_obj100D = 0x0;   /* 0 */

/* index 0x1014 :   Emergency COB ID. */
                    UNS32 Master_obj1014 = 0x80;	/* 128 */
                    subindex Master_Index1014[] =
                     {
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1014  }
                     };

/* index 0x1016 :   Consumer Heartbeat Time */
                    UNS8 Master_highestSubIndex_obj1016 = 0;
                    UNS32 Master_obj1016[]={0};

/* index 0x1017 :   Producer Heartbeat Time */
                    UNS16 Master_obj1017 = 0x0;   /* 0 */

/* index 0x1018 :   Identity. */
                    UNS8 Master_highestSubIndex_obj1018 = 4; /* number of subindex - 1*/
                    UNS32 Master_obj1018_Vendor_ID = 0x0;	/* 0 */
                    UNS32 Master_obj1018_Product_Code = 0x0;	/* 0 */
                    UNS32 Master_obj1018_Revision_Number = 0x0;	/* 0 */
                    UNS32 Master_obj1018_Serial_Number = 0x0;	/* 0 */
                    subindex Master_Index1018[] =
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1018  },
                       { RO, uint32, sizeof (UNS32), (void*)&Master_obj1018_Vendor_ID  },
                       { RO, uint32, sizeof (UNS32), (void*)&Master_obj1018_Product_Code  },
                       { RO, uint32, sizeof (UNS32), (void*)&Master_obj1018_Revision_Number  },
                       { RO, uint32, sizeof (UNS32), (void*)&Master_obj1018_Serial_Number  }
                     };

/* index 0x1019 :   Synchronous counter overflow value. */
                    UNS8 Master_obj1019 = 0x4;	/* 4 */
                    subindex Master_Index1019[] =
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1019  }
                     };

/* index 0x1280 :   Client SDO 1 Parameter. */
                    UNS8 Master_highestSubIndex_obj1280 = 3; /* number of subindex - 1*/
                    UNS32 Master_obj1280_COB_ID_Client_to_Server_Transmit_SDO = 0x601;	/* 1537 */
                    UNS32 Master_obj1280_COB_ID_Server_to_Client_Receive_SDO = 0x581;	/* 1409 */
                    UNS8 Master_obj1280_Node_ID_of_the_SDO_Server = 0x1;	/* 1 */
                    subindex Master_Index1280[] =
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1280  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1280_COB_ID_Client_to_Server_Transmit_SDO  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1280_COB_ID_Server_to_Client_Receive_SDO  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1280_Node_ID_of_the_SDO_Server  }
                     };

/* index 0x1281 :   Client SDO 2 Parameter. */
                    UNS8 Master_highestSubIndex_obj1281 = 3; /* number of subindex - 1*/
                    UNS32 Master_obj1281_COB_ID_Client_to_Server_Transmit_SDO = 0x602;	/* 1538 */
                    UNS32 Master_obj1281_COB_ID_Server_to_Client_Receive_SDO = 0x582;	/* 1410 */
                    UNS8 Master_obj1281_Node_ID_of_the_SDO_Server = 0x2;	/* 2 */
                    subindex Master_Index1281[] =
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1281  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1281_COB_ID_Client_to_Server_Transmit_SDO  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1281_COB_ID_Server_to_Client_Receive_SDO  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1281_Node_ID_of_the_SDO_Server  }
                     };

/* index 0x1282 :   Client SDO 3 Parameter. */
                    UNS8 Master_highestSubIndex_obj1282 = 3; /* number of subindex - 1*/
                    UNS32 Master_obj1282_COB_ID_Client_to_Server_Transmit_SDO = 0x603;	/* 1539 */
                    UNS32 Master_obj1282_COB_ID_Server_to_Client_Receive_SDO = 0x583;	/* 1411 */
                    UNS8 Master_obj1282_Node_ID_of_the_SDO_Server = 0x3;	/* 3 */
                    subindex Master_Index1282[] =
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1282  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1282_COB_ID_Client_to_Server_Transmit_SDO  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1282_COB_ID_Server_to_Client_Receive_SDO  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1282_Node_ID_of_the_SDO_Server  }
                     };

/* index 0x1283 :   Client SDO 4 Parameter. */
                    UNS8 Master_highestSubIndex_obj1283 = 3; /* number of subindex - 1*/
                    UNS32 Master_obj1283_COB_ID_Client_to_Server_Transmit_SDO = 0x604;	/* 1540 */
                    UNS32 Master_obj1283_COB_ID_Server_to_Client_Receive_SDO = 0x584;	/* 1412 */
                    UNS8 Master_obj1283_Node_ID_of_the_SDO_Server = 0x4;	/* 4 */
                    subindex Master_Index1283[] =
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1283  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1283_COB_ID_Client_to_Server_Transmit_SDO  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1283_COB_ID_Server_to_Client_Receive_SDO  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1283_Node_ID_of_the_SDO_Server  }
                     };

/* index 0x1284 :   Client SDO 5 Parameter. */
                    UNS8 Master_highestSubIndex_obj1284 = 3; /* number of subindex - 1*/
                    UNS32 Master_obj1284_COB_ID_Client_to_Server_Transmit_SDO = 0x605;	/* 1541 */
                    UNS32 Master_obj1284_COB_ID_Server_to_Client_Receive_SDO = 0x585;	/* 1413 */
                    UNS8 Master_obj1284_Node_ID_of_the_SDO_Server = 0x5;	/* 5 */
                    subindex Master_Index1284[] =
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1284  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1284_COB_ID_Client_to_Server_Transmit_SDO  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1284_COB_ID_Server_to_Client_Receive_SDO  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1284_Node_ID_of_the_SDO_Server  }
                     };

/* index 0x1285 :   Client SDO 6 Parameter. */
                    UNS8 Master_highestSubIndex_obj1285 = 3; /* number of subindex - 1*/
                    UNS32 Master_obj1285_COB_ID_Client_to_Server_Transmit_SDO = 0x606;	/* 1542 */
                    UNS32 Master_obj1285_COB_ID_Server_to_Client_Receive_SDO = 0x586;	/* 1414 */
                    UNS8 Master_obj1285_Node_ID_of_the_SDO_Server = 0x6;	/* 6 */
                    subindex Master_Index1285[] =
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1285  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1285_COB_ID_Client_to_Server_Transmit_SDO  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1285_COB_ID_Server_to_Client_Receive_SDO  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1285_Node_ID_of_the_SDO_Server  }
                     };

/* index 0x1400 :   Receive PDO 1 Parameter. */
                    UNS8 Master_highestSubIndex_obj1400 = 2; /* number of subindex - 1*/
                    UNS32 Master_obj1400_COB_ID_used_by_PDO = 0x208;	/* 512 */
                    UNS8 Master_obj1400_Transmission_Type = 0x01;	/* 1 */
                    subindex Master_Index1400[] =
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1400  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1400_COB_ID_used_by_PDO  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1400_Transmission_Type  }
                     };

/* index 0x1401 :   Receive PDO 2 Parameter. */
                    UNS8 Master_highestSubIndex_obj1401 = 6; /* number of subindex - 1*/
                    UNS32 Master_obj1401_COB_ID_used_by_PDO = 0x308;	/* 768 */
                    UNS8 Master_obj1401_Transmission_Type = 0x00;	/* 0 */
                    UNS16 Master_obj1401_Inhibit_Time = 0x0;	/* 0 */
                    UNS8 Master_obj1401_Compatibility_Entry = 0x0;	/* 0 */
                    UNS16 Master_obj1401_Event_Timer = 0x0;	/* 0 */
                    UNS8 Master_obj1401_SYNC_start_value = 0x0;	/* 0 */
                    subindex Master_Index1401[] =
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1401  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1401_COB_ID_used_by_PDO  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1401_Transmission_Type  },
                       { RW, uint16, sizeof (UNS16), (void*)&Master_obj1401_Inhibit_Time  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1401_Compatibility_Entry  },
                       { RW, uint16, sizeof (UNS16), (void*)&Master_obj1401_Event_Timer  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1401_SYNC_start_value  }
                     };

/* index 0x1402 :   Receive PDO 3 Parameter. */
                    UNS8 Master_highestSubIndex_obj1402 = 6; /* number of subindex - 1*/
                    UNS32 Master_obj1402_COB_ID_used_by_PDO = 0x80000408;	/* 1024 */
                    UNS8 Master_obj1402_Transmission_Type = 0x00;	/* 0 */
                    UNS16 Master_obj1402_Inhibit_Time = 0x0;	/* 0 */
                    UNS8 Master_obj1402_Compatibility_Entry = 0x0;	/* 0 */
                    UNS16 Master_obj1402_Event_Timer = 0x0;	/* 0 */
                    UNS8 Master_obj1402_SYNC_start_value = 0x0;	/* 0 */
                    subindex Master_Index1402[] =
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1402  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1402_COB_ID_used_by_PDO  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1402_Transmission_Type  },
                       { RW, uint16, sizeof (UNS16), (void*)&Master_obj1402_Inhibit_Time  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1402_Compatibility_Entry  },
                       { RW, uint16, sizeof (UNS16), (void*)&Master_obj1402_Event_Timer  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1402_SYNC_start_value  }
                     };

/* index 0x1403 :   Receive PDO 4 Parameter. */
                    UNS8 Master_highestSubIndex_obj1403 = 6; /* number of subindex - 1*/
                    UNS32 Master_obj1403_COB_ID_used_by_PDO = 0x80000508;	/* 1153 */
                    UNS8 Master_obj1403_Transmission_Type = 0x00;	/* 0 */
                    UNS16 Master_obj1403_Inhibit_Time = 0x0;	/* 0 */
                    UNS8 Master_obj1403_Compatibility_Entry = 0x0;	/* 0 */
                    UNS16 Master_obj1403_Event_Timer = 0x0;	/* 0 */
                    UNS8 Master_obj1403_SYNC_start_value = 0x0;	/* 0 */
                    subindex Master_Index1403[] =
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1403  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1403_COB_ID_used_by_PDO  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1403_Transmission_Type  },
                       { RW, uint16, sizeof (UNS16), (void*)&Master_obj1403_Inhibit_Time  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1403_Compatibility_Entry  },
                       { RW, uint16, sizeof (UNS16), (void*)&Master_obj1403_Event_Timer  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1403_SYNC_start_value  }
                     };

/* index 0x1600 :   Receive PDO 1 Mapping. */
                    UNS8 Master_highestSubIndex_obj1600 = 1; /* number of subindex - 1*/
                    UNS32 Master_obj1600[] =
                    {
                      0x60FF0020	/* 536870944 */
                    };
                    subindex Master_Index1600[] =
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1600  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1600[0]  }
                     };

/* index 0x1601 :   Receive PDO 2 Mapping. */
                    UNS8 Master_highestSubIndex_obj1601 = 1; /* number of subindex - 1*/
                    UNS32 Master_obj1601[] =
                    {
                      0x20010020	/* 536936480 */
                    };
                    subindex Master_Index1601[] =
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1601  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1601[0]  }
                     };

/* index 0x1602 :   Receive PDO 3 Mapping. */
                    UNS8 Master_highestSubIndex_obj1602 = 1; /* number of subindex - 1*/
                    UNS32 Master_obj1602[] =
                    {
                      0x20020020	/* 537002016 */
                    };
                    subindex Master_Index1602[] =
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1602  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1602[0]  }
                     };

/* index 0x1603 :   Receive PDO 4 Mapping. */
                    UNS8 Master_highestSubIndex_obj1603 = 1; /* number of subindex - 1*/
                    UNS32 Master_obj1603[] =
                    {
                      0x60780010	/* 1618477072 */
                    };
                    subindex Master_Index1603[] =
                    {
                       { RW, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1603  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1603[0]  }
                    };

/* index 0x1800 :   Transmit PDO 1 Parameter. */
                    UNS8 Master_highestSubIndex_obj1800 = 4; /* number of subindex - 1*/
                    UNS32 Master_obj1800_COB_ID_used_by_PDO =0x188; //0x80000188;	/*  */
                    UNS8 Master_obj1800_Transmission_Type = 0x1;	/* SYNC Send */
                    UNS16 Master_obj1800_Inhibit_Time = 0;	/* 0 */
                    UNS16 Master_obj1800_Event_Timer = 1000; // 1s  
                    subindex Master_Index1800[] =
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1800  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1800_COB_ID_used_by_PDO  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1800_Transmission_Type  },
                       { RW, uint16, sizeof (UNS16), (void*)&Master_obj1800_Inhibit_Time  },
                       { RW, uint16, sizeof (UNS16), (void*)&Master_obj1800_Event_Timer  },
                     };

/* index 0x1801 :   Transmit PDO 2 Parameter. */
                    UNS8 Master_highestSubIndex_obj1801 = 6; /* number of subindex - 1*/
                    UNS32 Master_obj1801_COB_ID_used_by_PDO = 0x80000288;//0x80000202;	/* 514 */
                    UNS8 Master_obj1801_Transmission_Type = Transmission_Type;	/* 255 */
                    UNS16 Master_obj1801_Inhibit_Time = 0;	/* 0 */
                    UNS8 Master_obj1801_Compatibility_Entry = 0x0;	/* 0 */
                    UNS16 Master_obj1801_Event_Timer = 1000;	/* 1rps */
                    UNS8 Master_obj1801_SYNC_start_value = 0x12;	/* 0 */
                    subindex Master_Index1801[] =
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1801  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1801_COB_ID_used_by_PDO  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1801_Transmission_Type  },
                       { RW, uint16, sizeof (UNS16), (void*)&Master_obj1801_Inhibit_Time  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1801_Compatibility_Entry  },
                       { RW, uint16, sizeof (UNS16), (void*)&Master_obj1801_Event_Timer  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1801_SYNC_start_value  }
                     };

/* index 0x1802 :   Transmit PDO 3 Parameter. */
                    UNS8 Master_highestSubIndex_obj1802 = 6; /* number of subindex - 1*/
                    UNS32 Master_obj1802_COB_ID_used_by_PDO = 0x80000388;	/* 515 */
                    UNS8 Master_obj1802_Transmission_Type = Transmission_Type;	/* 255 */
                    UNS16 Master_obj1802_Inhibit_Time = 0;	/* 1 */
                    UNS8 Master_obj1802_Compatibility_Entry = 0x0;	/* 0 */
                    UNS16 Master_obj1802_Event_Timer = 0;	/* 5 */
                    UNS8 Master_obj1802_SYNC_start_value = 0x13;	/* 0 */
                    subindex Master_Index1802[] =
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1802  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1802_COB_ID_used_by_PDO  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1802_Transmission_Type  },
                       { RW, uint16, sizeof (UNS16), (void*)&Master_obj1802_Inhibit_Time  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1802_Compatibility_Entry  },
                       { RW, uint16, sizeof (UNS16), (void*)&Master_obj1802_Event_Timer  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1802_SYNC_start_value  }
                     };

/* index 0x1803 :   Transmit PDO 4 Parameter. */
                    UNS8 Master_highestSubIndex_obj1803 = 6; /* number of subindex - 1*/
                    UNS32 Master_obj1803_COB_ID_used_by_PDO = 0x80000488;	/* 516 */
                    UNS8 Master_obj1803_Transmission_Type = Transmission_Type;	/* 255 */
                    UNS16 Master_obj1803_Inhibit_Time = 0;	/* 1 */
                    UNS8 Master_obj1803_Compatibility_Entry = 0x0;	/* 0 */
                    UNS16 Master_obj1803_Event_Timer = 0;	/* 5 */
                    UNS8 Master_obj1803_SYNC_start_value = 0x14;	/* 0 */
                    subindex Master_Index1803[] =
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1803  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1803_COB_ID_used_by_PDO  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1803_Transmission_Type  },
                       { RW, uint16, sizeof (UNS16), (void*)&Master_obj1803_Inhibit_Time  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1803_Compatibility_Entry  },
                       { RW, uint16, sizeof (UNS16), (void*)&Master_obj1803_Event_Timer  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1803_SYNC_start_value  }
                     };

/* index 0x1804 :   Transmit PDO 5 Parameter. */
                    UNS8 Master_highestSubIndex_obj1804 = 6; /* number of subindex - 1*/
                    UNS32 Master_obj1804_COB_ID_used_by_PDO = 0x00000205;	/* 517 */
                    UNS8 Master_obj1804_Transmission_Type = Transmission_Type;	/* 255 */
                    UNS16 Master_obj1804_Inhibit_Time = 0;	/* 1 */
                    UNS8 Master_obj1804_Compatibility_Entry = 0x0;	/* 0 */
                    UNS16 Master_obj1804_Event_Timer = 0;	/* 5 */
                    UNS8 Master_obj1804_SYNC_start_value = 0x15;	/* 0 */
                    subindex Master_Index1804[] =
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1804  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1804_COB_ID_used_by_PDO  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1804_Transmission_Type  },
                       { RW, uint16, sizeof (UNS16), (void*)&Master_obj1804_Inhibit_Time  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1804_Compatibility_Entry  },
                       { RW, uint16, sizeof (UNS16), (void*)&Master_obj1804_Event_Timer  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1804_SYNC_start_value  }
                     };

/* index 0x1805 :   Transmit PDO 6 Parameter. */
                    UNS8 Master_highestSubIndex_obj1805 = 6; /* number of subindex - 1*/
                    UNS32 Master_obj1805_COB_ID_used_by_PDO = 0x00000206;	/* 518 */
                    UNS8 Master_obj1805_Transmission_Type = Transmission_Type;	/* 255 */
                    UNS16 Master_obj1805_Inhibit_Time = 0;	/* 1 */
                    UNS8 Master_obj1805_Compatibility_Entry = 0x0;	/* 0 */
                    UNS16 Master_obj1805_Event_Timer = 0;	/* 5 */
                    UNS8 Master_obj1805_SYNC_start_value = 0x16;	/* 0 */
                    subindex Master_Index1805[] =
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1805  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1805_COB_ID_used_by_PDO  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1805_Transmission_Type  },
                       { RW, uint16, sizeof (UNS16), (void*)&Master_obj1805_Inhibit_Time  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1805_Compatibility_Entry  },
                       { RW, uint16, sizeof (UNS16), (void*)&Master_obj1805_Event_Timer  },
                       { RW, uint8, sizeof (UNS8), (void*)&Master_obj1805_SYNC_start_value  }
                     };

/* index 0x1A00 :   Transmit PDO 1 Mapping. */
                    UNS8 Master_highestSubIndex_obj1A00 = 1; /* number of subindex - 1*/
                    UNS32 Master_obj1A00[] =
                    {
                      	0x200B1910  // 相电流有效值 0.01A
                    };
                    subindex Master_Index1A00[] =
                    {
                       { RW, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1A00  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1A00[0]  }
                     };

/* index 0x1A01 :   Transmit PDO 2 Mapping. */
                    UNS8 Master_highestSubIndex_obj1A01 = 1; /* number of subindex - 1*/
                    UNS32 Master_obj1A01[] = {0x60FF0020, 0x606c0020};
                    subindex Master_Index1A01[] =
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1A01  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1A01[0]  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1A01[1]  }
                     };

/* index 0x1A02 :   Transmit PDO 3 Mapping. */
                    UNS8 Master_highestSubIndex_obj1A02 = 1; /* number of subindex - 1*/
                    UNS32 Master_obj1A02[] =
                    {
                      0x60FF0020	/* 1627324448 */
                    };
                    subindex Master_Index1A02[] =
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1A02  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1A02[0]  }
                     };

/* index 0x1A03 :   Transmit PDO 4 Mapping. */
                    UNS8 Master_highestSubIndex_obj1A03 = 1; /* number of subindex - 1*/
                    UNS32 Master_obj1A03[] =
                    {
                      0x60FF0020	/* 1627324448 */
                    };
                    subindex Master_Index1A03[] =
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1A03  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1A03[0]  }
                     };

/* index 0x1A04 :   Transmit PDO 5 Mapping. */
                    UNS8 Master_highestSubIndex_obj1A04 = 1; /* number of subindex - 1*/
                    UNS32 Master_obj1A04[] =
                    {
                      0x60FF0020	/* 1627324448 */
                    };
                    subindex Master_Index1A04[] =
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1A04  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1A04[0]  }
                     };

/* index 0x1A05 :   Transmit PDO 6 Mapping. */
                    UNS8 Master_highestSubIndex_obj1A05 = 1; /* number of subindex - 1*/
                    UNS32 Master_obj1A05[] =
                    {
                      0x60FF0020	/* 1627324448 */
                    };
                    subindex Master_Index1A05[] =
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&Master_highestSubIndex_obj1A05  },
                       { RW, uint32, sizeof (UNS32), (void*)&Master_obj1A05[0]  }
                     };

/* index 0x2000 :   Mapped variable D0 */
                    subindex Master_Index2000[] =
                     {
                       { RW, int32, sizeof (INTEGER32), (void*)&D0  }
                     };

/* index 0x2001 :   Mapped variable D1 */
                    subindex Master_Index2001[] =
                     {
                       { RW, uint32, sizeof (UNS32), (void*)&D1  }
                     };

/* index 0x2002 :   Mapped variable D2 */
                    subindex Master_Index2002[] =
                     {
                       { RW, uint32, sizeof (UNS32), (void*)&D2  }
                     };

/* index 0x6078 :   Mapped variable Current actual value */
                    subindex Master_Index6078[] =
                     {
                       { RO, int16, sizeof (INTEGER16), (void*)&Current_actual_value  }
                     };

/* index 0x60FF :   Mapped variable Target velocity */
                    subindex Master_Index60FF[] =
                     {
                       { RW, int32, sizeof (INTEGER32), (void*)&Target_velocity  }
                     };

/**************************************************************************/
/* Declaration of pointed variables                                       */
/**************************************************************************/

const indextable Master_objdict[] =
{
  { (subindex*)Master_Index1000,sizeof(Master_Index1000)/sizeof(Master_Index1000[0]), 0x1000},
  { (subindex*)Master_Index1001,sizeof(Master_Index1001)/sizeof(Master_Index1001[0]), 0x1001},
  { (subindex*)Master_Index1005,sizeof(Master_Index1005)/sizeof(Master_Index1005[0]), 0x1005},
  { (subindex*)Master_Index1006,sizeof(Master_Index1006)/sizeof(Master_Index1006[0]), 0x1006},
  { (subindex*)Master_Index1014,sizeof(Master_Index1014)/sizeof(Master_Index1014[0]), 0x1014},
  { (subindex*)Master_Index1018,sizeof(Master_Index1018)/sizeof(Master_Index1018[0]), 0x1018},
  { (subindex*)Master_Index1019,sizeof(Master_Index1019)/sizeof(Master_Index1019[0]), 0x1019},
  { (subindex*)Master_Index1280,sizeof(Master_Index1280)/sizeof(Master_Index1280[0]), 0x1280},
  { (subindex*)Master_Index1281,sizeof(Master_Index1281)/sizeof(Master_Index1281[0]), 0x1281},
  { (subindex*)Master_Index1282,sizeof(Master_Index1282)/sizeof(Master_Index1282[0]), 0x1282},
  { (subindex*)Master_Index1283,sizeof(Master_Index1283)/sizeof(Master_Index1283[0]), 0x1283},
  { (subindex*)Master_Index1284,sizeof(Master_Index1284)/sizeof(Master_Index1284[0]), 0x1284},
  { (subindex*)Master_Index1285,sizeof(Master_Index1285)/sizeof(Master_Index1285[0]), 0x1285},
  { (subindex*)Master_Index1400,sizeof(Master_Index1400)/sizeof(Master_Index1400[0]), 0x1400},
  { (subindex*)Master_Index1401,sizeof(Master_Index1401)/sizeof(Master_Index1401[0]), 0x1401},
  { (subindex*)Master_Index1402,sizeof(Master_Index1402)/sizeof(Master_Index1402[0]), 0x1402},
  { (subindex*)Master_Index1403,sizeof(Master_Index1403)/sizeof(Master_Index1403[0]), 0x1403},
  { (subindex*)Master_Index1600,sizeof(Master_Index1600)/sizeof(Master_Index1600[0]), 0x1600},
  { (subindex*)Master_Index1601,sizeof(Master_Index1601)/sizeof(Master_Index1601[0]), 0x1601},
  { (subindex*)Master_Index1602,sizeof(Master_Index1602)/sizeof(Master_Index1602[0]), 0x1602},
  { (subindex*)Master_Index1603,sizeof(Master_Index1603)/sizeof(Master_Index1603[0]), 0x1603},
  { (subindex*)Master_Index1800,sizeof(Master_Index1800)/sizeof(Master_Index1800[0]), 0x1800},
  { (subindex*)Master_Index1801,sizeof(Master_Index1801)/sizeof(Master_Index1801[0]), 0x1801},
  { (subindex*)Master_Index1802,sizeof(Master_Index1802)/sizeof(Master_Index1802[0]), 0x1802},
  { (subindex*)Master_Index1803,sizeof(Master_Index1803)/sizeof(Master_Index1803[0]), 0x1803},
  { (subindex*)Master_Index1804,sizeof(Master_Index1804)/sizeof(Master_Index1804[0]), 0x1804},
  { (subindex*)Master_Index1805,sizeof(Master_Index1805)/sizeof(Master_Index1805[0]), 0x1805},
  { (subindex*)Master_Index1A00,sizeof(Master_Index1A00)/sizeof(Master_Index1A00[0]), 0x1A00},
  { (subindex*)Master_Index1A01,sizeof(Master_Index1A01)/sizeof(Master_Index1A01[0]), 0x1A01},
  { (subindex*)Master_Index1A02,sizeof(Master_Index1A02)/sizeof(Master_Index1A02[0]), 0x1A02},
  { (subindex*)Master_Index1A03,sizeof(Master_Index1A03)/sizeof(Master_Index1A03[0]), 0x1A03},
  { (subindex*)Master_Index1A04,sizeof(Master_Index1A04)/sizeof(Master_Index1A04[0]), 0x1A04},
  { (subindex*)Master_Index1A05,sizeof(Master_Index1A05)/sizeof(Master_Index1A05[0]), 0x1A05},
  { (subindex*)Master_Index2000,sizeof(Master_Index2000)/sizeof(Master_Index2000[0]), 0x2000},
  { (subindex*)Master_Index2001,sizeof(Master_Index2001)/sizeof(Master_Index2001[0]), 0x2001},
  { (subindex*)Master_Index2002,sizeof(Master_Index2002)/sizeof(Master_Index2002[0]), 0x2002},
  { (subindex*)Master_Index6078,sizeof(Master_Index6078)/sizeof(Master_Index6078[0]), 0x6078},
  { (subindex*)Master_Index60FF,sizeof(Master_Index60FF)/sizeof(Master_Index60FF[0]), 0x60FF},
};

const indextable * Master_scanIndexOD (CO_Data *d, UNS16 wIndex, UNS32 * errorCode)
{
	(void)d;
	int i;
	switch(wIndex){
		case 0x1000: i = 0;break;
		case 0x1001: i = 1;break;
		case 0x1005: i = 2;break;
		case 0x1006: i = 3;break;
		case 0x1007: i = 4;break;
		case 0x1014: i = 5;break;
		case 0x1018: i = 6;break;
		case 0x1019: i = 7;break;
		case 0x1280: i = 8;break;
		case 0x1281: i = 9;break;
		case 0x1282: i = 10;break;
		case 0x1283: i = 11;break;
		case 0x1284: i = 12;break;
		case 0x1285: i = 13;break;
		case 0x1400: i = 14;break;
		case 0x1401: i = 15;break;
		case 0x1402: i = 16;break;
		case 0x1403: i = 17;break;
		case 0x1600: i = 18;break;
		case 0x1601: i = 19;break;
		case 0x1602: i = 20;break;
		case 0x1603: i = 21;break;
		case 0x1800: i = 22;break;
		case 0x1801: i = 23;break;
		case 0x1802: i = 24;break;
		case 0x1803: i = 25;break;
		case 0x1804: i = 26;break;
		case 0x1805: i = 27;break;
		case 0x1A00: i = 28;break;
		case 0x1A01: i = 29;break;
		case 0x1A02: i = 30;break;
		case 0x1A03: i = 31;break;
		case 0x1A04: i = 32;break;
		case 0x1A05: i = 33;break;
		case 0x2000: i = 34;break;
		case 0x2001: i = 35;break;
		case 0x2002: i = 36;break;
		case 0x6078: i = 37;break;
		case 0x60FF: i = 38;break;
		default:
			*errorCode = OD_NO_SUCH_OBJECT;
			return NULL;
	}
	*errorCode = OD_SUCCESSFUL;
	return &Master_objdict[i];
}

/*
 * To count at which received SYNC a PDO must be sent.
 * Even if no pdoTransmit are defined, at least one entry is computed
 * for compilations issues.
 */
s_PDO_status Master_PDO_status[6] = {s_PDO_status_Initializer,s_PDO_status_Initializer,s_PDO_status_Initializer,s_PDO_status_Initializer,s_PDO_status_Initializer,s_PDO_status_Initializer};

const quick_index Master_firstIndex = {
  0, /* SDO_SVR */
  8, /* SDO_CLT */
  14, /* PDO_RCV */
  18, /* PDO_RCV_MAP */
  22, /* PDO_TRS */
  28 /* PDO_TRS_MAP */
};

const quick_index Master_lastIndex = {
  0, /* SDO_SVR */
  13, /* SDO_CLT */
  17, /* PDO_RCV */
  21, /* PDO_RCV_MAP */
  27, /* PDO_TRS */
  33 /* PDO_TRS_MAP */
};

const UNS16 Master_ObjdictSize = sizeof(Master_objdict)/sizeof(Master_objdict[0]);

CO_Data masterObjdict_Data = CANOPEN_NODE_DATA_INITIALIZER(Master); 
