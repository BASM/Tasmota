/*
  xsns_88_ru5300.ino - Support for Chafon CF-RU5300 UHR Tag Reader on Tasmota

  Copyright (C) 2021 Myravjev Leonid

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

#ifdef USE_RU5300
/*********************************************************************************************\
 * Read random tag, and check password
 *
\*********************************************************************************************/

#define XSNS_88            88

//#define RU5300_BAUDRATE   115200
#define RU5300_BAUDRATE   9600
#define RU5300_RECVWAIT   10   //cycles-- value * 100 ms
#define RU5300_ANSSIZE    20

#include <TasmotaSerial.h>
#include "WiFiClientSecureLightBearSSL.h"   // needs to be before "ESP8266WiFi.h" to avoid conflict with Arduino headers
TasmotaSerial *RU5300Serial = nullptr;

enum {
  RU5300_STAGE_SCANTAG , // query tag
  RU5300_STAGE_RECVSTAG, // check pass
  RU5300_STAGE_CHKPTAG , //
  RU5300_STAGE_RECVPTAG
};

struct {
  uint8_t  stage;
  uint8_t  recv_try = 0;
  uint8_t  recv_idx;
  uint8_t  recv[32];
  union {
    uint8_t  b[8];
    uint64_t TID;
  };
  uint32_t LASTTID;
} RU5300;

typedef struct _tid_ {
  uint8_t  tclass;
  uint8_t  manuf_hi;
  uint8_t  model:4;
  uint8_t  manuf_lo:4;
  union {
    struct {
      uint8_t  vermin:4;
      uint8_t  vermaj:4;
    };
    uint8_t model_lo;
  };
} uhrtaginfo_t;

#define htobe16(v) __builtin_bswap16(v)
#define htobe32(v) __builtin_bswap32(v)
#define htobe64(v) __builtin_bswap64(v)

void RU5300Init() {
  if (PinUsed(GPIO_RU5300_RX)) {
    RU5300Serial = new TasmotaSerial(Pin(GPIO_RU5300_RX), Pin(GPIO_RU5300_TX), 1, 64);
    if (RU5300Serial->begin(RU5300_BAUDRATE)) {//FIXME 8N1 ??
      if (RU5300Serial->hardwareSerial()) {
        ClaimSerial();
      }
    }
  }
  RU5300.stage=RU5300_STAGE_SCANTAG;
  RU5300.TID=0x00;
  RU5300.LASTTID=0x00;
}

static uint8_t RU5300Crc(uint8_t *buff, int len) {
  int     i;
  uint8_t uSum=0;

  for (i=0; i<len; i++)  uSum+=buff[i];
  uSum=(~uSum)+1;
  return uSum;
}


static int RU5300WriteLenAndCRC(void *start, void *pend) {
  uint16_t len;
#pragma pack(push,1)
  typedef struct {
    uint16_t head;
    uint16_t len;
    uint8_t  data[];
  } msg_t;
#pragma pack(pop)
  msg_t *msg=(msg_t*)start;
  len = (uintptr_t)pend - (uintptr_t)msg->data+1;
  msg->len=htobe16(len);
  msg->data[len-1]=RU5300Crc((uint8_t*)start, len+3);
  return len+4;
}

static void *RU5300AddInt8(void *vptr, uint8_t data) {
  uint8_t *p=(uint8_t*)vptr;
  p[0]=data;
  return (void*)&p[1];
}

static void *RU5300AddInt16(void *vptr, uint16_t data) {
  uint16_t *p=(uint16_t*)vptr;
  p[0]=htobe16(data);
  return (void*)&p[1];
}

static void *RU5300AddInt32(void *vptr, uint32_t data) {
  uint32_t *p=(uint32_t*)vptr;
  p[0]=htobe32(data);
  return (void*)&p[1];
}

#define CMD_READ_TAG_DATA 0x02
#define RU5300_MEM_TID    2
static void RU5300SendReadCardTID(uint32_t pass) {
  int          status,len;
  uint8_t      tbuff[256];
  void        *vmsg=tbuff;

  vmsg=RU5300AddInt16   (vmsg, 0x5357            );  // head
  vmsg=RU5300AddInt16   (vmsg, 0                 );  // len, fix later
  vmsg=RU5300AddInt8    (vmsg, 0xff              );  // reader address 0xff broadcast
  vmsg=RU5300AddInt8    (vmsg, CMD_READ_TAG_DATA );  // cmd

  vmsg=RU5300AddInt8    (vmsg, RU5300_MEM_TID    );  // memory type
  vmsg=RU5300AddInt8    (vmsg, 0x00              );  // start address
  vmsg=RU5300AddInt8    (vmsg, 0x06              );  // read len (words)
  vmsg=RU5300AddInt32   (vmsg, pass);           //password

  len = RU5300WriteLenAndCRC(tbuff, vmsg);

  RU5300Serial->flush();
  RU5300Serial->write(tbuff, len);
  AddLog(LOG_LEVEL_DEBUG, PSTR("SEND %i bytes"), len);
  AddLogBuffer(LOG_LEVEL_DEBUG, (uint8_t*)tbuff, len);
  //RU5300.block_time=RU5300_RECVWAIT;
  RU5300.recv_try=30;//RU5300_RECVWAIT;
  RU5300.recv_idx=0;
}


// return 2 -- unsupported passwords, send TID without read by password
//        1 -- wait more
//        0 -- success read
//       -1 -- error
static int RU5300RecvReadCardTID(int save) {
  int     j,i;
  uint8_t crc;
  union {
    uint32_t id;
    uint8_t  bid[4];
  } id;
  j=RU5300Serial->available();

  for (i=0; i<j; i++)
    RU5300.recv[RU5300.recv_idx++]=RU5300Serial->read();
  i=RU5300.recv_idx;

  if (RU5300.recv_try==0) return -1;
  else RU5300.recv_try--;

  if (i>0)
    AddLog(LOG_LEVEL_DEBUG, PSTR("RECV %i, wait %i bytes, try: %i"), i, RU5300_ANSSIZE, RU5300.recv_try);
  if (i<RU5300_ANSSIZE) return 1;

  AddLogBuffer(LOG_LEVEL_DEBUG, (uint8_t*)RU5300.recv, RU5300.recv_idx);

  RU5300Serial->flush();

  crc=RU5300Crc(RU5300.recv, i-1);
  if (crc!=RU5300.recv[i-1]) return -1;


  id.bid[3]=RU5300.recv[7];
  id.bid[2]=RU5300.recv[8];
  id.bid[1]=RU5300.recv[9];
  id.bid[0]=RU5300.recv[10];

  AddLog(LOG_LEVEL_DEBUG, PSTR("ID: %x (%x %x %x %x)"), id.id,
      id.bid[3],
      id.bid[2],
      id.bid[1],
      id.bid[0]);

  RU5300.b[7]=RU5300.recv[11];
  RU5300.b[6]=RU5300.recv[12];
  RU5300.b[5]=RU5300.recv[13];
  RU5300.b[4]=RU5300.recv[14];
  RU5300.b[3]=RU5300.recv[15];
  RU5300.b[2]=RU5300.recv[16];
  RU5300.b[1]=RU5300.recv[17];
  RU5300.b[0]=RU5300.recv[18];

  if (id.id==0xe2801160) return 2; //Monoza R6 unsupported password
  return 0;
}

static int RU5300CalcPass(uint32_t *acc) {
  br_sha256_context ctx;
  uint8_t       buffer[32];
  uint8_t       result[32];
  uint64_t      tidbe=htobe64(RU5300.TID);
  uint64_t      p1=htobe64(RU5300_MASTERPASS_HI);
  uint64_t      p2=htobe64(RU5300_MASTERPASS_LO);

  memcpy(&buffer[0] , &tidbe  , 8);
  memcpy(&buffer[8] , &p1     , 8);
  memcpy(&buffer[16], &p2     , 8);
  AddLogBuffer(LOG_LEVEL_DEBUG, buffer, 24);

  br_sha256_init  (&ctx);
  br_sha256_update(&ctx, buffer, 24);
  br_sha256_out   (&ctx, result);

  memcpy(acc , &result[4], 4);

  return 0;
}

#if 1
#pragma pack(push,1)
typedef struct {
  uint32_t  key;         // key for wiegand
  char      devname[32]; // device name
} udpmsg;
#pragma pack(pop)

static int RU5300SendUDP(uint32_t key) {
  WiFiUDP udp; 

  if (udp.beginPacket(IPAddress(255, 255, 255, 255), 12000)) {
    udpmsg msg;

    strncpy(msg.devname,SettingsText(SET_DEVICENAME),sizeof(msg.devname));
    msg.key=key;

    udp.write((char*)&msg,sizeof(udpmsg));
    udp.endPacket();
    AddLog(LOG_LEVEL_INFO, PSTR("UDP send %s 0x%8x (%i)"),msg.devname,key,key);
  } else {
    AddLog(LOG_LEVEL_INFO, PSTR("CAN'T SEND UDP PACKET"));
  }

  return 0;
}
#endif


void RU5300ScanForTag() {
  int          status;
  uhrtaginfo_t taginfo;
  uint32_t     pass;

  if (!RU5300Serial) { return; }

  switch (RU5300.stage) {
    case RU5300_STAGE_SCANTAG:

      AddLog(LOG_LEVEL_DEBUG, PSTR("SCAN"));
      RU5300SendReadCardTID(0x00000000);
      RU5300.stage=RU5300_STAGE_RECVSTAG;
      break;
    case RU5300_STAGE_RECVSTAG:

      //AddLog(LOG_LEVEL_DEBUG, PSTR("RECV"));
      status = RU5300RecvReadCardTID(1);
      if (status==1) break;//wait
      if ((status==0) || (status==2))
        RU5300.LASTTID=RU5300.TID;

      if (status==2) goto send_tid;
      if   (status==0) RU5300.stage=RU5300_STAGE_CHKPTAG;
      else             RU5300.stage=RU5300_STAGE_SCANTAG;//wrong
      break;
    case RU5300_STAGE_CHKPTAG:
      RU5300CalcPass(&pass);
      AddLog(LOG_LEVEL_INFO, PSTR("CHECK pass %lx for tid: %08lX (%li)"),(unsigned long)pass, (unsigned long)RU5300.TID, (unsigned long)RU5300.TID);
      RU5300SendReadCardTID(pass);
      RU5300.stage=RU5300_STAGE_RECVPTAG;
      break;
    case RU5300_STAGE_RECVPTAG:
      //AddLog(LOG_LEVEL_DEBUG, PSTR("RECV pass"));
      status = RU5300RecvReadCardTID(0);
      if (status==1) break;//wait
      if (status==0) {
        send_tid:
        AddLog(LOG_LEVEL_INFO, PSTR("TAG FOUND %08lX, old tid: %08lx"), (unsigned long)RU5300.LASTTID,(unsigned long)RU5300.TID);
        ResponseTime_P(PSTR(",\"RU5300\":{\"UID\":\"%08lX\"}}"), (unsigned long) RU5300.LASTTID);

        RU5300SendUDP(RU5300.LASTTID);

        //udp.beginPacketMulticast(MULTICAST_IP, MULTICAST_PORT, WiFi.localIP());
        //udp.write("HELLO", 5);
        //udp.endPacket();

        //MqttPublishTeleSensor();
      } else {
        AddLog(LOG_LEVEL_INFO, PSTR("Read error: %i, wrong password?"),status);
      };
      RU5300.stage=RU5300_STAGE_SCANTAG;
      break;
  }
}

#ifdef USE_WEBSERVER
void RU5300Show(void) {
  if (!RU5300Serial) { return; }
  WSContentSend_PD(PSTR("{s}RU5300 UID{m}%08X {e}"), RU5300.LASTTID);
}
#endif  // USE_WEBSERVER

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns88(byte function) {
  bool result = false;

  switch (function) {
    case FUNC_INIT:
      RU5300Init();
      break;
    //case FUNC_LOOP: //FUNC_EVERY_50_MSECOND:
    case FUNC_EVERY_50_MSECOND:
      RU5300ScanForTag();
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      RU5300Show();
      break;
#endif  // USE_WEBSERVER
  }
  return result;
}

#endif  // USE_RM5300
