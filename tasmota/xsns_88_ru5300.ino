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
#define htobe16(v) __builtin_bswap16(v)
#define htobe32(v) __builtin_bswap32(v)
#define htobe64(v) __builtin_bswap64(v)

#define be32toh(v) __builtin_bswap32(v)
#define be64toh(v) __builtin_bswap64(v)
/*********************************************************************************************\
 * Read random tag, and check password
 *
\*********************************************************************************************/

#define XSNS_88            88
#include <TasmotaSerial.h>
#define RECVBUFFSIZE      128
#define RU5300_BAUDRATE   115200


struct {
  uint8_t  recv_idx;
  uint8_t  recv[RECVBUFFSIZE];
} RU5300;

TasmotaSerial *RU5300Serial = nullptr;

enum {
  RU5300_STAGE_SCANTAG , // query tag
  RU5300_STAGE_RECVSTAG, // check pass
  RU5300_STAGE_CHKPTAG , //
  RU5300_STAGE_RECVPTAG
};

                // 20*50 -- 1 second
#define TIMEOUTCACHE (20)*3 //5 second


#define RU5300CACHELEN 32

struct {
  uint32_t tid;
  int      to;  //timeout
} RU5300t32to[RU5300CACHELEN];

void RU5300TimeoutIterate(void) {
  int i;

  for (i=0;i<RU5300CACHELEN;i++)
    if (RU5300t32to[i].to>0)
      RU5300t32to[i].to--;
}

int RU5300SendByTimeout(uint32_t t32) {
  int  i;
  bool found=false;
  int  line=0;

  for (i=0;i<RU5300CACHELEN;i++) {
    if (RU5300t32to[i].to==0) line=i;//Search last empty, if no empty return 0
    if (RU5300t32to[i].tid==t32) {
      found=1;
      line=i;
      break;
    }
  }

  if (found)
    if (RU5300t32to[line].to>0) {
      AddLog(LOG_LEVEL_INFO, PSTR("Not send, timeout: %i"), RU5300t32to[line].to);
      return 1;
    }

  RU5300t32to[line].tid=t32;
  RU5300t32to[line].to=TIMEOUTCACHE;
  RU5300SendUDP(t32);

  return 0;
}

static int RU5300SendConfirm(void) {
  uint8_t  confirmmsg[]= {0x53, 0x57, 0x00, 0x03, 0xFF, 0x45, 0x0F};

  AddLog(LOG_LEVEL_DEBUG, PSTR("Send confirm"));
  RU5300Serial->write(confirmmsg, 7);
  return 0;
}


void RU5300Init() {
  AddLog(LOG_LEVEL_INFO, PSTR("Init RU5300"));
  if (PinUsed(GPIO_RU5300_RX)) {
    RU5300Serial = new TasmotaSerial(Pin(GPIO_RU5300_RX), Pin(GPIO_RU5300_TX), 1, RECVBUFFSIZE);
    if (RU5300Serial->begin(RU5300_BAUDRATE)) {//FIXME 8N1 ??
      if (RU5300Serial->hardwareSerial()) {
        ClaimSerial();
        RU5300SendConfirm();
      }
    }
  }
  memset(RU5300t32to,0,sizeof(RU5300t32to));
  AddLog(LOG_LEVEL_DEBUG, PSTR("Init success, p: %p"), RU5300Serial);
}

static uint8_t RU5300Crc(uint8_t *buff, int len) {
  int     i;
  uint8_t uSum=0;

  for (i=0; i<len; i++)  uSum+=buff[i];
  uSum=(~uSum)+1;
  return uSum;
}

// Move recv buffer to right reader
static void RU5300MoveBuffer(void) {
  int i,len;

  len=RU5300.recv_idx;
  if (len==0) return;

  for (i=1; i<len; i++) {
    if (RU5300.recv[i]==0x43) {
      if ((i+1)>=len) {
        if (RU5300.recv[i+1]==0x54)
          break;
      } else break;
    }
  }
  len=len-i;
  if (len>0)
    memmove(&RU5300.recv[0], &RU5300.recv[i], len);
  RU5300.recv_idx=len;
  //hexdump("MOVE RESULT", RU5300.recv, len);
}

struct RU5300resptype{
  int      len;
  int      addr;
  int      cmd;
  int      status;
  uint8_t *data;
} ;

// Recive Response data block
// 
// * respinfo -- response info (set only if returned zero '0')
// 
// return 0 - recived valid data block
//        1 - wait data
//       -1 - data drop
//
static int RU5300Recv(struct RU5300resptype *respinfo) {
  int     i,j,len;
  uint8_t crc;

  j=RU5300Serial->available();

  for (i=0; i<j; i++) {
    RU5300.recv[RU5300.recv_idx++]=RU5300Serial->read();
    //AddLog(LOG_LEVEL_INFO, PSTR("I:%i, j:%i, recvidx: %i -- %x\n"),i,j, RU5300.recv_idx-1,RU5300.recv[RU5300.recv_idx-1]);
    if (RU5300.recv_idx>=RECVBUFFSIZE) break;
  }
  if (RU5300.recv_idx<7) return 1; //wait data

  // Format: 43 54 LEN ADDR ..... CRC
  if ((RU5300.recv[0]!=0x43)&&(RU5300.recv[1]!=0x54)) {
    AddLog(LOG_LEVEL_INFO, PSTR("Check header err\n"));
    RU5300MoveBuffer();
    return 1;
  }

  len=(RU5300.recv[2]<<8)|RU5300.recv[3];
  //len+=5;
  if (len>RECVBUFFSIZE) {
    RU5300MoveBuffer();
    AddLog(LOG_LEVEL_INFO, PSTR("Check len err, len: %i, buff len: %i"),len,RECVBUFFSIZE);
    return 1;
  }

  if ((len+4)>(RU5300.recv_idx)) return 1;

  crc=RU5300Crc(RU5300.recv, len+3);
  if (crc!=RU5300.recv[len+3]) {
    RU5300MoveBuffer();
    AddLog(LOG_LEVEL_INFO, PSTR("Check crc err %x, must %x"), crc, RU5300.recv[len+3]);
    return -1;
  }
  AddLog(LOG_LEVEL_DEBUG, PSTR("SUCCESS"));

  // SUCCESS recive
  respinfo->len=len;
  respinfo->addr=RU5300.recv[4];
  respinfo->cmd=RU5300.recv[5];
  respinfo->status=RU5300.recv[6];
  memmove(&RU5300.recv[0],&RU5300.recv[6],len);

  respinfo->data=&RU5300.recv[0];

  return 0;
}

enum {
 CMD_ACTIVE_DATA=0x45,
};


// return 2 -- unsupported passwords, send TID without read by password
//        1 -- wait more
//        0 -- success read
//       -1 -- error
static int RU5300RecvRead(uint32_t *EP, uint64_t *TID) {
  int      i,tags;
  struct RU5300resptype info;
  uint8_t  devsn[7];
  uint8_t  tid[64];
  uint8_t *p;

  if (RU5300Recv(&info)!=0) return 1;
  if (info.cmd!=CMD_ACTIVE_DATA) return 1;

  AddLogBuffer(LOG_LEVEL_DEBUG, info.data, info.len);

  memcpy(devsn, info.data, 7); //Copy serial
  tags=info.data[8];
  p=&info.data[9];

  for (i=0;i<tags;i++) {
    int tlen=p[0];
    int t   =p[1]; // type always must be 0x01
    int ant =p[2];
    int rssi=p[tlen-1];
    memcpy(tid, &p[3], tlen-4);
    //hexdump("TAG ID", tid, tlen-4);

    AddLog(LOG_LEVEL_DEBUG, PSTR("TAG: %i LEN: %i, RSSI: %i, ANT: %i"),i, tlen, rssi, ant);
    // RU5300 read always by one(1) tag on at a time
    if (i==0) {
      memcpy(EP ,&tid[0],4);
      memcpy(TID,&tid[4],8);
      *EP =be32toh(*EP);
      *TID=be64toh(*TID);
      break;
    }
    (void)t;
  }

  // optional
  RU5300SendConfirm();
  RU5300MoveBuffer();

  return 0;
}

void RU5300ScanForTag() {
  int          status;
  uint32_t     t32, t32xor;
  uint32_t     EP;
  uint64_t     TID;

  RU5300TimeoutIterate();
  if (!RU5300Serial) { return; }

  status = RU5300RecvRead(&EP,&TID);
  if (status==1) return;//wait

  t32=TID;
  t32xor=TID^(TID>>32);

  //AddLog(LOG_LEVEL_INFO, PSTR("SCAN TAG EP: %08X, TID64: 0x%lX TID: 0x%08X (XOR: 0x%08X)"),EP,TID,t32,t32xor);

  //AddLog(LOG_LEVEL_INFO, PSTR("TAG FOUND %08lX, old tid: %08lx"), (unsigned long)RU5300.LASTTID,(unsigned long)RU5300.TID);
  //ResponseTime_P(PSTR(",\"RU5300\":{\"UID\":\"%08lX\"}}"), (unsigned long) RU5300.LASTTID);


  if (EP==0xe2801160) {
    RU5300SendByTimeout(t32xor);//FOR R6 send XOR lo and hi TID
  } else {
    RU5300SendByTimeout(t32);
  }
}

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
//#ifdef USE_WEBSERVER
//    case FUNC_WEB_SENSOR:
//      RU5300Show();
//      break;
//#endif  // USE_WEBSERVER
  }
  return result;
}

#endif  // USE_RU5300
