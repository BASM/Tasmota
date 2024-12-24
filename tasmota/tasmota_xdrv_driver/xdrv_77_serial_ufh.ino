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

#define MODE_INIT        0
#define MODE_INVENTORY   1
#define MODE_READTID     2

#define MODE_RECV_NONE     0
#define MODE_RECV_WAIT     1
#define MODE_RECV_SUCCESS  2
#define MODE_RECV_ERROR    3

static int debug=0;

enum COMMANDS {
  ///////////////////////////////////////////////////////////////////
  ////////////// EPC C1 G2（ISO18000-6C）COMMAND ////////////////////
  ///////////////////////////////////////////////////////////////////
  // The function is used to inventory tags in the effective field and get their EPC values.
  CMD_INVENTORY         =0x01,
  // The function is used to read part or all of a Tag’s Password, EPC, TID, or User memory.
  // To the word as a unit, start to read data from the designated address.
  CMD_UHF_READ_DATA         =0x02,
  // The function is used to write several words in a Tag’s Reserved, EPC, TID, or User memory.
  CMD_WRITE_DATA        =0x03,
  // The function is used to write EPC value in a Tag’s EPC memory. Random write one
  // tag in the effective field.
  CMD_WRITE_EPC         =0x04,
  // The function is used to kill tag. After the tag killed, it never process command.
  CMD_KILL_TAG          =0x05,
  // The function is used to set Password area as readable and writeable from any state,
  // readable and writeable from the secured state, permanently readable and writeable,
  // never readable and writeable. It used to set EPC, TID or User as writeable from any
  // state, writeable from the secured state, permanently writeable, never writeable.
  CMD_LOCK              =0x06,
  // The function is used to erase multiple words in a Tag’s Password, EPC, TID, or User memory.
  CMD_BLOCK_ERASE       =0x07,
  // The function is used to set designated tag read protection. After the tag protected,
  // it never process command. Even if inventory tag, reader can not get the EPC number. 
  // The read protection can be removed by executing Reset ReadProtect. Only NXP's UCODE EPC G2X tags valid. 
  CMD_READ_PROTECT      =0x08,
  // The function is used to random set one tag read protection in the effective field. The tag must be having the same access password. Only NXP's UCODE EPC G2X tags valid. 
  CMD_READ_PROTECT_NOEPC=0x09,
  // The function is used to remove only one tag read protection in the effective field. The tag must be having the same access password. Only NXP's UCODE EPC G2X tags valid. 
  CMD_RESET_READ_PROTECT=0x0A,
  // The function is used to check only one tag in the effective field, whether the tag is protected. It can not check the tag whether the tag support protection setting. Only NXP's UCODE EPC G2X tags valid.
  CMD_CHECK_READ_PROTECT=0x0B,
  //The function is used to set or reset the EAS status bit of designated tag. Only NXP's UCODE EPC G2X tags valid.
  CMD_EAS_ALARM         =0x0C,
  // The function is used to check EAS status bit of any tag in the effective field. Only NXP's UCODE EPC G2X tags valid.
  CMD_CHECK_EAS_ALARM   =0x0D,
  // The function is used to permanently lock the designated data in designated tag’s user memory. The locked data can be read only, but not written and not erased. Only NXP's UCODE EPC G2X tags valid.
  CMD_BLOCK_LOCK        =0x0E,
  // The function is used to inventory one tag in the effective field and get their EPC values.
  CMD_INVENTORY_SINGLE  =0x0F,
  // The function is used to write multiple words in a Tag’s Reserved, EPC, TID, or User memory.
  CMD_BLOCK_WRITE       =0x10,

  ///////////////////////////////////////////////////
  /////////////   18000-6B COMMAND //////////////////
  ///////////////////////////////////////////////////
  // The function is used to Inventory only one tag in the effective field and get their ID values. If more than one tag in the effective field at the same time, reader may be get nothing. 
  CMD_INV_SIGNAL_6B     =0x50,
  // The function is used to according to the given conditions Inventory tags in the effective field and get their ID values.
  CMD_INV_MULTI_6B      =0x51,
  // The function is used to start to read several bytes from the designated address. 
  CMD_READ_DATA_6B      =0x52,
  // The function is used to start to write several bytes from the designated address. 
  CMD_WRITE_DATA_6B     =0x53,
  // The function is used to check whether the designated byte is locked. 
  CMD_CHECK_LOCK_6B     =0x54,
  // The function is used to lock the designated byte.
  CMD_LOCK_6B           =0x55,

  ///////////////////////////////////////////////////////////////////
  ////////////// READER DEFINED COMMAND /////////////////////////////
  ///////////////////////////////////////////////////////////////////
  // This function is used to get reader-related information such as
  // reader address (Adr), firmware version, supported protocol type, Inventory ScanTime, power and frequency.
  CMD_GET_READER_INFO   =0x21,
  // Sets the current region. The function is used to set the reader working of the lower limit and the upper limit of frequency.
  CMD_SET_REGION        =0x22,
  // This function is used to set a new address of the reader. The address value will store in reader’s inner nonvolatile memory. Default address value is 0x00. The value range is 0x00~0xFE. The address 0xFF is reserved as the broadcasting address. When user tries to write a 0xFF to Adr, the reader will set the value to 0x00 automatically. 
  CMD_SET_ADDRESS       =0x24,
  // This function is used to set a new value to Inventory ScanTime of an appointed reader. The range is 3~255 corresponding to 3*100ms~255*100ms Inventory ScanTime. The default value of Inventory ScanTime is 10*100ms. 
  CMD_SET_SCANTIME      =0x25,
  // The function is used to change the serial port baud rate.
  CMD_BAUD_RATE         =0x28,
  // The function is used to set the power of reader.
  CMD_SET_POWER         =0x2F,
  // Acousto-optic Control
  CMD_UHF_ACOUSTO_OPTIC_CTL =0x33,
  // BIZZER (undocumented)
  //  05 00 35 80 64 21 -- 05 00 35 00 6c a5    -- set close
  //  05 00 35 81 ed 30 -- 05 00 35 00 6c a5    -- set open
  //
  //  05 00 35 00 6c a5 -- 06 00 35 00 00 03 b4 -- get close
  //  05 00 35 00 6c a5 -- 05 00 35 00 6c a5    -- get open
  CMD_BIZZER_CTL        =0x35, // where manual?
};

#pragma pack(push,1)
typedef struct {
  union {
    struct {
      uint8_t  len;
      uint8_t  addr;
      uint8_t  cmd;
      //uint8_t  crc;//send crc
    };
    uint8_t raw[3];
  };
} uhrmsg_t;

typedef struct _uhrresp_ {
  uhrmsg_t msg;        // 0x00 -- 0x02
  uint8_t  status;
  uint16_t crc;
} uhrmsgresp_t;

typedef struct _uhrreader_ {
  int fd;
  int addr;
  int opened;
} uhrdev_t;

typedef struct _ltag_ {
  uint8_t wlen;     // len in words 0..15
  uint8_t data[32]; // 30 -- max len TAG
} uhrtag_t;

typedef struct _tid_ {
  union {
    struct {
      uint8_t  mclass;
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
    };
    uint32_t raw;
  };
} uhrtaginfo_t;

typedef struct _uhrinfo_ {
  uhrmsg_t msg;        // 0x00 -- 0x03
  uint16_t version;    // 0x04
  uint8_t  model;      // 0x06
  uint8_t  supro;      // 0x07    support protocol
  uint8_t  dmaxfre;    // 0x08
  uint8_t  dminfre;    // 0x09
  uint8_t  power;      // 0x10
  uint8_t  scantime;   // 0x11
  uint8_t  crc;        // 0x12
  uint8_t  padding[3]; // 0x13-0x16
} uhrmsginfo_t;
#pragma pack(pop)

enum {
  UHRLT_KLL =0x00,
  UHRLT_ACC =0x01,
  UHRLT_EPC =0x02,
  UHRLT_TID =0x03,
  UHRLT_USR =0x04,
};

enum {
  UHRLM_W_ANY  =0x00,
  UHRLM_W_P    =0x01,
  UHRLM_W_SEC  =0x02,
  UHRLM_W_NO   =0x03,
};

enum {
  UHRMEM_PASS=0,
  UHRMEM_EPC =1,
  UHRMEM_TID =2,
  UHRMEM_USER=3
};

enum URHERRORS {
  // 0x00-0x19 ; 0xf9-0xff -- TAG or READER errors
  UHRERR_GOOD     =0x00, // GOOD
  UHRERR_NOTAG    =0xFB,

  //////////////////////// System errors
  UHRERR_NOTAGS   =0x20,
  UHRERR_MANYTAGS =0x21, // many tags
  UHRERR_GOODFIX  =0x22, // Tag is fixed, all good
  UHRERR_CANTLOCK =0x23, // can't lock password
  UHRERR_WRITE    =0x30, // Error write to reader
  UHRERR_CRC      =0x31,
  UHRERR_TIMEOUT  =0x32,
  UHRERR_OVFLOW   =0x38,
  UHRERR_MREAD    =0x55,// misread 
  UHRERR_UANS     =0xEE,
  ///// TAG ERRR 0x70 + tag error code
  UHRERR_TERR_OTHER    =0x70, // 0 -- other error
  UHRERR_TERR_OVERRUN  =0x73, // 3 -- overrrun memory
  UHRERR_TERR_LOCKED   =0x74, // 4 -- memory locked
  UHRERR_TERR_POWER    =0x7b, // b -- insufficient power
  UHRERR_TERR_ANY      =0x7f, // tag unsupported TERR code

  ///
  UHRERR_POORCOMM =0xFA,// Are some tags in the effective field, but Poor Communication between reader and tag.
  UHRERR_INVALID  =0xFF,
};

struct {
  bool           active = false;
  uint8_t        tx = 0;           // GPIO for Serial Tx
  uint8_t        rx = 0;           // GPIO for Serial Rx
  uint8_t        wg=0;             // wiegand init
  uint8_t        wgd0=0;           // wiegand D0
  uint8_t        wgd1=0;           // wiegand D1
  uint8_t        mode=MODE_INIT;
  uint8_t        waitrecv=MODE_RECV_NONE; // Mode SEND/RECV/SUCCES/ERROR
  uint8_t        recv[256];        // UART ans
   int8_t        idx=-1;           // UART ans idx, -1 no read answer, onty dec timeout
   int8_t        timeout=10;
  uint8_t        addr=0;
  uhrtag_t       tag;              // work one tag only
  TasmotaSerial *serial = NULL;
} UHF_Serial;



static void dumprecv(const char *name, uint8_t *data, int len) {
  int  i;
  int  idx=0;
  char buff[512];

  strcpy(buff, name);
  idx=strlen(buff);
  for (i=0; i<len; i++) {
    sprintf(&buff[idx]," %2.2X",data[i]);
    idx=strlen(buff);
  }

  AddLog(LOG_LEVEL_INFO,buff);
  return ;
}

static int SendDataToPort(uint8_t *data, int len) {
  int wlen;

  dumprecv("SEND", data, len);

  //hexdump("Send data", data, len);

  //TODO purgecomm
  wlen=UHF_Serial.serial->write(data,len);
  if (wlen!=len) return UHRERR_WRITE;
  return 0;
}

static int usleep(int ums) {
  delayMicroseconds(ums);
  return 0;
}

// Send command and read answer from device
//
static int CmdToReader(uint8_t *data, int size) {
  int     status=0,rlen,rover;
  uint8_t cmd;

  cmd=data[2];//save old cmd

  UHF_Serial.serial->flush();
  status = SendDataToPort  (data, size);
  if (status!=0) return status;
  UHF_Serial.idx=0;
  UHF_Serial.timeout=10;

  return status;
}


#define  POLYNOMIAL   0x8408 
#define  PRESET_VALUE 0xffff
void UhrWriteCRC(uint8_t *pData, int len) {
  int i,j;
  unsigned int current_crc_value = PRESET_VALUE;
  for (i=0;i<len;i++) {
    current_crc_value=current_crc_value^((unsigned int)pData[i]);
    for (j=0; j<8; j++) {
      if (current_crc_value&0x0001)
        current_crc_value=(current_crc_value>>1)^POLYNOMIAL;
      else  current_crc_value=(current_crc_value>>1);
    }
  }
  pData[i++] = (unsigned char)(current_crc_value&0x00ff);
  pData[i] = (unsigned char)((current_crc_value>>8)&0x00ff);
}

static int CheckCRC(uint8_t *pData, int len) {
  UhrWriteCRC(pData,len);
  if ((pData[len + 1] == 0) && (pData[len] == 0))
    return 0;
  else  return 0x31;
}

static void WriteCRC(void *mbegin) {
  uhrmsg_t *msg=(uhrmsg_t*)mbegin;
  UhrWriteCRC(msg->raw, msg->len-1);
}

static void *AddInt8(void *vptr, uint8_t data) {
  uint8_t *p=(uint8_t*)vptr;
  p[0]=data;
  return &p[1];
}

static uint32_t htobe32(uint32_t data) {
 return ntohl(data);
}

#define ntohll(x) ((1==ntohl(1)) ? (x) : ((uint64_t)ntohl((x) & 0xFFFFFFFF) << 32) | ntohl((x) >> 32))
uint64_t be64toh(uint64_t data) {
  return ntohll(data);
}

static void *AddInt32(void *vptr, uint32_t data) {
  uint32_t *p=(uint32_t *)vptr;
  uint32_t val=htobe32(data);
  memcpy(p,&val,4); //not aligment 
  return &p[1];
}

// Add data to msg request
// * vptr -- buffer,
// * wlen -- add count (by words)
// * data -- data
// \return -- dataptr
static void *AddData(void *vptr, uint8_t len, uint8_t *data) {
  uint8_t *p=(uint8_t*)vptr;
  p[0]=len;
  len*=2; //Size on word, not bytes
  if (len>0) memcpy(&p[1], data, len);
  return &p[1+len];
}

static int WriteLen(void *mbegin, void *mend) {
  uhrmsg_t *msg=(uhrmsg_t*)mbegin;
  msg->len = (uintptr_t)mend - (uintptr_t)mbegin+1;
  return msg->len+1;//+1 CRC -1 for first byte
}

static int UhrPrintInfo(void *vinfo) { //тип указывать нельзя, оно как-то автогерентит какую-то фигню и не собирается
  uhrmsginfo_t *info=(uhrmsginfo_t *)vinfo;
  AddLog(LOG_LEVEL_ERROR,"COM addr     : %x, cmd: %x", info->msg.addr, info->msg.cmd);
  AddLog(LOG_LEVEL_ERROR,"Version      : %x", info->version);
  AddLog(LOG_LEVEL_ERROR,"Support Proto: %x", info->supro);
  AddLog(LOG_LEVEL_ERROR,"Model        : %x", info->model);
  AddLog(LOG_LEVEL_ERROR,"dMaxFre      : %x", info->dmaxfre);
  AddLog(LOG_LEVEL_ERROR,"dMinFre      : %x", info->dminfre);
  AddLog(LOG_LEVEL_ERROR,"power        : %x", info->power);
  AddLog(LOG_LEVEL_ERROR,"scantime     : %x", info->scantime);
  AddLog(LOG_LEVEL_ERROR,"CRC          : %x", info->crc);
  return 0;
}

static int UhrGetReaderInformation(void *vinfo) {
  uhrmsginfo_t *info=(uhrmsginfo_t *)vinfo;
  info->msg.len  = sizeof(uhrmsg_t)+1;
  info->msg.addr = 0; //WTF addr?? FIXME
  info->msg.cmd  = CMD_GET_READER_INFO;
  UhrWriteCRC(info->msg.raw, info->msg.len-1);

  return CmdToReader(info->msg.raw, info->msg.len + 1);
}

static int TestGetReaderInfo(void) {
  int          status;
  uhrmsginfo_t devinfo;

  status = UhrGetReaderInformation(&devinfo);
  if (status != 0) {
    AddLog(LOG_LEVEL_ERROR,"Error get info");
    return -1;
  }
  UhrPrintInfo(&devinfo);
  return 0;
}


void UHFSerialInit(void) {
  uint32_t bus=0;
  int      res=0;

  AddLog(LOG_LEVEL_ERROR, PSTR(D_LOG_APPLICATION "TRY INIT 4"));
  if (PinUsed(GPIO_UHF_SER_TX, bus) && PinUsed(GPIO_UHF_SER_RX, bus)) {
    int baudrate = 57600;
    int hw=0; //HW mode
    int nwmode=0; //interrupt
    int buffer_size=256;
    bool invert=false;

    UHF_Serial.tx = Pin(GPIO_UHF_SER_TX, bus);
    UHF_Serial.rx = Pin(GPIO_UHF_SER_RX, bus);

    if (PinUsed(GPIO_UHF_WGD0, bus) && PinUsed(GPIO_UHF_WGD1, bus)) {
      UHF_Serial.wgd0 = Pin(GPIO_UHF_WGD0, bus);
      UHF_Serial.wgd1 = Pin(GPIO_UHF_WGD1, bus);

      pinMode(UHF_Serial.wgd0,OUTPUT);
      pinMode(UHF_Serial.wgd1,OUTPUT);
      UHF_Serial.wg   = 1;
    }

    AddLog(LOG_LEVEL_ERROR, PSTR(D_LOG_APPLICATION "Init UART, rx: %i, tx: %i hw: %i, nwmode %i, buffsize: %i, invert: %i"), UHF_Serial.rx, UHF_Serial.tx, hw, nwmode, buffer_size, invert);
    UHF_Serial.serial = new TasmotaSerial(UHF_Serial.rx, UHF_Serial.tx, hw, nwmode, buffer_size, invert);
    if (!UHF_Serial.serial->begin(baudrate)) {
      AddLog(LOG_LEVEL_ERROR, PSTR(D_LOG_APPLICATION "Can't init UART"));
      return;
    }
    //res = TestGetReaderInfo();
    res=0;
    if (res==0) {
      UHF_Serial.active = true;
      AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "UHF activate success"));
    }
  } else {
    AddLog(LOG_LEVEL_ERROR, PSTR(D_LOG_APPLICATION "ERROR NO PINS"));
  }
}

int UhrBeep(int active, int silent, int times) {
  int           len;
  uint8_t       tbuff[64];
  uint8_t      *vmsg=tbuff;

  vmsg=(uint8_t*)AddInt8    (vmsg, 0               );  //len
  vmsg=(uint8_t*)AddInt8    (vmsg, UHF_Serial.addr );  //address
  vmsg=(uint8_t*)AddInt8    (vmsg, CMD_UHF_ACOUSTO_OPTIC_CTL);  //cmd
  vmsg=(uint8_t*)AddInt8    (vmsg, active       );  //active
  vmsg=(uint8_t*)AddInt8    (vmsg, silent       );  //silent
  vmsg=(uint8_t*)AddInt8    (vmsg, times        );  //times

  len = WriteLen(tbuff, vmsg);
  WriteCRC(tbuff);

  return CmdToReader(tbuff, len);
}


void UHFAnswerParce(void) {
  int rlen;
  int rover;

  if (UHF_Serial.timeout==0) return;
  if (UHF_Serial.idx==-1) { UHF_Serial.timeout--; return; }

  //memset(recv, 0, 256);
  //if (len==NULL) rover=256;
  //else           rover=*len;
  rover=256-UHF_Serial.idx;

  rlen=UHF_Serial.serial->read(&UHF_Serial.recv[UHF_Serial.idx], rover);
  if (rlen>0) UHF_Serial.idx+=rlen;
  rover-=rlen;
  if (CheckCRC(UHF_Serial.recv, UHF_Serial.idx) ==0) {
    //AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "SUCCESS Answer recived %i size "), UHF_Serial.idx);
    dumprecv("RECV", UHF_Serial.recv, UHF_Serial.idx);
    UHF_Serial.waitrecv=MODE_RECV_SUCCESS;
    UHF_Serial.timeout=0;
    return;
  };
  if (rover<=0) {
    //status=UHRERR_OVFLOW;
    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "Overflow"));
    UHF_Serial.waitrecv=MODE_RECV_ERROR;
    UHF_Serial.timeout=0;
    return;
  }
  if (--UHF_Serial.timeout<=0) {
    uint8_t *data=(uint8_t*)UHF_Serial.recv;
    int      len =UHF_Serial.idx;

    dumprecv("RECV PORT ERR", data,len);

    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "ERROR timeout or crc: data size %i"), UHF_Serial.idx);
    UHF_Serial.waitrecv=MODE_RECV_ERROR;
    UHF_Serial.timeout=0;
    //if (UHF_Serial.idx==0) return UHRERR_TIMEOUT;
    //else                   return UHRERR_CRC;
    return;
  }
  //AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "Wait more data %i timeout %i"), UHF_Serial.idx, UHF_Serial.timeout);
  //if (rlen>0)
  //if (debug) hexdump("Recv:", recv, idx);

  //if (recv[2] != cmd) return UHRERR_UANS;
//ensure:
//  memcpy(out, recv, idx);
//  if (len!=NULL) len[0]=idx;
//  if (recv[3] != 0 )  return recv[3];

  return ;
}

typedef struct _uhrinv_ {
  uhrmsg_t msg;        // 0x00 -- 0x02
  uint8_t status;
  uint8_t count;
  uint8_t mtag[254];// [ [len + EPC], [len+EPC]... ]
  uint16_t crc;
} uhrmsginv_t;

static int UhrInventory(void) {
  int status,len;
  uint8_t       tbuff[16];
  uint8_t      *vmsg=tbuff;

  vmsg=(uint8_t*)AddInt8    (vmsg, sizeof(uhrmsg_t)+1 );  //len
  vmsg=(uint8_t*)AddInt8    (vmsg, UHF_Serial.addr    );  //address
  vmsg=(uint8_t*)AddInt8    (vmsg, CMD_INVENTORY      );  //cmd

  len = WriteLen(tbuff, vmsg);
  WriteCRC(tbuff);

  return CmdToReader(tbuff, len);
}

static int UhrParseInventory(void *vtags, uint16_t *count) { //FIXME XXX secure violation: no check UHF_Serial.idx
  int i;
  uhrtag_t *t;
  uhrmsginv_t  *msg=(uhrmsginv_t  *)UHF_Serial.recv;
  uhrtag_t *tags=(uhrtag_t *)vtags;

  if (msg->count > 200) { *count=0; return 0; } //sorry, FIXME too
  t=(uhrtag_t *)msg->mtag;
  for (i=0; i < msg->count; i++) {
    if (i>=*count) break;
    tags[i].wlen=t->wlen/2;
    memcpy(tags[i].data,t->data,t->wlen);

    t=(uhrtag_t *)(((uint8_t*)t)+sizeof(t->wlen)+t->wlen);//FIXME checkit , one label check only
  }
  *count=msg->count;
  return 0;
}

int UhrReadCardTID(void *vtag, uint32_t pass) {
  uhrtag_t    *tag=(uhrtag_t *)vtag;
  int          status,rsize,len;
  uint8_t      tbuff[128];
  void        *vmsg=tbuff;

  vmsg=AddInt8    (vmsg, 0                );  //len
  vmsg=AddInt8    (vmsg, UHF_Serial.addr  );  //address
  vmsg=AddInt8    (vmsg, CMD_UHF_READ_DATA);  //cmd
  if (tag==NULL)
       vmsg=AddData  (vmsg, 0, NULL);
  else vmsg=AddData  (vmsg, tag->wlen, tag->data);
  vmsg=AddInt8    (vmsg, UHRMEM_TID);     //mem type
  vmsg=AddInt8    (vmsg, 0);              //start
  vmsg=AddInt8    (vmsg, 6);              //world count 
  vmsg=AddInt32   (vmsg, pass);           //password

  len = WriteLen(tbuff, vmsg);
  WriteCRC(tbuff);

  return CmdToReader(tbuff, len);
}

static int UhrParceReadCardTID(void *vtaginfo, uint64_t *tid) {
  uint64_t val;
  uhrtaginfo_t *taginfo=(uhrtaginfo_t *)vtaginfo;

  if (UHF_Serial.idx!=(12+4+2)) {
    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "Wrong answer size, must %i, real: %i"),(12+4+2),UHF_Serial.idx);
    return 1;
  }

  memcpy(taginfo, &UHF_Serial.recv[4], 4);
  //memcpy(tid, &tbuff[6], 8);

  memcpy(&val,&UHF_Serial.recv[8],8);
  //*taginfo=be16toh(*((uint16_t*)&tbuff[4]));
  
  tid[0] =be64toh(val);
  AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "PARCE TAG : %llx"), val);
  AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "PARCE TAGI: %lx"), taginfo->raw);

  //memcpy(data,tbuff+4,16);
  //hexdump("TBUFF", data, 26);

  return 0;
}

static void TagTID2tag(void *vtag, void *vinfo, uint64_t *TID) {
  uhrtag_t     *tag  =(uhrtag_t *)    vtag;
  uhrtaginfo_t *info =(uhrtaginfo_t *)vinfo;
  uint64_t htid=be64toh(*TID);
  tag->wlen=6;
  memcpy(&tag->data[0], info  , sizeof(info[0]));
  memcpy(&tag->data[4], &htid , 8);
}

static uint32_t xor32(uint64_t a) {
  return a^(a>>32);
}

#define WGPULSETIME 2
#define WGPAUSETIME 20
static int sendone(void) {
  digitalWrite(UHF_Serial.wgd1,HIGH);
  usleep(WGPULSETIME);
  digitalWrite(UHF_Serial.wgd1,LOW);
  return 0;
}

static int sendzero(void) {
  digitalWrite(UHF_Serial.wgd0,HIGH);
  usleep(WGPULSETIME);
  digitalWrite(UHF_Serial.wgd0,LOW);
  return 0;
}

static int senduint32_t(uint32_t cmdle) {
  int i;
  int tmp;
  int odd=1,even=0;
  uint32_t cmd=cmdle;

  for (tmp=0,i=0; i<16; i++)
    if ((cmd>>i)&1) tmp++;
  if ((tmp%2)==0) odd=0;

  for (tmp=0,i=16; i<32; i++)
    if ((cmd>>i)&1) tmp++;
  if ((tmp%2)==0) even=1;

  if (even)  sendzero();
  else       sendone();
  usleep(WGPAUSETIME);

  if (debug) printf(" ");
  for (i=0; i<32; i++) {
    int b = (cmd>>(31-i))&1;
    if (b) sendone();
    else   sendzero();
    usleep(WGPAUSETIME);
    if ((i%8)==7) if (debug) printf(" ");
  }

  if (odd) sendzero();
  else     sendone();
  usleep(WGPAUSETIME);
  return 0;
}

static int UhrWgSend(uint32_t wgcmd) {
  if (UHF_Serial.wg==0) AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "Can't send WG, no config"));
  else                  senduint32_t(wgcmd);
  return 0;
}


void UHFSerialSecond(void) {
  int status;

  if (UHF_Serial.timeout!=0) {
    AddLog(LOG_LEVEL_ERROR, PSTR(D_LOG_APPLICATION "waitans, idx: %i, timeout: %i"), UHF_Serial.idx, UHF_Serial.timeout);
    return;
  }

  switch (UHF_Serial.mode) {
    case MODE_INIT:
      switch (UHF_Serial.waitrecv) {
        case MODE_RECV_NONE:
          AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "BEEP init"));
          UhrBeep(1, 1, 3);
          UHF_Serial.waitrecv=MODE_RECV_WAIT;
          break;
        case MODE_RECV_WAIT:
          AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "Wait beep..."));
          break;
        case MODE_RECV_SUCCESS:
          AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "Recived, switch to CYCLE mode..."));
          UHF_Serial.mode=MODE_INVENTORY;
          UHF_Serial.waitrecv=MODE_RECV_NONE;
          break;
        case MODE_RECV_ERROR:
          AddLog(LOG_LEVEL_ERROR, PSTR(D_LOG_APPLICATION "Wrong UHF reader, beeping..."));
          UhrBeep(1, 1, 1);
          UHF_Serial.waitrecv=MODE_RECV_WAIT;
          break;
      }
      break;
    case MODE_INVENTORY:
      switch(UHF_Serial.waitrecv) {
        case MODE_RECV_NONE:
          AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "Inventory..."));
          UhrInventory();
          UHF_Serial.waitrecv=MODE_RECV_WAIT;
          break;
        case MODE_RECV_WAIT:
          AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "Wait recv Inventory..."));
          break;
        case MODE_RECV_SUCCESS: {
          uint16_t count;
          uhrtag_t tags[8]; count=8;
          UhrParseInventory(tags, &count);
          AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "Inventory success... tags count: %i"), count);
          if (count==0) {
            UHF_Serial.waitrecv=MODE_RECV_NONE;
            break;
          }
          UHF_Serial.waitrecv=MODE_RECV_NONE;
          UHF_Serial.mode=MODE_READTID;
          UHF_Serial.tag=tags[0];
          //for (i=0; i<count; i++) {
          //  UhrPrintTag(&tags[i]);
          //}
          break; }
        case MODE_RECV_ERROR:
          AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "Inventory error..."));
          UHF_Serial.waitrecv=MODE_RECV_NONE;
          break;
      }
      break;
    case MODE_READTID:
      switch(UHF_Serial.waitrecv) {
        case MODE_RECV_NONE: {
          uhrtag_t *tag = &UHF_Serial.tag;
          dumprecv("Read tid", tag->data, tag->wlen);
          UhrReadCardTID(&UHF_Serial.tag, 0x00000000);// try password zero //, &taginfo, &TID);
          //UhrReadCardTID(NULL, 0x00000000);// try password zero //, &taginfo, &TID);
          UHF_Serial.waitrecv=MODE_RECV_WAIT;
          break; }
        case MODE_RECV_WAIT:
          AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "Wait read tid..."));
          break;
        case MODE_RECV_SUCCESS: {
          uint64_t TID;
          uint32_t t32;
          int      status;
          uhrtag_t      tidtag;
          uhrtaginfo_t  taginfo;
          AddLog(LOG_LEVEL_DEBUG, PSTR(D_LOG_APPLICATION "TID READ SUCCESS..."));

          status=UhrParceReadCardTID(&taginfo, &TID);
          if (status!=0) {
            UHF_Serial.waitrecv=MODE_RECV_NONE;
            UHF_Serial.mode    =MODE_INVENTORY;}
          TagTID2tag(&tidtag, &taginfo, &TID);
          if ((taginfo.raw&0xfffff)==0x180e2) { //Monza R6 do not supported passowrds
            uint32_t t32=xor32(TID);
            UhrWgSend(t32);//32 bites
            AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "WG SEND 1 %llx..."),t32);
            UhrBeep(1, 1, 1);
          } else {
            uint32_t t32=TID;
            UhrWgSend(t32);//32 bites
            AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "WG SEND 2 %llx..."),t32);
            UhrBeep(1, 1, 2);
          }
          UHF_Serial.waitrecv=MODE_RECV_NONE;
          UHF_Serial.mode    =MODE_INVENTORY;
          break; }
        case MODE_RECV_ERROR:
          AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "Error read TID..."));
          UHF_Serial.waitrecv=MODE_RECV_NONE;
          UHF_Serial.mode    =MODE_INVENTORY;
          break;
      }
      break;
  }
  return ;
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
      case FUNC_EVERY_250_MSECOND:
        UHFAnswerParce();
        break;
      case FUNC_INIT:
        UHFSerialInit();
        break;
    }
  }
  return result;
}

#endif  // USE_UHF_SERIAL

