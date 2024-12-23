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

static int debug=0;
struct {
  bool           active = false;
  uint8_t        tx = 0;           // GPIO for Serial Tx
  uint8_t        rx = 0;           // GPIO for Serial Rx
  uint8_t        recv[256];        // UART ans
   int8_t        idx;              // UART ans idx, -1 no read answer
   int8_t        timeout;
  TasmotaSerial *serial = NULL;
} UHF_Serial;

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
  char    data[32]; // 30 -- max len TAG
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



static int SendDataToPort(uint8_t *data, int len) {
  int wlen;

  switch (len) {
    case 0: AddLog(LOG_LEVEL_ERROR,"OUT PORT ERROR NO DATA"); break;
    case 1: AddLog(LOG_LEVEL_ERROR,"OUT PORT data %x, len: %i", data[0], len); break;
    case 2: AddLog(LOG_LEVEL_ERROR,"OUT PORT data %x %x, len: %i", data[0], data[1], len); break;
    case 3: AddLog(LOG_LEVEL_ERROR,"OUT PORT data %x %x %x, len: %i", data[0], data[1], data[2], len); break;
    case 4: AddLog(LOG_LEVEL_ERROR,"OUT PORT data %x %x %x %x, len: %i", data[0], data[1], data[2], data[3], len); break;
    case 5: AddLog(LOG_LEVEL_ERROR,"OUT PORT data %x %x %x %x %x, len: %i", data[0], data[1], data[2], data[3], data[4], len); break;
    case 6: AddLog(LOG_LEVEL_ERROR,"OUT PORT data %x %x %x %x %x %x, len: %i", data[0], data[1], data[2], data[3], data[4], data[5], len); break;
    case 7: AddLog(LOG_LEVEL_ERROR,"OUT PORT data %x %x %x %x %x %x %x, len: %i", data[0], data[1], data[2], data[3], data[4], data[5], data[6], len); break;
    case 8: AddLog(LOG_LEVEL_ERROR,"OUT PORT data %x %x %x %x %x %x %x %x, len: %i",
             data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], len); break;
    default:
    AddLog(LOG_LEVEL_ERROR,"OUT PORT data %x %x %x %x %x %x %x %x ... len: %i",
             data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], len); break;
  }

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
static int CmdToReader(uint8_t *data, int size, uint8_t *out, int *len) {
  int     status=0,rlen,rover;
  uint8_t cmd;
  uhrmsgresp_t *resp;

  cmd=data[2];//save old cmd
  if (out==NULL) out=data;
  resp=(uhrmsgresp_t*)out;
  (void)resp;//TODO make the code more readable

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

  return CmdToReader(info->msg.raw, info->msg.len + 1, NULL, NULL);
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
    int hw=1; //HW mode
    int nwmode=1; //interrupt
    int buffer_size=256;
    bool invert=false;

    UHF_Serial.tx = Pin(GPIO_UHF_SER_TX, bus);
    UHF_Serial.rx = Pin(GPIO_UHF_SER_RX, bus);

    AddLog(LOG_LEVEL_ERROR, PSTR(D_LOG_APPLICATION "Init UART, rx: %i, tx: %i hw: %i, nwmode %i, buffsize: %i, invert: %i"), UHF_Serial.rx, UHF_Serial.tx, hw, nwmode, buffer_size, invert);
    UHF_Serial.serial = new TasmotaSerial(UHF_Serial.rx, UHF_Serial.tx, hw, nwmode, buffer_size, invert);
    if (!UHF_Serial.serial->begin(baudrate, SERIAL_8N1)) {
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
  int           addr=0;
  uint8_t       tbuff[64];
  uint8_t      *vmsg=tbuff;

  vmsg=(uint8_t*)AddInt8    (vmsg, 0            );  //len
  vmsg=(uint8_t*)AddInt8    (vmsg, addr         );  //address
  vmsg=(uint8_t*)AddInt8    (vmsg, CMD_UHF_ACOUSTO_OPTIC_CTL);  //cmd
  vmsg=(uint8_t*)AddInt8    (vmsg, active       );  //active
  vmsg=(uint8_t*)AddInt8    (vmsg, silent       );  //silent
  vmsg=(uint8_t*)AddInt8    (vmsg, times        );  //times

  len = WriteLen(tbuff, vmsg);
  WriteCRC(tbuff);

  return CmdToReader(tbuff, len, NULL, NULL);
}

void UHFAnswerParce(void) {
  int rlen;
  int rover;

  if (UHF_Serial.idx==-1) return;

  //memset(recv, 0, 256);
  //if (len==NULL) rover=256;
  //else           rover=*len;
  rover=256-UHF_Serial.idx;

  rlen=UHF_Serial.serial->read(&UHF_Serial.recv[UHF_Serial.idx], rover);
  if (rlen>0) UHF_Serial.idx+=rlen;
  rover-=rlen;
  if (CheckCRC(UHF_Serial.recv, UHF_Serial.idx) ==0) {
    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "SUCCESS Answer recived "));
    UHF_Serial.idx=-1;
    return;
  };
  if (rover<=0) {
    //status=UHRERR_OVFLOW;
    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "Overflow"));
    UHF_Serial.idx=-1;
    return;
  }
  if (UHF_Serial.timeout--<=0) {
    AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "ERROR timeout or crc: data size %i"), UHF_Serial.idx);
    UHF_Serial.idx=-1;
    //if (UHF_Serial.idx==0) return UHRERR_TIMEOUT;
    //else                   return UHRERR_CRC;
    return;
  }
  AddLog(LOG_LEVEL_INFO, PSTR(D_LOG_APPLICATION "Wait more data %i timeout %i"), UHF_Serial.idx, UHF_Serial.timeout);
  //if (rlen>0)
  //if (debug) hexdump("Recv:", recv, idx);

  //if (recv[2] != cmd) return UHRERR_UANS;
//ensure:
//  memcpy(out, recv, idx);
//  if (len!=NULL) len[0]=idx;
//  if (recv[3] != 0 )  return recv[3];

  return ;
}

void UHFSerialSecond(void) {
  int status;

  if (UHF_Serial.idx!=-1) {
    AddLog(LOG_LEVEL_ERROR, PSTR(D_LOG_APPLICATION "waitans, idx: %i, timeout: %i"), UHF_Serial.idx, UHF_Serial.timeout);
    return;
  }

  status = UhrBeep(1, 1, 1);
  AddLog(LOG_LEVEL_ERROR, PSTR(D_LOG_APPLICATION "Hello world %s asdf"), "42");
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

