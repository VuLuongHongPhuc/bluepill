/*
 * mcp2515.h
 *
 *  Created on: May 29, 2024
 *      Author: admin
 */

#ifndef MCP2515_MCP2515_H_
#define MCP2515_MCP2515_H_






#define MCP_8MHz_1000kBPS_CFG1 (0x00)
#define MCP_8MHz_1000kBPS_CFG2 (0x80)
#define MCP_8MHz_1000kBPS_CFG3 (0x80)

#define MCP_8MHz_500kBPS_CFG1 (0x00)
#define MCP_8MHz_500kBPS_CFG2 (0x90)
#define MCP_8MHz_500kBPS_CFG3 (0x82)

#define MCP_8MHz_250kBPS_CFG1 (0x00)
#define MCP_8MHz_250kBPS_CFG2 (0xB1)
#define MCP_8MHz_250kBPS_CFG3 (0x85)

#define MCP_8MHz_200kBPS_CFG1 (0x00)
#define MCP_8MHz_200kBPS_CFG2 (0xB4)
#define MCP_8MHz_200kBPS_CFG3 (0x86)

#define MCP_8MHz_125kBPS_CFG1 (0x01)
#define MCP_8MHz_125kBPS_CFG2 (0xB1)
#define MCP_8MHz_125kBPS_CFG3 (0x85)

#define MCP_8MHz_100kBPS_CFG1 (0x01)
#define MCP_8MHz_100kBPS_CFG2 (0xB4)
#define MCP_8MHz_100kBPS_CFG3 (0x86)

#define MCP_8MHz_80kBPS_CFG1 (0x01)
#define MCP_8MHz_80kBPS_CFG2 (0xBF)
#define MCP_8MHz_80kBPS_CFG3 (0x87)

#define MCP_8MHz_50kBPS_CFG1 (0x03)
#define MCP_8MHz_50kBPS_CFG2 (0xB4)
#define MCP_8MHz_50kBPS_CFG3 (0x86)

#define MCP_8MHz_40kBPS_CFG1 (0x03)
#define MCP_8MHz_40kBPS_CFG2 (0xBF)
#define MCP_8MHz_40kBPS_CFG3 (0x87)

#define MCP_8MHz_33k3BPS_CFG1 (0x47)
#define MCP_8MHz_33k3BPS_CFG2 (0xE2)
#define MCP_8MHz_33k3BPS_CFG3 (0x85)

#define MCP_8MHz_31k25BPS_CFG1 (0x07)
#define MCP_8MHz_31k25BPS_CFG2 (0xA4)
#define MCP_8MHz_31k25BPS_CFG3 (0x84)

#define MCP_8MHz_20kBPS_CFG1 (0x07)
#define MCP_8MHz_20kBPS_CFG2 (0xBF)
#define MCP_8MHz_20kBPS_CFG3 (0x87)

#define MCP_8MHz_10kBPS_CFG1 (0x0F)
#define MCP_8MHz_10kBPS_CFG2 (0xBF)
#define MCP_8MHz_10kBPS_CFG3 (0x87)

#define MCP_8MHz_5kBPS_CFG1 (0x1F)
#define MCP_8MHz_5kBPS_CFG2 (0xBF)
#define MCP_8MHz_5kBPS_CFG3 (0x87)


typedef enum  {
    CAN_5KBPS,
    CAN_10KBPS,
    CAN_20KBPS,
    CAN_31K25BPS,
    CAN_33KBPS,
    CAN_40KBPS,
    CAN_50KBPS,
    CAN_80KBPS,
    CAN_83K3BPS,
    CAN_95KBPS,
    CAN_100KBPS,
    CAN_125KBPS,
    CAN_200KBPS,
    CAN_250KBPS,
    CAN_500KBPS,
    CAN_1000KBPS
}CAN_SPEED;


typedef enum  {
	INSTRUCTION_WRITE       = 0x02,
	INSTRUCTION_READ        = 0x03,
	INSTRUCTION_BITMOD      = 0x05,
	INSTRUCTION_LOAD_TX0    = 0x40,
	INSTRUCTION_LOAD_TX1    = 0x42,
	INSTRUCTION_LOAD_TX2    = 0x44,
	INSTRUCTION_RTS_TX0     = 0x81,
	INSTRUCTION_RTS_TX1     = 0x82,
	INSTRUCTION_RTS_TX2     = 0x84,
	INSTRUCTION_RTS_ALL     = 0x87,
	INSTRUCTION_READ_RX0    = 0x90,
	INSTRUCTION_READ_RX1    = 0x94,
	INSTRUCTION_READ_STATUS = 0xA0,
	INSTRUCTION_RX_STATUS   = 0xB0,
	INSTRUCTION_RESET       = 0xC0
}INSTRUCTION;







typedef enum {
	MCP_RXF0SIDH = 0x00,
	MCP_RXF0SIDL = 0x01,
	MCP_RXF0EID8 = 0x02,
	MCP_RXF0EID0 = 0x03,
	MCP_RXF1SIDH = 0x04,
	MCP_RXF1SIDL = 0x05,
	MCP_RXF1EID8 = 0x06,
	MCP_RXF1EID0 = 0x07,
	MCP_RXF2SIDH = 0x08,
	MCP_RXF2SIDL = 0x09,
	MCP_RXF2EID8 = 0x0A,
	MCP_RXF2EID0 = 0x0B,
	MCP_CANSTAT  = 0x0E,
	MCP_CANCTRL  = 0x0F,
	MCP_RXF3SIDH = 0x10,
	MCP_RXF3SIDL = 0x11,
	MCP_RXF3EID8 = 0x12,
	MCP_RXF3EID0 = 0x13,
	MCP_RXF4SIDH = 0x14,
	MCP_RXF4SIDL = 0x15,
	MCP_RXF4EID8 = 0x16,
	MCP_RXF4EID0 = 0x17,
	MCP_RXF5SIDH = 0x18,
	MCP_RXF5SIDL = 0x19,
	MCP_RXF5EID8 = 0x1A,
	MCP_RXF5EID0 = 0x1B,
	MCP_TEC      = 0x1C,
	MCP_REC      = 0x1D,
	MCP_RXM0SIDH = 0x20,
	MCP_RXM0SIDL = 0x21,
	MCP_RXM0EID8 = 0x22,
	MCP_RXM0EID0 = 0x23,
	MCP_RXM1SIDH = 0x24,
	MCP_RXM1SIDL = 0x25,
	MCP_RXM1EID8 = 0x26,
	MCP_RXM1EID0 = 0x27,
	MCP_CNF3     = 0x28,
	MCP_CNF2     = 0x29,
	MCP_CNF1     = 0x2A,
	MCP_CANINTE  = 0x2B,
	MCP_CANINTF  = 0x2C,
	MCP_EFLG     = 0x2D,
	MCP_TXB0CTRL = 0x30,
	MCP_TXB0SIDH = 0x31,
	MCP_TXB0SIDL = 0x32,
	MCP_TXB0EID8 = 0x33,
	MCP_TXB0EID0 = 0x34,
	MCP_TXB0DLC  = 0x35,
	MCP_TXB0DATA = 0x36,
	MCP_TXB1CTRL = 0x40,
	MCP_TXB1SIDH = 0x41,
	MCP_TXB1SIDL = 0x42,
	MCP_TXB1EID8 = 0x43,
	MCP_TXB1EID0 = 0x44,
	MCP_TXB1DLC  = 0x45,
	MCP_TXB1DATA = 0x46,
	MCP_TXB2CTRL = 0x50,
	MCP_TXB2SIDH = 0x51,
	MCP_TXB2SIDL = 0x52,
	MCP_TXB2EID8 = 0x53,
	MCP_TXB2EID0 = 0x54,
	MCP_TXB2DLC  = 0x55,
	MCP_TXB2DATA = 0x56,
	MCP_RXB0CTRL = 0x60,
	MCP_RXB0SIDH = 0x61,
	MCP_RXB0SIDL = 0x62,
	MCP_RXB0EID8 = 0x63,
	MCP_RXB0EID0 = 0x64,
	MCP_RXB0DLC  = 0x65,
	MCP_RXB0DATA = 0x66,
	MCP_RXB1CTRL = 0x70,
	MCP_RXB1SIDH = 0x71,
	MCP_RXB1SIDL = 0x72,
	MCP_RXB1EID8 = 0x73,
	MCP_RXB1EID0 = 0x74,
	MCP_RXB1DLC  = 0x75,
	MCP_RXB1DATA = 0x76
} REGISTER;

typedef enum  {
	MASK0,
	MASK1
}MASK;

typedef enum  {
	RXF0 = 0,
	RXF1 = 1,
	RXF2 = 2,
	RXF3 = 3,
	RXF4 = 4,
	RXF5 = 5
}RXF;

/*******************  Bit definition for CANINTE register  *******************/
/* CANINTE: CAN INTERRUPT ENABLE REGISTER (ADDRESS: 2Bh) */

typedef union {
  struct {
    uint8_t RX0IE:1;
    uint8_t RX1IE:1;
    uint8_t TX0IE:1;
    uint8_t TX1IE:1;
    uint8_t TX2IE:1;
    uint8_t ERRIE:1;
    uint8_t WAKIE:1;
    uint8_t MERRE:1;
  };
  struct {
    uint8_t b:8;
  };
} __CANINTEbits_t;

typedef __CANINTEbits_t CANINTE;

#define CANINTE_RX0IE_Pos                    (0U)
#define CANINTE_RX0IE_Msk                    (0x1UL << CANINTE_RX0IE_Pos)         /*!< 0x00000001 */
#define CANINTE_RX0IE                        CANINTE_RX0IE_Msk                    /*!< Receive Buffer 0 Full Interrupt Enable bit */
#define CANINTE_RX1IE_Pos                    (1U)
#define CANINTE_RX1IE_Msk                    (0x1UL << CANINTE_RX1IE_Pos)         /*!< 0x00000002 */
#define CANINTE_RX1IE                        CANINTE_RX1IE_Msk                    /*!< Receive Buffer 1 Full Interrupt Enable bit */
#define CANINTE_TX0IE_Pos                    (2U)
#define CANINTE_TX0IE_Msk                    (0x1UL << CANINTE_TX0IE_Pos)         /*!< 0x00000004 */
#define CANINTE_TX0IE                        CANINTE_TX0IE_Msk                    /*!< Transmit Buffer 0 Empty Interrupt Enable bit */
#define CANINTE_TX1IE_Pos                    (3U)
#define CANINTE_TX1IE_Msk                    (0x1UL << CANINTE_TX1IE_Pos)         /*!< 0x00000008 */
#define CANINTE_TX1IE                        CANINTE_TX1IE_Msk                    /*!< Transmit Buffer 1 Empty Interrupt Enable bit */
#define CANINTE_TX2IE_Pos                    (4U)
#define CANINTE_TX2IE_Msk                    (0x1UL << CANINTE_TX2IE_Pos)         /*!< 0x00000010 */
#define CANINTE_TX2IE                        CANINTE_TX2IE_Msk                    /*!< Transmit Buffer 2 Empty Interrupt Enable bit */
#define CANINTE_ERRIE_Pos                    (5U)
#define CANINTE_ERRIE_Msk                    (0x1UL << CANINTE_ERRIE_Pos)         /*!< 0x00000020 */
#define CANINTE_ERRIE                        CANINTE_ERRIE_Msk                    /*!< Error Interrupt Enable bit (multiple sources in EFLG register) */
#define CANINTE_WAKIE_Pos                    (6U)
#define CANINTE_WAKIE_Msk                    (0x1UL << CANINTE_WAKIE_Pos)         /*!< 0x00000040 */
#define CANINTE_WAKIE                        CANINTE_WAKIE_Msk                    /*!< Wake-up Interrupt Enable bit */
#define CANINTE_MERRE_Pos                    (7U)
#define CANINTE_MERRE_Msk                    (0x1UL << CANINTE_MERRE_Pos)         /*!< 0x00000080 */
#define CANINTE_MERRE                        CANINTE_MERRE_Msk                    /*!< Message Error Interrupt Enable bit */



/*******************  Bit definition for CANINTF register  *******************/
/* CANINTF: CAN INTERRUPT FLAG REGISTER (ADDRESS: 2Ch) */

typedef union {
  struct {
    uint8_t RX0IF:1;
    uint8_t RX1IF:1;
    uint8_t TX0IF:1;
    uint8_t TX1IF:1;
    uint8_t TX2IF:1;
    uint8_t ERRIF:1;
    uint8_t WAKIF:1;
    uint8_t MERRF:1;
  };
  struct {
    uint8_t b:8;
  };
} __CANINTFbits_t;

typedef __CANINTFbits_t CANINTF;


#define CANINTF_RX0IF_Pos                    (0U)
#define CANINTF_RX0IF_Msk                    (0x1UL << CANINTF_RX0IF_Pos)         /*!< 0x00000001 */
#define CANINTF_RX0IF                        CANINTF_RX0IF_Msk                    /*!< Receive Buffer 0 Full Interrupt Flag bit */
#define CANINTF_RX1IF_Pos                    (1U)
#define CANINTF_RX1IF_Msk                    (0x1UL << CANINTF_RX1IF_Pos)         /*!< 0x00000002 */
#define CANINTF_RX1IF                        CANINTF_RX1IF_Msk                    /*!< Receive Buffer 1 Full Interrupt Flag bit */
#define CANINTF_TX0IF_Pos                    (2U)
#define CANINTF_TX0IF_Msk                    (0x1UL << CANINTF_TX0IF_Pos)         /*!< 0x00000004 */
#define CANINTF_TX0IF                        CANINTF_TX0IF_Msk                    /*!< Transmit Buffer 0 Empty Interrupt Flag bit */
#define CANINTF_TX1IF_Pos                    (3U)
#define CANINTF_TX1IF_Msk                    (0x1UL << CANINTF_TX1IF_Pos)         /*!< 0x00000008 */
#define CANINTF_TX1IF                        CANINTF_TX1IF_Msk                    /*!< Transmit Buffer 1 Empty Interrupt Flag bit */
#define CANINTF_TX2IF_Pos                    (4U)
#define CANINTF_TX2IF_Msk                    (0x1UL << CANINTF_TX2IF_Pos)         /*!< 0x00000010 */
#define CANINTF_TX2IF                        CANINTF_TX2IF_Msk                    /*!< Transmit Buffer 2 Empty Interrupt Flag bit */
#define CANINTF_ERRIF_Pos                    (5U)
#define CANINTF_ERRIF_Msk                    (0x1UL << CANINTF_ERRIF_Pos)         /*!< 0x00000020 */
#define CANINTF_ERRIF                        CANINTF_ERRIF_Msk                    /*!< Error Interrupt Flag bit (multiple sources in EFLG register) */
#define CANINTF_WAKIF_Pos                    (6U)
#define CANINTF_WAKIF_Msk                    (0x1UL << CANINTF_WAKIF_Pos)         /*!< 0x00000040 */
#define CANINTF_WAKIF                        CANINTF_WAKIF_Msk                    /*!< Wake-up Interrupt Flag bit */
#define CANINTF_MERRF_Pos                    (7U)
#define CANINTF_MERRF_Msk                    (0x1UL << CANINTF_MERRF_Pos)         /*!< 0x00000080 */
#define CANINTF_MERRF                        CANINTF_MERRF_Msk                    /*!< Message Error Interrupt Flag bit */


/*
extern volatile uint32_t   IPC1 __attribute__((section("sfrs"), address(0xBF810150)));
typedef union {
  struct {
    uint32_t T1IS:2;
    uint32_t T1IP:3;
    uint32_t :3;
    uint32_t IC1EIS:2;
    uint32_t IC1EIP:3;
    uint32_t :3;
    uint32_t IC1IS:2;
    uint32_t IC1IP:3;
    uint32_t :3;
    uint32_t OC1IS:2;
    uint32_t OC1IP:3;
  };
  struct {
    uint32_t w:32;
  };
} __IPC1bits_t;
extern volatile __IPC1bits_t IPC1bits __asm__ ("IPC1") __attribute__((section("sfrs"), address(0xBF810150)));
extern volatile uint32_t        IPC1CLR __attribute__((section("sfrs"),address(0xBF810154)));
extern volatile uint32_t        IPC1SET __attribute__((section("sfrs"),address(0xBF810158)));
extern volatile uint32_t        IPC1INV __attribute__((section("sfrs"),address(0xBF81015C)));
*/


/*******************  Bit definition for RXB0CTRL register  *******************/
/* RXB0CTRL: RECEIVE BUFFER 0 CONTROL REGISTER (ADDRESS: 60h) */
typedef union {
  struct {
    uint8_t        :1;
    uint8_t RXM    :2;
    uint8_t        :1;
    uint8_t RXRTR  :1;
    uint8_t BUKT   :1;
    uint8_t        :1;
    uint8_t FILHIT0:1;
  };
  struct {
    uint8_t b:8;
  };
} __RXB0CTRLbits_t;

typedef __RXB0CTRLbits_t RXB0CTRL;

#define RXBnCTRL_RXM_Pos      (5U)
#define RXBnCTRL_RXM_Msk      (3UL << RXBnCTRL_RXM_Pos)    /*!< 0x00000060 */
#define RXBnCTRL_RXM_ANY      RXBnCTRL_RXM_Msk             /*!< 11 = Turns mask/filters off; receives any message */
#define RXBnCTRL_RXM_FILTER   0x00                         /*!< 00 = Receives all valid messages using either Standard or Extended Identifiers that meet filter criteria */

#define RXBnCTRL_RTR_Pos      (3U)
#define RXBnCTRL_RTR_Msk      (1UL <<  RXBnCTRL_RTR_Pos)   /*!< 0x00000008 */
#define RXBnCTRL_RTR          RXBnCTRL_RTR_Msk             /*!< Received Remote Transfer Request bit */

#define RXB0CTRL_BUKT_Pos     (2U)
#define RXB0CTRL_BUKT_Msk     (1UL << RXB0CTRL_BUKT_Pos)   /*!< 0x00000004 */
#define RXB0CTRL_BUKT         RXB0CTRL_BUKT_Msk            /*!< Rollover Enable bit */

#define RXB0CTRL_FILHIT0_Pos   (0U)
#define RXB0CTRL_FILHIT0_Msk   (1UL << RXB0CTRL_FILHIT0_Pos) /*!< 0x00000001 */
#define RXB0CTRL_FILHIT0       RXB0CTRL_FILHIT0_Msk          /*!< Filter Hit bit (indicates which acceptance filter enabled reception of message) */



/*******************  Bit definition for RXB1CTRL register  *******************/
/* RXB1CTRL: RECEIVE BUFFER 1 CONTROL REGISTER (ADDRESS: 70h) */

typedef union {
  struct {
    uint8_t        :1;
    uint8_t RXM    :2;
    uint8_t        :1;
    uint8_t RXRTR  :1;
    uint8_t FILHIT0:3;
  };
  struct {
    uint8_t b:8;
  };
} __RXB1CTRLbits_t;

typedef __RXB1CTRLbits_t RXB1CTRL;

#define RXB1CTRL_FILHIT_Pos   (0U)
#define RXB1CTRL_FILHIT_MASK  (7UL << RXB1CTRL_FILHIT_Pos) /*!< 0x00000007 */
#define RXB1CTRL_FILHIT0      0x01                         /*!< Filter Hit bits (indicates which acceptance filter enabled reception of message) */
#define RXB1CTRL_FILHIT1      0x02
#define RXB1CTRL_FILHIT2      0x04



/*******************  Error value definition *********************************/

typedef enum {
	ERROR_OK        = 0,
	ERROR_FAIL      = 1,
	ERROR_ALLTXBUSY = 2,
	ERROR_FAILINIT  = 3,
	ERROR_FAILTX    = 4,
	ERROR_NOMSG     = 5
}MCP_ERROR;






/*** Prototypes *********************************************************/

void MCP2515_CAN_initialize(void);
//MCP_ERROR MCP2515_reset(void);

MCP_ERROR MCP2515_SetFilterMask(const MASK mask, const bool ext, const uint32_t ulData);
MCP_ERROR MCP2515_SetFilter(const RXF num, const bool ext, const uint32_t ulData);

MCP_ERROR MCP2515_SetBitrate(const CAN_SPEED canSpeed);

MCP_ERROR MCP2515_SetConfigMode();
MCP_ERROR MCP2515_SetListenOnlyMode();
MCP_ERROR MCP2515_SetSleepMode();
MCP_ERROR MCP2515_SetLoopbackMode();
MCP_ERROR MCP2515_SetNormalMode();

MCP_ERROR MCP2515_ReadMessage(struct can_frame *frame);
MCP_ERROR MCP2515_WriteMessage(const struct can_frame *frame);


/*** Initialize SPI input function *******************************************/

/* NOTES: To provide these function */
typedef void (*functDelay)(long unsigned int);
typedef void (*functWritePin)(uint8_t);
typedef void (*functWrite)(uint8_t);
typedef uint8_t (*functRead)(void);
//typedef void (*functReadWrite)(uint8_t* txBuf, uint8_t* rxBuf, uint16_t len);


typedef struct{
	functDelay    Delay;
	functWritePin CS;
	functRead     Read;
	functWrite    Write;
}StructSpi;


bool MCP2515_SPI_initialize(StructSpi* pSPI);




#endif /* MCP2515_MCP2515_H_ */
