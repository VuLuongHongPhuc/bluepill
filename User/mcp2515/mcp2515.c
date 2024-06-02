/*
 * mcp2515.c
 *
 *  Created on: May 29, 2024
 *      Author: admin
 */


#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <stdbool.h>
#include "can.h"
#include "mcp2515.h"


/*** private define *****************************/
#define TXB_ABTF    0x40
#define	TXB_MLOA    0x20
#define TXB_TXERR   0x10
#define	TXB_TXREQ   0x08
#define	TXB_TXIE    0x04
#define	TXB_TXP     0x03


#define TXB_EXIDE_MASK  0x08
#define DLC_MASK        0x0F
#define RTR_MASK        0x40

#define MCP_SIDH  0
#define MCP_SIDL  1
#define MCP_EID8  2
#define MCP_EID0  3
#define MCP_DLC   4
#define MCP_DATA  5




/*! \brief Index transmit buffer block */
typedef enum  {
	TXB0 = 0,
	TXB1,
	TXB2,
	TXBnMax
}TXBn;

typedef struct  {
	REGISTER CTRL;         /*!< register address TXBnCTRL */
	REGISTER SIDH;         /*!< register address TXBnSIDH */
	REGISTER DATA;         /*!< register address TXBnDATA */
} TXBn_REGS_TypeDef;

static const TXBn_REGS_TypeDef TXB[TXBnMax] = {
    {MCP_TXB0CTRL, MCP_TXB0SIDH, MCP_TXB0DATA},
    {MCP_TXB1CTRL, MCP_TXB1SIDH, MCP_TXB1DATA},
    {MCP_TXB2CTRL, MCP_TXB2SIDH, MCP_TXB2DATA}
};






/*! \brief Index receive buffer block */
typedef enum {
	RXB0 = 0,
	RXB1,
	RXBnMax,
}RXBn;

typedef struct {
	REGISTER CTRL;          /*!< register address RXBnCTRL */
	REGISTER SIDH;          /*!< register address RXBnSIDH */
	REGISTER DATA;          /*!< register address RXBnDATA */
	uint8_t  CANINTF_RXnIF; /*!< Mask CANINTF_RXnIF_Msk */
}RXBn_REGS_TypeDef;

static const RXBn_REGS_TypeDef RXB[RXBnMax] = {
    {MCP_RXB0CTRL, MCP_RXB0SIDH, MCP_RXB0DATA, CANINTF_RX0IF_Msk},
    {MCP_RXB1CTRL, MCP_RXB1SIDH, MCP_RXB1DATA, CANINTF_RX1IF_Msk}
};

#define STAT_RX0IF  0x01
#define STAT_RX1IF  0x02




#define CANSTAT_OPMOD  0xE0
#define CANSTAT_ICOD   0x0E

#define CANCTRL_REQOP   0xE0
#define CANCTRL_ABAT    0x10
#define CANCTRL_OSM     0x08
#define CANCTRL_CLKEN   0x04
#define CANCTRL_CLKPRE  0x03

typedef enum {
	CANCTRL_REQOP_NORMAL     = 0x00,
	CANCTRL_REQOP_SLEEP      = 0x20,
	CANCTRL_REQOP_LOOPBACK   = 0x40,
	CANCTRL_REQOP_LISTENONLY = 0x60,
	CANCTRL_REQOP_CONFIG     = 0x80,
	CANCTRL_REQOP_POWERUP    = 0xE0
}CANCTRL_REQOP_MODE;


/*** Variables *************************************************************/


/*** Prototypes ************************************************************/
uint8_t MCP2515_ReadRegister(const REGISTER reg);
void MCP2515_ReadRegisters(const REGISTER reg, uint8_t* values, const uint8_t n);
void MCP2515_SetRegister(const REGISTER reg, const uint8_t value);
void MCP2515_SetRegisters(const REGISTER reg, const uint8_t* values, const uint8_t n);
void MCP2515_ModifyRegister(const REGISTER reg, const uint8_t mask, const uint8_t data);
uint8_t MCP2515_GetStatus(void);

MCP_ERROR MCP2515_SetMode(const CANCTRL_REQOP_MODE mode);

void MCP2515_PrepareId(uint8_t *buffer, const bool ext, const uint32_t id);


/*** Code *****************/


static StructSpi* hSPI = NULL;

bool MCP2515_SPI_initialize(StructSpi* pSPI)
{
	if (pSPI == NULL) { return false; }

	hSPI = pSPI;

	hSPI->CS(1);

	return true;
}


void MCP2515_CAN_initialize(void)
{
	hSPI->CS(0);
	hSPI->Write(INSTRUCTION_RESET);
	hSPI->CS(1);

	hSPI->Delay(10);

	uint8_t zeros[14];
	memset(zeros, 0, sizeof(zeros));
	MCP2515_SetRegisters(MCP_TXB0CTRL, zeros, 14);
	MCP2515_SetRegisters(MCP_TXB1CTRL, zeros, 14);
	MCP2515_SetRegisters(MCP_TXB2CTRL, zeros, 14);

	MCP2515_SetRegister(MCP_RXB0CTRL, 0);
	MCP2515_SetRegister(MCP_RXB1CTRL, 0);

	// interrupt on RX0 & RX1
	MCP2515_SetRegister(MCP_CANINTE, CANINTE_RX0IE | CANINTE_RX1IE);

	// receives any messages using either Standard or Extended Identifiers
	// no filter/mask
	MCP2515_ModifyRegister(MCP_RXB0CTRL,
				   RXBnCTRL_RXM_Msk | RXB0CTRL_BUKT_Msk | RXB0CTRL_FILHIT0_Msk,
				   RXBnCTRL_RXM_ANY | RXB0CTRL_BUKT );

	MCP2515_ModifyRegister(MCP_RXB1CTRL,
					   RXBnCTRL_RXM_Msk | RXB1CTRL_FILHIT_MASK,
					   RXBnCTRL_RXM_ANY);


}



MCP_ERROR MCP2515_reset(void)
{
	hSPI->CS(0);
    hSPI->Write(INSTRUCTION_RESET);
    hSPI->CS(1);

    hSPI->Delay(10);

    uint8_t zeros[14];
    memset(zeros, 0, sizeof(zeros));
    MCP2515_SetRegisters(MCP_TXB0CTRL, zeros, 14);
    MCP2515_SetRegisters(MCP_TXB1CTRL, zeros, 14);
    MCP2515_SetRegisters(MCP_TXB2CTRL, zeros, 14);

    MCP2515_SetRegister(MCP_RXB0CTRL, 0);
    MCP2515_SetRegister(MCP_RXB1CTRL, 0);

    MCP2515_SetRegister(MCP_CANINTE, CANINTE_RX0IE | CANINTE_RX1IE | CANINTE_ERRIE | CANINTE_MERRE);

    // receives all valid messages using either Standard or Extended Identifiers that
    // meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1
    MCP2515_ModifyRegister(MCP_RXB0CTRL,
                   RXBnCTRL_RXM_Msk | RXB0CTRL_BUKT_Msk | RXB0CTRL_FILHIT0_Msk,
				   RXBnCTRL_RXM_FILTER | RXB0CTRL_BUKT);
    MCP2515_ModifyRegister(MCP_RXB1CTRL,
                   RXBnCTRL_RXM_Msk | RXB1CTRL_FILHIT_MASK,
				   RXBnCTRL_RXM_FILTER | RXB1CTRL_FILHIT0);

    // clear filters and masks
    // do not filter any standard frames for RXF0 used by RXB0
    // do not filter any extended frames for RXF1 used by RXB1
    RXF filters[] = {RXF0, RXF1, RXF2, RXF3, RXF4, RXF5};
    for (int i=0; i<6; i++) {
        bool ext = (i == 1);
        MCP_ERROR result = MCP2515_SetFilter(filters[i], ext, 0);
        if (result != ERROR_OK) {
            return result;
        }
    }

    MASK masks[] = {MASK0, MASK1};
    for (int i=0; i<2; i++) {
        MCP_ERROR result = MCP2515_SetFilterMask(masks[i], true, 0);
        if (result != ERROR_OK) {
            return result;
        }
    }

    return ERROR_OK;
}



uint8_t MCP2515_ReadRegister(const REGISTER reg)
{
	hSPI->CS(0);

	hSPI->Write(INSTRUCTION_READ);
	hSPI->Write(reg);
	uint8_t ret = hSPI->Read();

    hSPI->CS(1);

    return ret;
}

void MCP2515_ReadRegisters(const REGISTER reg, uint8_t* values, const uint8_t n)
{

	hSPI->CS(0);

	hSPI->Write(INSTRUCTION_READ);
	hSPI->Write(reg);
	// mcp2515 has auto-increment of address-pointer
	for (uint8_t i=0; i<n; i++) {
		values[i] = hSPI->Read();
	}

	hSPI->CS(1);
}

void MCP2515_SetRegister(const REGISTER reg, const uint8_t value)
{
	hSPI->CS(0);

	hSPI->Write(INSTRUCTION_WRITE);
	hSPI->Write(reg);
	hSPI->Write(value);

    hSPI->CS(1);
}

void MCP2515_SetRegisters(const REGISTER reg, const uint8_t* values, const uint8_t n)
{
	hSPI->CS(0);

	hSPI->Write(INSTRUCTION_WRITE);
	hSPI->Write(reg);
    for (uint8_t i=0; i<n; i++) {
    	hSPI->Write(values[i]);
    }

    hSPI->CS(1);
}

void MCP2515_ModifyRegister(const REGISTER reg, const uint8_t mask, const uint8_t data)
{
	hSPI->CS(0);

	hSPI->Write(INSTRUCTION_BITMOD);
	hSPI->Write(reg);
	hSPI->Write(mask);
	hSPI->Write(data);

    hSPI->CS(1);
}

uint8_t MCP2515_GetStatus(void)
{
	hSPI->CS(0);

	hSPI->Write(INSTRUCTION_READ_STATUS);
    uint8_t i = hSPI->Read();

    hSPI->CS(1);

    return i;
}

MCP_ERROR MCP2515_SetFilterMask(const MASK mask, const bool ext, const uint32_t ulData)
{
    MCP_ERROR res = MCP2515_SetConfigMode();
    if (res != ERROR_OK) {
        return res;
    }

    uint8_t tbufdata[4];
    MCP2515_PrepareId(tbufdata, ext, ulData);

    REGISTER reg;
    switch (mask) {
        case MASK0: reg = MCP_RXM0SIDH; break;
        case MASK1: reg = MCP_RXM1SIDH; break;
        default:
            return ERROR_FAIL;
    }

    MCP2515_SetRegisters(reg, tbufdata, 4);

    return ERROR_OK;
}

MCP_ERROR MCP2515_SetFilter(const RXF num, const bool ext, const uint32_t ulData)
{
    MCP_ERROR res = MCP2515_SetConfigMode();
    if (res != ERROR_OK) {
        return res;
    }

    REGISTER reg;

    switch (num) {
        case RXF0: reg = MCP_RXF0SIDH; break;
        case RXF1: reg = MCP_RXF1SIDH; break;
        case RXF2: reg = MCP_RXF2SIDH; break;
        case RXF3: reg = MCP_RXF3SIDH; break;
        case RXF4: reg = MCP_RXF4SIDH; break;
        case RXF5: reg = MCP_RXF5SIDH; break;
        default:
            return ERROR_FAIL;
    }

    uint8_t tbufdata[4];
    MCP2515_PrepareId(tbufdata, ext, ulData);
    MCP2515_SetRegisters(reg, tbufdata, 4);

    return ERROR_OK;
}

MCP_ERROR MCP2515_SetConfigMode()
{
    return MCP2515_SetMode(CANCTRL_REQOP_CONFIG);
}

MCP_ERROR MCP2515_SetListenOnlyMode()
{
    return MCP2515_SetMode(CANCTRL_REQOP_LISTENONLY);
}

MCP_ERROR MCP2515_SetSleepMode()
{
    return MCP2515_SetMode(CANCTRL_REQOP_SLEEP);
}

MCP_ERROR MCP2515_SetLoopbackMode()
{
    return MCP2515_SetMode(CANCTRL_REQOP_LOOPBACK);
}

MCP_ERROR MCP2515_SetNormalMode()
{
    return MCP2515_SetMode(CANCTRL_REQOP_NORMAL);
}

MCP_ERROR MCP2515_SetMode(const CANCTRL_REQOP_MODE mode)
{
	MCP2515_ModifyRegister(MCP_CANCTRL, CANCTRL_REQOP, mode);

    //unsigned long endTime = millis() + 10;

	uint8_t time_count = 0;

    bool modeMatch = false;
    while (time_count++ < 10)
    {
        uint8_t newmode = MCP2515_ReadRegister(MCP_CANSTAT);
        newmode &= CANSTAT_OPMOD;

        modeMatch = (newmode == mode);

        if (modeMatch) {
            break;
        }

        hSPI->Delay(1);
    }

    return modeMatch ? ERROR_OK : ERROR_FAIL;
}

MCP_ERROR MCP2515_SetBitrate(const CAN_SPEED canSpeed)
{
	MCP_ERROR error = MCP2515_SetConfigMode();
	if (error != ERROR_OK)
	{
		return error;
	}

	uint8_t set, cfg1, cfg2, cfg3;
	set = 1;
	switch (canSpeed)
	{
		case (CAN_5KBPS):                                               //   5KBPS
		cfg1 = MCP_8MHz_5kBPS_CFG1;
		cfg2 = MCP_8MHz_5kBPS_CFG2;
		cfg3 = MCP_8MHz_5kBPS_CFG3;
		break;

		case (CAN_10KBPS):                                              //  10KBPS
		cfg1 = MCP_8MHz_10kBPS_CFG1;
		cfg2 = MCP_8MHz_10kBPS_CFG2;
		cfg3 = MCP_8MHz_10kBPS_CFG3;
		break;

		case (CAN_20KBPS):                                              //  20KBPS
		cfg1 = MCP_8MHz_20kBPS_CFG1;
		cfg2 = MCP_8MHz_20kBPS_CFG2;
		cfg3 = MCP_8MHz_20kBPS_CFG3;
		break;

		case (CAN_31K25BPS):                                            //  31.25KBPS
		cfg1 = MCP_8MHz_31k25BPS_CFG1;
		cfg2 = MCP_8MHz_31k25BPS_CFG2;
		cfg3 = MCP_8MHz_31k25BPS_CFG3;
		break;

		case (CAN_33KBPS):                                              //  33.333KBPS
		cfg1 = MCP_8MHz_33k3BPS_CFG1;
		cfg2 = MCP_8MHz_33k3BPS_CFG2;
		cfg3 = MCP_8MHz_33k3BPS_CFG3;
		break;

		case (CAN_40KBPS):                                              //  40Kbps
		cfg1 = MCP_8MHz_40kBPS_CFG1;
		cfg2 = MCP_8MHz_40kBPS_CFG2;
		cfg3 = MCP_8MHz_40kBPS_CFG3;
		break;

		case (CAN_50KBPS):                                              //  50Kbps
		cfg1 = MCP_8MHz_50kBPS_CFG1;
		cfg2 = MCP_8MHz_50kBPS_CFG2;
		cfg3 = MCP_8MHz_50kBPS_CFG3;
		break;

		case (CAN_80KBPS):                                              //  80Kbps
		cfg1 = MCP_8MHz_80kBPS_CFG1;
		cfg2 = MCP_8MHz_80kBPS_CFG2;
		cfg3 = MCP_8MHz_80kBPS_CFG3;
		break;

		case (CAN_100KBPS):                                             // 100Kbps
		cfg1 = MCP_8MHz_100kBPS_CFG1;
		cfg2 = MCP_8MHz_100kBPS_CFG2;
		cfg3 = MCP_8MHz_100kBPS_CFG3;
		break;

		case (CAN_125KBPS):                                             // 125Kbps
		cfg1 = MCP_8MHz_125kBPS_CFG1;
		cfg2 = MCP_8MHz_125kBPS_CFG2;
		cfg3 = MCP_8MHz_125kBPS_CFG3;
		break;

		case (CAN_200KBPS):                                             // 200Kbps
		cfg1 = MCP_8MHz_200kBPS_CFG1;
		cfg2 = MCP_8MHz_200kBPS_CFG2;
		cfg3 = MCP_8MHz_200kBPS_CFG3;
		break;

		case (CAN_250KBPS):                                             // 250Kbps
		cfg1 = MCP_8MHz_250kBPS_CFG1;
		cfg2 = MCP_8MHz_250kBPS_CFG2;
		cfg3 = MCP_8MHz_250kBPS_CFG3;
		break;

		case (CAN_500KBPS):                                             // 500Kbps
		cfg1 = MCP_8MHz_500kBPS_CFG1;
		cfg2 = MCP_8MHz_500kBPS_CFG2;
		cfg3 = MCP_8MHz_500kBPS_CFG3;
		break;

		case (CAN_1000KBPS):                                            //   1Mbps
		cfg1 = MCP_8MHz_1000kBPS_CFG1;
		cfg2 = MCP_8MHz_1000kBPS_CFG2;
		cfg3 = MCP_8MHz_1000kBPS_CFG3;
		break;

		default:
		set = 0;
		break;
	}

	if (set)
	{
		MCP2515_SetRegister(MCP_CNF1, cfg1);
		MCP2515_SetRegister(MCP_CNF2, cfg2);
		MCP2515_SetRegister(MCP_CNF3, cfg3);
		return ERROR_OK;
	}
	else
	{
		return ERROR_FAIL;
	}
}






MCP_ERROR MCP2515_ReadReceiveBuffer(const RXBn rxbn, struct can_frame *frame)
{
    const RXBn_REGS_TypeDef* rxb = &RXB[rxbn];

    uint8_t tbufdata[5];

    MCP2515_ReadRegisters(rxb->SIDH, tbufdata, 5);

    uint32_t id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if ( (tbufdata[MCP_SIDL] & TXB_EXIDE_MASK) ==  TXB_EXIDE_MASK ) {
        id = (id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        id = (id<<8) + tbufdata[MCP_EID8];
        id = (id<<8) + tbufdata[MCP_EID0];
        id |= CAN_EFF_FLAG;
    }

    uint8_t dlc = (tbufdata[MCP_DLC] & DLC_MASK);
    if (dlc > CAN_MAX_DLEN) {
        return ERROR_FAIL;
    }

    uint8_t ctrl = MCP2515_ReadRegister(rxb->CTRL);
    if (ctrl & RXBnCTRL_RTR) {
        id |= CAN_RTR_FLAG;
    }

    frame->can_id = id;
    frame->can_dlc = dlc;

    /* Retrieve data */
    MCP2515_ReadRegisters(rxb->DATA, frame->data, dlc);

    /* Clear flag RXnIF */
    MCP2515_ModifyRegister(MCP_CANINTF, rxb->CANINTF_RXnIF, 0);

    return ERROR_OK;
}

MCP_ERROR MCP2515_ReadMessage(struct can_frame *frame)
{
    MCP_ERROR rc;
    uint8_t stat = MCP2515_GetStatus();

    if ( stat & STAT_RX0IF ) {
        rc = MCP2515_ReadReceiveBuffer(RXB0, frame);
    } else if ( stat & STAT_RX1IF ) {
        rc = MCP2515_ReadReceiveBuffer(RXB1, frame);
    } else {
        rc = ERROR_NOMSG;
    }

    return rc;
}









void MCP2515_PrepareId(uint8_t *buffer, const bool ext, const uint32_t id)
{
    uint16_t canid = (uint16_t)(id & 0x0FFFF);

    if (ext)
    {
        buffer[MCP_EID0] = (uint8_t) (canid & 0xFF);
        buffer[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        buffer[MCP_SIDL] = (uint8_t) (canid & 0x03);
        buffer[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        buffer[MCP_SIDL] |= TXB_EXIDE_MASK;
        buffer[MCP_SIDH] = (uint8_t) (canid >> 5);
    }
    else
    {
        buffer[MCP_SIDH] = (uint8_t) (canid >> 3);
        buffer[MCP_SIDL] = (uint8_t) ((canid & 0x07 ) << 5);
        buffer[MCP_EID0] = 0;
        buffer[MCP_EID8] = 0;
    }
}

MCP_ERROR MCP2515_TransmitMessage(const TXBn txbn, const struct can_frame *frame)
{
	/* check dlc length */
    if (frame->can_dlc > CAN_MAX_DLEN) { return ERROR_FAILTX; }


    const TXBn_REGS_TypeDef *txbuf = &TXB[txbn];

    uint8_t data[13];

    bool ext = (frame->can_id & CAN_EFF_FLAG);
    bool rtr = (frame->can_id & CAN_RTR_FLAG);
    uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

    MCP2515_PrepareId(data, ext, id);

    data[MCP_DLC] = rtr ? (frame->can_dlc | RTR_MASK) : frame->can_dlc;

    memcpy(&data[MCP_DATA], frame->data, frame->can_dlc);

    MCP2515_SetRegisters(txbuf->SIDH, data, 5 + frame->can_dlc);

    MCP2515_ModifyRegister(txbuf->CTRL, TXB_TXREQ, TXB_TXREQ);

    uint8_t ctrl = MCP2515_ReadRegister(txbuf->CTRL);
    if ((ctrl & (TXB_ABTF | TXB_MLOA | TXB_TXERR)) != 0)
    {
        return ERROR_FAILTX;
    }
    return ERROR_OK;
}

MCP_ERROR MCP2515_WriteMessage(const struct can_frame *frame)
{
    if (frame->can_dlc > CAN_MAX_DLEN) {
        return ERROR_FAILTX;
    }

    TXBn txBuffers[TXBnMax] = {TXB0, TXB1, TXB2};

    for (int i=0; i<TXBnMax; i++)
    {
        const TXBn_REGS_TypeDef *txbuf = &TXB[txBuffers[i]];
        uint8_t ctrlval = MCP2515_ReadRegister(txbuf->CTRL);
        if ( (ctrlval & TXB_TXREQ) == 0 )
        {
            return MCP2515_TransmitMessage(txBuffers[i], frame);
        }
    }

    return ERROR_ALLTXBUSY;
}


/*EOF*/
