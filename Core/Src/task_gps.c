/*
 * task_gps.c
 *
 *  Created on: Feb 18, 2026
 *      Author: brzan
 */

#include "task_gps.h"
#include "main.h"
#include "cmsis_os2.h"
#include "fore_board.h"

#include "usart.h"

static void ubx_checksum(const uint8_t *data, uint16_t len,
                         uint8_t *ck_a, uint8_t *ck_b)
{
    for (uint16_t i = 0; i < len; i++) {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

static void ubx_send(uint8_t cls, uint8_t id,
                     const uint8_t *payload, uint16_t length)
{
    uint8_t tx_buffer[6 + 64 + 2];
    uint16_t idx = 0;

    tx_buffer[idx++] = 0xB5;
    tx_buffer[idx++] = 0x62;

    tx_buffer[idx++] = cls;
    tx_buffer[idx++] = id;

    tx_buffer[idx++] = length & 0xFF;
    tx_buffer[idx++] = (length >> 8) & 0xFF;

    for (uint16_t i = 0; i < length; i++)
    {
    	tx_buffer[idx++] = payload[i];
    }

    uint8_t ck_a = 0, ck_b = 0;
    ubx_checksum(&tx_buffer[2], 4+length, &ck_a, &ck_b);

    tx_buffer[idx++] = ck_a;
    tx_buffer[idx++] = ck_b;

    HAL_UART_Transmit(&huart2, tx_buffer, 6+length+2, 100);
}


typedef enum {
    UBX_SYNC1,
    UBX_SYNC2,
    UBX_CLASS,
    UBX_ID,
    UBX_LEN1,
    UBX_LEN2,
    UBX_PAYLOAD,
    UBX_CK_A,
    UBX_CK_B
} ubx_state_t;
#define UBX_MAX_PAYLOAD_LENGTH 500
typedef struct {
    ubx_state_t state;

    uint8_t cls;
    uint8_t id;

    uint16_t payload_len;
    uint16_t payload_cnt;

    uint8_t payload[UBX_MAX_PAYLOAD_LENGTH];

    uint8_t ck_a;
    uint8_t ck_b;

    uint8_t rx_ck_a;
    uint8_t rx_ck_b;

} ubx_parser_t;
static ubx_parser_t parser;

static inline uint32_t rd_u32(uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}
static inline int32_t rd_i32(uint8_t *p)
{
    return (int32_t)rd_u32(p);
}
void UBX_NAV_PVT_handler(void)
{
	fore_board_t* pfb = fore_board_get_ptr();
	uint8_t *p = parser.payload;

    pfb->gnss_info.year  = (uint16_t)p[4] | ((uint16_t)p[5] << 8);
    pfb->gnss_info.month = p[6];
    pfb->gnss_info.day   = p[7];
    pfb->gnss_info.hour  = p[8];
    pfb->gnss_info.min   = p[9];
    pfb->gnss_info.sec   = p[10];

    pfb->gnss_info.valid   = p[11];
    pfb->gnss_info.tAcc_ns = rd_u32(&p[12]);
    pfb->gnss_info.nano_ns = rd_i32(&p[16]);

    pfb->gnss_info.fixType = p[20];
    pfb->gnss_info.flags   = p[21];
    pfb->gnss_info.flags2  = p[22];
    pfb->gnss_info.numSV   = p[23];

    pfb->gnss_info.lon = rd_i32(&p[24]);
    pfb->gnss_info.lat = rd_i32(&p[28]);

    pfb->gnss_info.height_mm = rd_i32(&p[32]);
    pfb->gnss_info.hMSL_mm   = rd_i32(&p[36]);

    pfb->gnss_info.hAcc_mm = rd_u32(&p[40]);
    pfb->gnss_info.vAcc_mm = rd_u32(&p[44]);

    pfb->gnss_info.gSpeed_mms     = rd_i32(&p[60]);
    pfb->gnss_info.headMot_deg1e5 = rd_i32(&p[64]);
    pfb->gnss_info.sAcc_mms       = rd_u32(&p[68]);
    pfb->gnss_info.headAcc_deg1e5 = rd_u32(&p[72]);

    pfb->gnss_info.pDOP_centi = (uint16_t)p[76] | ((uint16_t)p[77] << 8);
    pfb->gnss_info.flags3     = (uint16_t)p[78] | ((uint16_t)p[79] << 8);
}

static void UBX_ACK_ACK_handler(void);
static void UBX_ACK_NAK_handler(void);

volatile int UBX_frame_recieved = 0;
volatile int UBX_frame_not_handled = 0;
volatile int UBX_ACK_ACK = 0;
volatile int UBX_ACK_NAK = 0;
volatile int UBX_CFG_VALGET = 0;
volatile int UBX_NAV_PVT = 0;
void ubx_frame_recieved(void)
{
	UBX_frame_recieved++;

	switch (parser.cls)
	{
	case 0x05: // UBX-ACK
		switch (parser.id)
		{
		case 0x01: // UBX-ACK-ACK
			UBX_ACK_ACK++;
			UBX_ACK_ACK_handler();
			break;
		case 0x00: // UBX-ACK-NAK
			UBX_ACK_NAK++;
			UBX_ACK_NAK_handler();
			break;
		default:
			UBX_frame_not_handled++;
			break;
		}
		break;

	case 0x06: // UBX-CFG
		switch (parser.id)
		{
		case 0x8b: // UBX-CFG-VALGET
			UBX_CFG_VALGET++;
			break;
		default:
			UBX_frame_not_handled++;
			break;
		}
		break;

	case 0x01:	// UBX-NAV
		switch (parser.id)
		{
		case 0x07: // UBX-NAV-PVT
			UBX_NAV_PVT++;
			UBX_NAV_PVT_handler();
			break;
		default:
			UBX_frame_not_handled++;
			break;
		}
		break;

	default:
		UBX_frame_not_handled++;
		break;
	}
}
void ubx_parser_reset(void)
{
    parser.state = UBX_SYNC1;
    parser.cls = 0;
	parser.id = 0;
	parser.payload_len = 0;
	parser.payload_cnt = 0;
    parser.ck_a = 0;
    parser.ck_b = 0;
}
void ubx_process_byte(uint8_t byte)
{
	switch (parser.state)
	{
		case UBX_SYNC1:
			if (byte == 0xB5)
				parser.state = UBX_SYNC2;
			break;

		case UBX_SYNC2:
	        if (byte == 0x62)
	        	parser.state = UBX_CLASS;
	        else
	        	parser.state = UBX_SYNC1;
	        break;

		case UBX_CLASS:
			parser.cls = byte;
			parser.ck_a = 0;
			parser.ck_b = 0;
			parser.state = UBX_ID;
		    parser.ck_a += byte;
		    parser.ck_b += parser.ck_a;
			break;

		case UBX_ID:
			parser.id = byte;
			parser.state = UBX_LEN1;
		    parser.ck_a += byte;
		    parser.ck_b += parser.ck_a;
			break;

		case UBX_LEN1:
			parser.payload_len = byte;
			parser.state = UBX_LEN2;
		    parser.ck_a += byte;
		    parser.ck_b += parser.ck_a;
			break;

		case UBX_LEN2:
			parser.payload_len |= (byte << 8);

			if (parser.payload_len > UBX_MAX_PAYLOAD_LENGTH)
				ubx_parser_reset();
			else if (parser.payload_len > 0)
				parser.state = UBX_PAYLOAD;
			else if (parser.payload_len == 0)
				parser.state = UBX_CK_A;
		    parser.ck_a += byte;
		    parser.ck_b += parser.ck_a;
			break;

		case UBX_PAYLOAD:
			parser.payload[parser.payload_cnt++] = byte;

			if (parser.payload_cnt >= parser.payload_len)
				parser.state = UBX_CK_A;
			parser.ck_a += byte;
			parser.ck_b += parser.ck_a;
			break;

		case UBX_CK_A:
			parser.rx_ck_a = byte;
			parser.state = UBX_CK_B;
			break;

		case UBX_CK_B:
			parser.rx_ck_b = byte;

			if (parser.rx_ck_a == parser.ck_a && parser.rx_ck_b == parser.ck_b)
				ubx_frame_recieved();

			ubx_parser_reset();
			break;
	}
}

void ubx_process_span(uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        ubx_process_byte(data[i]);
    }
}

typedef enum {
    GPS_INIT_DISABLE_NMEA,
	GPS_INIT_DISABLE_NMEA_WAIT_ACK,

	GPS_INIT_SETUP_UBX_PVT,
	GPS_INIT_SETUP_UBX_PVT_WAIT_ACK,

    GPS_INIT_SETUP_DYN_MODEL,
    GPS_INIT_SETUP_DYN_MODEL_WAIT_ACK,

    GPS_INIT_SEND_MONVER,
    GPS_INIT_WAIT_MONVER,

    GPS_INIT_DONE
} gps_init_state_t;
gps_init_state_t gps_init_state = GPS_INIT_DISABLE_NMEA;


/** GPS ACK CHECKING FUNCTIONALITIES */
typedef enum {
    GPS_ACK_IDLE = 0,
    GPS_ACK_PENDING,
    GPS_ACK_OK,
    GPS_ACK_NAK,
    GPS_ACK_TIMEOUT
} gps_ack_status_t;
typedef struct {
    uint8_t expected_cls;
    uint8_t expected_id;
    gps_ack_status_t status;
    uint32_t started_at_ms;
    uint32_t timeout_ms;
} gps_ack_tracker_t;
static gps_ack_tracker_t gps_ack = {
    .expected_cls = 0,
    .expected_id = 0,
    .status = GPS_ACK_IDLE,
    .started_at_ms = 0,
    .timeout_ms = 200
};
static void gps_ack_set_pending(uint8_t cls, uint8_t id)
{
    gps_ack.expected_cls = cls;
    gps_ack.expected_id = id;
    gps_ack.status = GPS_ACK_PENDING;
    gps_ack.started_at_ms = osKernelGetTickCount();
}
static gps_ack_status_t gps_ack_check(uint8_t cls, uint8_t id)
{
    if ((gps_ack.expected_cls != cls) || (gps_ack.expected_id != id)) {
        return GPS_ACK_IDLE;
    }

    if (gps_ack.status == GPS_ACK_PENDING) {
        uint32_t now = osKernelGetTickCount();
        if ((now - gps_ack.started_at_ms) >= gps_ack.timeout_ms) {
            gps_ack.status = GPS_ACK_TIMEOUT;
        }
    }

    return gps_ack.status;
}
static void UBX_ACK_ACK_handler(void)
{
    if (parser.payload_len >= 2) 
	{
		if ((gps_ack.status == GPS_ACK_PENDING) &&
			(gps_ack.expected_cls == parser.payload[0]) &&
			(gps_ack.expected_id == parser.payload[1])) 
		{
			gps_ack.status = GPS_ACK_OK;
		}
    }
}
static void UBX_ACK_NAK_handler(void)
{
    if (parser.payload_len >= 2) 
	{
		if ((gps_ack.status == GPS_ACK_PENDING) &&
			(gps_ack.expected_cls == parser.payload[0]) &&
			(gps_ack.expected_id == parser.payload[1])) 
		{
			gps_ack.status = GPS_ACK_NAK;
    	}
    }
}

volatile static int gps_init_disable_nmea_done = 0;
volatile static int gps_init_setup_ubx_pvt_done = 0;
volatile static int gps_init_setup_dyn_model_done = 0;
// Dopakowac te zmienne do Valid flagi bo czm nie

#define GPS_RX_BUFFER_SIZE 512
static uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE];

extern volatile uint32_t task_gps_alive;
void task_gps(void *argument)
{
	// Start the DMA USART2 RX
	HAL_UART_Receive_DMA(&huart2, gps_rx_buffer, GPS_RX_BUFFER_SIZE);

	while (1)
	{
		// RX handling
		{
			static uint16_t old_pos = 0;
			uint16_t pos = GPS_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

	//	    if (pos == old_pos)
	//	    	// nothing new

			if (pos > old_pos)
			{
				// linear region
				ubx_process_span(&gps_rx_buffer[old_pos],
								 pos - old_pos);
			}
			else if  (pos < old_pos)
			{
				// wrapped region
				ubx_process_span(&gps_rx_buffer[old_pos],
								 GPS_RX_BUFFER_SIZE - old_pos);

				if (pos > 0)
					ubx_process_span(&gps_rx_buffer[0], pos);
			}

			old_pos = pos;
		}

		// INIT sequence
		{
			switch (gps_init_state)
			{
			case GPS_INIT_DISABLE_NMEA:
			{
				// Send CFG-UART1OUTPROT-NMEA False on id 0x10740002 type L aka U1

				uint8_t payload[] = {
						0x00,	// version
						0x01,	// layers = RAM
						0x00, 0x00, // reserved

						0x02, 0x00, 0x74, 0x10, // id (reversed order of bytes)
						0x00, // value
				};

				// UBX-CFG-VALSET 0x06 0x8A
				gps_ack_set_pending(0x06, 0x8A);
				ubx_send(0x06, 0x8A, payload, sizeof(payload));
				gps_init_state = GPS_INIT_DISABLE_NMEA_WAIT_ACK;
				break;
			}
			case GPS_INIT_DISABLE_NMEA_WAIT_ACK:
			{
			    static uint8_t retries = 0;
				gps_ack_status_t st = gps_ack_check(0x06, 0x8A);

				if (st == GPS_ACK_OK) {
					gps_init_disable_nmea_done = 1;
					retries = 0;
					gps_init_state = GPS_INIT_SETUP_UBX_PVT;
				} else if ((st == GPS_ACK_NAK) || (st == GPS_ACK_TIMEOUT)) {
					if (retries < 3) {
						retries++;
						gps_init_state = GPS_INIT_DISABLE_NMEA;
					} else {
						retries = 0;
						gps_init_state = GPS_INIT_DONE;
					}
				}
				break;
			}
			case GPS_INIT_SETUP_UBX_PVT:
			{
				// CFG-RATE-MEAS 0x30210001 U2 val 100ms
				// CFG-RATE-NAV 0x30210002 U2 val 1
				// CFG-MSGOUT-UBX_NAV_PVT_UART1

				uint8_t payloadd[] = {
						0x00,	// version
						0x01,	// layers = RAM
						0x00, 0x00, // reserved

						0x01, 0x00, 0x21, 0x30, // CFG-RATE-MEAS
						0x64, 0x00, // U2 value 100

						0x02, 0x00, 0x21, 0x30, // CFG-RATE-NAV
						0x01, 0x00, // U2 value 1

						0x07, 0x00, 0x91, 0x20, // CFG-MSGOUT-UBX_NAV_PVT_UART1
						0x01, // U1 value
				};

				gps_ack_set_pending(0x06, 0x8A);
				ubx_send(0x06, 0x8A, payloadd, sizeof(payloadd));
				gps_init_state = GPS_INIT_SETUP_UBX_PVT_WAIT_ACK;
				break;
			}
			case GPS_INIT_SETUP_UBX_PVT_WAIT_ACK:
			{
				static uint8_t retries_setup = 0;
				gps_ack_status_t st = gps_ack_check(0x06, 0x8A);

				if (st == GPS_ACK_OK) {
					gps_init_setup_ubx_pvt_done = 1;
					retries_setup = 0;
					gps_init_state = GPS_INIT_SETUP_DYN_MODEL;
				} else if ((st == GPS_ACK_NAK) || (st == GPS_ACK_TIMEOUT)) {
					if (retries_setup < 3) {
						retries_setup++;
						gps_init_state = GPS_INIT_SETUP_UBX_PVT;
					} else {
						retries_setup = 0;
						gps_init_state = GPS_INIT_DONE;
					}
				}
				break;
			}
			case GPS_INIT_SETUP_DYN_MODEL:
			{
				// CFG-NAVSPG-DYNMODEL = 5 (Sea)
				// Key ID: 0x20110021, type U1
				uint8_t payload_dyn[] = {
					0x00,               // version
					0x01,               // layers = RAM
					0x00, 0x00,         // reserved
					0x21, 0x00, 0x11, 0x20,  // CFG-NAVSPG-DYNMODEL key (0x20110021, LE)
					0x05,               // Sea
				};
				gps_ack_set_pending(0x06, 0x8A);
				ubx_send(0x06, 0x8A, payload_dyn, sizeof(payload_dyn));
				gps_init_state = GPS_INIT_SETUP_DYN_MODEL_WAIT_ACK;
				break;
			}
			case GPS_INIT_SETUP_DYN_MODEL_WAIT_ACK:
			{
				static uint8_t retries_dyn = 0;
				gps_ack_status_t st = gps_ack_check(0x06, 0x8A);

				if (st == GPS_ACK_OK) {
					gps_init_setup_dyn_model_done = 1;
					retries_dyn = 0;
					gps_init_state = GPS_INIT_DONE;
				} else if ((st == GPS_ACK_NAK) || (st == GPS_ACK_TIMEOUT)) {
					if (retries_dyn < 3) {
						retries_dyn++;
						gps_init_state = GPS_INIT_SETUP_DYN_MODEL;
					} else {
						retries_dyn = 0;
						gps_init_state = GPS_INIT_DONE;
					}
				}
				break;
			}
			case GPS_INIT_DONE:
				break;
			default:
				break;
			}
		}

		osDelay(10);
		task_gps_alive++;
	}
}

