#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "Board/LEDs.h"
#include "Log.h"
#include "Main.h"
#include "Power.h"
#include "Timer.h"
#include "uart.h"
#include "UBX.h"

#include "md5/md5.h"

#define ABS(a)   ((a) < 0     ? -(a) : (a))
#define MIN(a,b) (((a) < (b)) ?  (a) : (b))
#define MAX(a,b) (((a) > (b)) ?  (a) : (b))

#define UBX_INVALID_VALUE   INT32_MAX

#define UBX_TIMEOUT         500 // ACK/NAK timeout (ms)
#define UBX_MAX_PAYLOAD_LEN 64

#define UBX_SYNC_1          0xb5
#define UBX_SYNC_2          0x62

#define UBX_NAV             0x01
#define UBX_NAV_POSLLH      0x02
#define UBX_NAV_STATUS      0x03
#define UBX_NAV_SOL         0x06
#define UBX_NAV_VELNED      0x12
#define UBX_NAV_TIMEUTC     0x21

#define UBX_ACK             0x05
#define UBX_ACK_NAK         0x00
#define UBX_ACK_ACK         0x01

#define UBX_CFG             0x06
#define UBX_CFG_PRT         0x00
#define UBX_CFG_MSG         0x01
#define UBX_CFG_RST         0x04
#define UBX_CFG_RATE        0x08
#define UBX_CFG_NAV5        0x24

#define UBX_NMEA            0xf0
#define UBX_NMEA_GPGGA      0x00
#define UBX_NMEA_GPGLL      0x01
#define UBX_NMEA_GPGSA      0x02
#define UBX_NMEA_GPGSV      0x03
#define UBX_NMEA_GPRMC      0x04
#define UBX_NMEA_GPVTG      0x05

#define UBX_UNITS_KMH       0
#define UBX_UNITS_MPH       1

#define UBX_BUFFER_LEN      4

#define UBX_MSG_POSLLH      0x01
#define UBX_MSG_SOL         0x02
#define UBX_MSG_VELNED      0x04
#define UBX_MSG_TIMEUTC     0x08
#define UBX_MSG_ALL         (UBX_MSG_POSLLH | UBX_MSG_SOL | UBX_MSG_VELNED | UBX_MSG_TIMEUTC)

static uint8_t  UBX_msg_class;
static uint8_t  UBX_msg_id;
static uint16_t UBX_payload_len;
static uint8_t  UBX_payload[UBX_MAX_PAYLOAD_LEN];

typedef struct
{
	uint8_t msgClass;  // Message class
	uint8_t msgID;     // Message identifier
	uint8_t rate;      // Send rate
}
UBX_cfg_msg;

typedef struct
{
	uint8_t  portID;       // Port identifier number
	uint8_t  reserved0;    // Reserved
	uint16_t txReady;      // TX ready pin configuration
	uint32_t mode;         // UART mode
	uint32_t baudRate;     // Baud rate (bits/sec)
	uint16_t inProtoMask;  // Input protocols
	uint16_t outProtoMask; // Output protocols
	uint16_t flags;        // Flags
	uint16_t reserved5;    // Always set to zero
}
UBX_cfg_prt;

typedef struct
{
	uint16_t measRate; // Measurement rate             (ms)
	uint16_t navRate;  // Nagivation rate, in number 
	                   //   of measurement cycles
	uint16_t timeRef;  // Alignment to reference time:
	                   //   0 = UTC time; 1 = GPS time
}
UBX_cfg_rate;

typedef struct
{
	uint16_t navBbrMask; // BBR sections to clear
	uint8_t  resetMode;  // Reset type
	uint8_t  res;        // Reserved
}
UBX_cfg_rst;

typedef struct
{
	uint16_t mask;             // Only masked parameters will be applied
	uint8_t  dynModel;         // Dynamic platform model
	uint8_t  fixMode;          // Position fixing mode
	int32_t  fixedAlt;         // Fixed altitude (MSL) for 2D mode       (m)
	uint32_t fixedAltVar;      // Fixed altitude variance for 2D mode    (m^2)
	int8_t   minElev;          // Minimum elevation for satellite        (deg)
	uint8_t  drLimit;          // Maximum time to perform dead reckoning (s)
	uint16_t pDop;             // Position DOP mask
	uint16_t tDop;             // Time DOP mask
	uint16_t pAcc;             // Position accuracy mask                 (m)
	uint16_t tAcc;             // Time accuracy mask                     (m)
	uint8_t  staticHoldThresh; // Static hold threshold                  (cm/s)
	uint8_t  res1;             // Reserved, set to 0
	uint32_t res2;             // Reserved, set to 0
	uint32_t res3;             // Reserved, set to 0
	uint32_t res4;             // Reserved, set to 0
}
UBX_cfg_nav5;

typedef struct
{
	uint32_t iTOW;     // GPS time of week             (ms)
	int32_t  lon;      // Longitude                    (deg)
	int32_t  lat;      // Latitude                     (deg)
	int32_t  height;   // Height above ellipsoid       (mm)
	int32_t  hMSL;     // Height above mean sea level  (mm)
	uint32_t hAcc;     // Horizontal accuracy estimate (mm)
	uint32_t vAcc;     // Vertical accuracy estimate   (mm)
}
UBX_nav_posllh;

typedef struct
{
	uint32_t iTOW;     // GPS time of week             (ms)
	uint8_t  gpsFix;   // GPS fix type
	uint8_t  flags;    // Navigation status flags
	uint8_t  diffStat; // Differential status
	uint8_t  res;      // Reserved
	uint32_t ttff;     // Time to first fix            (ms)
	uint32_t msss;     // Time since startup           (ms)
}
UBX_nav_status;

typedef struct
{
	uint32_t iTOW;     // GPS time of week             (ms)
	int32_t  fTOW;     // Fractional nanoseconds       (ns)
	int16_t  week;     // GPS week
	uint8_t  gpsFix;   // GPS fix type
	uint8_t  flags;    // Fix status flags
	int32_t  ecefX;    // ECEF X coordinate            (cm)
	int32_t  ecefY;    // ECEF Y coordinate            (cm)
	int32_t  ecefZ;    // ECEF Z coordinate            (cm)
	uint32_t pAcc;     // 3D position accuracy         (cm)
	int32_t  ecefVX;   // ECEF X velocity              (cm/s)
	int32_t  ecefVY;   // ECEF Y velocity              (cm/s)
	int32_t  ecefVZ;   // ECEF Z velocity              (cm/s)
	uint32_t sAcc;     // Speed accuracy               (cm/s)
	uint16_t pDOP;     // Position DOP
	uint8_t  res1;     // Reserved
	uint8_t  numSV;    // Number of SVs in solution
	uint32_t res2;     // Reserved
}
UBX_nav_sol;

typedef struct
{
	uint32_t iTOW;     // GPS time of week             (ms)
	int32_t  velN;     // North velocity               (cm/s)
	int32_t  velE;     // East velocity                (cm/s)
	int32_t  velD;     // Down velocity                (cm/s)
	uint32_t speed;    // 3D speed                     (cm/s)
	uint32_t gSpeed;   // Ground speed                 (cm/s)
	int32_t  heading;  // 2D heading                   (deg)
	uint32_t sAcc;     // Speed accuracy estimate      (cm/s)
	uint32_t cAcc;     // Heading accuracy estimate    (deg)
}
UBX_nav_velned;

typedef struct
{
	uint32_t iTOW;     // GPS time of week             (ms)
	uint32_t tAcc;     // Time accuracy estimate       (ns)
	int32_t  nano;     // Nanoseconds of second        (ns)
	uint16_t year;     // Year                         (1999..2099)
	uint8_t  month;    // Month                        (1..12)
	uint8_t  day;      // Day of month                 (1..31)
	uint8_t  hour;     // Hour of day                  (0..23)
	uint8_t  min;      // Minute of hour               (0..59)
	uint8_t  sec;      // Second of minute             (0..59)
	uint8_t  valid;    // Validity flags
}
UBX_nav_timeutc;

typedef struct
{
	uint8_t clsID;     // Class ID of acknowledged message
	uint8_t msgID;     // Message ID of acknowledged message
}
UBX_ack_ack;

typedef struct
{
	uint8_t clsID;     // Class ID of not-acknowledged message
	uint8_t msgID;     // Message ID of not-acknowledged message
}
UBX_ack_nak;

uint8_t  UBX_model         = 6;
uint16_t UBX_rate          = 200;

static uint32_t UBX_time_of_week = 0;
static uint8_t  UBX_msg_received = 0;

char UBX_buf[150];

typedef struct
{
	int32_t  lon;      // Longitude                    (deg)
	int32_t  lat;      // Latitude                     (deg)
	int32_t  hMSL;     // Height above mean sea level  (mm)
	uint32_t hAcc;     // Horizontal accuracy estimate (mm)
	uint32_t vAcc;     // Vertical accuracy estimate   (mm)

	uint8_t  gpsFix;   // GPS fix type
	uint8_t  numSV;    // Number of SVs in solution

	int32_t  velN;     // North velocity               (cm/s)
	int32_t  velE;     // East velocity                (cm/s)
	int32_t  velD;     // Down velocity                (cm/s)
	uint32_t speed;    // 3D speed                     (cm/s)
	uint32_t gSpeed;   // Ground speed                 (cm/s)
	int32_t  heading;  // 2D heading                   (deg)
	uint32_t sAcc;     // Speed accuracy estimate      (cm/s)
	uint32_t cAcc;     // Heading accuracy estimate    (deg)

	int32_t  nano;     // Nanoseconds of second        (ns)
	uint16_t year;     // Year                         (1999..2099)
	uint8_t  month;    // Month                        (1..12)
	uint8_t  day;      // Day of month                 (1..31)
	uint8_t  hour;     // Hour of day                  (0..23)
	uint8_t  min;      // Minute of hour               (0..59)
	uint8_t  sec;      // Second of minute             (0..59)
}
UBX_saved_t ;
static UBX_saved_t UBX_saved[UBX_BUFFER_LEN];

static uint8_t UBX_read  = 0;
static uint8_t UBX_write = 0;

static volatile uint8_t UBX_hasFix = 0;

static const char UBX_header[] PROGMEM = 
	"time,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,heading,cAcc,gpsFix,numSV\r\n"
	",(deg),(deg),(m),(m/s),(m/s),(m/s),(m),(m),(m/s),(deg),(deg),,\r\n";

static enum
{
	st_idle,
	st_flush_1,
	st_flush_2,
	st_flush_3,
	st_signature,
	st_flush_4,
	st_flush_5,
	st_flush_6
}
UBX_state = st_idle;

static md5_ctx_t  UBX_md5_ctx;
static uint8_t    UBX_md5_block[MD5_BLOCK_BYTES];
static uint8_t    UBX_md5_block_free = MD5_BLOCK_BYTES;
static md5_hash_t UBX_md5_hash;

extern int disk_is_ready(void);

void UBX_Update(void)
{
	static uint16_t counter;

	static enum
	{
		st_solid,
		st_blinking
	}
	state = st_solid;

	switch (state)
	{
	case st_solid:
		if (UBX_hasFix)
		{
			counter = 0;
			state = st_blinking;
		}
		break;
	case st_blinking:
		if (!UBX_hasFix)
		{
			LEDs_ChangeLEDs(LEDS_ALL_LEDS, Main_activeLED);
			state = st_solid;
		}
		break;
	}
	
	if (state == st_blinking)
	{
		if (counter == 0)
		{
			LEDs_ChangeLEDs(LEDS_ALL_LEDS, 0);
		}
		else if (counter == 900)
		{
			LEDs_ChangeLEDs(LEDS_ALL_LEDS, Main_activeLED);
		}

		counter = (counter + 1) % 1000;
	}
}

static uint8_t UBX_HandleByte(
	unsigned char ch)
{
	uint8_t ret = 0;

	static enum
	{
		st_sync_1, 
		st_sync_2, 
		st_class, 
		st_id,
		st_length_1,
		st_length_2,
		st_payload,
		st_ck_a,
		st_ck_b
	}
	state = st_sync_1;
	
	static uint8_t ck_a, ck_b;
	static uint16_t index;
	
	switch (state)
	{
	case st_sync_1:
		if (ch == UBX_SYNC_1)
		{
			state = st_sync_2;
		}
		break;
	case st_sync_2:
		if (ch == UBX_SYNC_2)
		{
			state = st_class;
		}
		else
		{
			state = st_sync_1;
		}
		break;
	case st_class:
		UBX_msg_class = ch;
		ck_a = ck_b = ch;
		state = st_id;
		break;
	case st_id:
		UBX_msg_id = ch;
		ck_a += ch;
		ck_b += ck_a;
		state = st_length_1;
		break;
	case st_length_1:
		UBX_payload_len = ch;
		ck_a += ch;
		ck_b += ck_a;
		state = st_length_2;
		break;
	case st_length_2:
		UBX_payload_len += ch << 8;
		ck_a += ch;
		ck_b += ck_a;
		if (UBX_payload_len == 0)
		{
			state = st_ck_a;
		}
		else if (UBX_payload_len <= UBX_MAX_PAYLOAD_LEN)
		{
			state = st_payload;
			index = 0;
		}
		else
		{
			state = st_sync_1;
		}
		break;
	case st_payload:
		UBX_payload[index++] = ch;
		ck_a += ch;
		ck_b += ck_a;
		if (index == UBX_payload_len)
		{
			state = st_ck_a;
		}
		break;
	case st_ck_a:
		if (ck_a == ch)
		{
			state = st_ck_b;
		}
		else
		{
			state = st_sync_1;
		}
		break;
	case st_ck_b:
		if (ck_b == ch)
		{
			ret = 1;
		}
		state = st_sync_1;
		break;
	}

	return ret;
}

static uint8_t UBX_WaitForAck(
	uint8_t  msg_class,
	uint8_t  msg_id,
	uint16_t timeout)
{
	unsigned int ch;

	Timer_Set(timeout);
	
	while (Timer_Get() != 0)
	{
		if ((ch = uart_getc()) != UART_NO_DATA)
		{
			if (UBX_HandleByte(ch))
			{
				if (UBX_msg_class == UBX_ACK &&
				    UBX_msg_id == UBX_ACK_ACK)
				{
					UBX_ack_ack ack = *((UBX_ack_ack *) UBX_payload);
					if (ack.clsID == msg_class &&
					    ack.msgID == msg_id)
					{
						return 1; // ACK
					}
				}
				else if (UBX_msg_class == UBX_ACK &&
				         UBX_msg_id == UBX_ACK_NAK)
				{
					UBX_ack_nak nak = *((UBX_ack_nak *) UBX_payload);
					if (nak.clsID == msg_class &&
					    nak.msgID == msg_id)
					{
						return 0; // NAK
					}
				}
			}
		}
	}

	return 0;
}

static void UBX_SendMessage(
	uint8_t  msg_class,
	uint8_t  msg_id,
	uint16_t size,
	void     *data)
{
	uint16_t i;
	uint8_t  *bytes = (uint8_t *) data;
	uint8_t  ck_a = 0, ck_b = 0;

	uart_putc(UBX_SYNC_1);
	uart_putc(UBX_SYNC_2);

	#define SEND_BYTE(a) uart_putc(a); ck_a += a; ck_b += ck_a;

	SEND_BYTE(msg_class);
	SEND_BYTE(msg_id);

	SEND_BYTE(size & 0xff);
	SEND_BYTE((size >> 8) & 0xff);

	for (i = 0; i < size; ++i)
	{
		SEND_BYTE(bytes[i]);
	}

	#undef SEND_BYTE
	
	uart_putc(ck_a);
	uart_putc(ck_b);
}

static void UBX_UpdateSignature(
	const char *ptr, 
	size_t len)
{
	md5_ctx_t saved_md5_ctx;

	// Handle complete blocks
	while (len > UBX_md5_block_free)
	{
		memcpy(UBX_md5_block + MD5_BLOCK_BYTES - UBX_md5_block_free, ptr, UBX_md5_block_free);
		ptr += UBX_md5_block_free;
		len -= UBX_md5_block_free;
		UBX_md5_block_free = MD5_BLOCK_BYTES;

		md5_nextBlock(&UBX_md5_ctx, UBX_md5_block);
	}

	// Save MD5 context
	saved_md5_ctx = UBX_md5_ctx;

	// Handle partial block
	memcpy(UBX_md5_block + MD5_BLOCK_BYTES - UBX_md5_block_free, ptr, len);
	UBX_md5_block_free -= len;

	md5_lastBlock(&UBX_md5_ctx, UBX_md5_block, (MD5_BLOCK_BYTES - UBX_md5_block_free) * 8);
	md5_ctx2hash(&UBX_md5_hash, &UBX_md5_ctx);

	// Restore MD5 context
	UBX_md5_ctx = saved_md5_ctx;
}

static void UBX_UpdateSignature_P(
	const char *str)
{
	char *ptr;
	size_t len;
	
	ptr = UBX_buf;
	len = 0;

	while ((*(ptr++) = pgm_read_byte(str++)))
	{
		++len;
	}
	
	UBX_UpdateSignature(UBX_buf, len);
}

static void UBX_ReceiveMessage(
	uint8_t msg_received, 
	uint32_t time_of_week)
{
	UBX_saved_t *current = UBX_saved + (UBX_write % UBX_BUFFER_LEN);

	if (time_of_week != UBX_time_of_week)
	{
		UBX_time_of_week = time_of_week;
		UBX_msg_received = 0;
	}

	UBX_msg_received |= msg_received;

	if (UBX_msg_received == UBX_MSG_ALL)
	{
		if (current->gpsFix == 0x03)
		{
			UBX_hasFix = 1;

			if (!Log_IsInitialized())
			{
				Power_Hold();

				Log_Init(
					current->year,
					current->month,
					current->day,
					current->hour,
					current->min,
					current->sec);

				Log_WriteString(UBX_header);
				UBX_UpdateSignature_P(UBX_header);

				UBX_state = st_flush_1;
			}

			++UBX_write;
		}
		else
		{
			UBX_hasFix = 0;
		}
		
		UBX_msg_received = 0;
	}
}

static void UBX_HandleNavSol(void)
{
	UBX_saved_t *current = UBX_saved + (UBX_write % UBX_BUFFER_LEN);
	UBX_nav_sol *nav_sol = (UBX_nav_sol *) UBX_payload;

	current->gpsFix = nav_sol->gpsFix;
	current->numSV  = nav_sol->numSV;

	UBX_ReceiveMessage(UBX_MSG_SOL, nav_sol->iTOW);
}

static void UBX_HandlePosition(void)
{
	UBX_saved_t *current = UBX_saved + (UBX_write % UBX_BUFFER_LEN);
	UBX_nav_posllh *nav_pos_llh = (UBX_nav_posllh *) UBX_payload;

	current->lon  = nav_pos_llh->lon;
	current->lat  = nav_pos_llh->lat;
	current->hMSL = nav_pos_llh->hMSL;
	current->hAcc = nav_pos_llh->hAcc;
	current->vAcc = nav_pos_llh->vAcc;

	UBX_ReceiveMessage(UBX_MSG_POSLLH, nav_pos_llh->iTOW);
}

static void UBX_HandleVelocity(void)
{
	UBX_saved_t *current = UBX_saved + (UBX_write % UBX_BUFFER_LEN);
	UBX_nav_velned *nav_velned = (UBX_nav_velned *) UBX_payload;

	current->velN    = nav_velned->velN;
	current->velE    = nav_velned->velE;
	current->velD    = nav_velned->velD;
	current->speed   = nav_velned->speed;
	current->gSpeed  = nav_velned->gSpeed;
	current->heading = nav_velned->heading;
	current->sAcc    = nav_velned->sAcc;
	current->cAcc    = nav_velned->cAcc;

	UBX_ReceiveMessage(UBX_MSG_VELNED, nav_velned->iTOW);
}

static void UBX_HandleTimeUTC(void)
{
	UBX_saved_t *current = UBX_saved + (UBX_write % UBX_BUFFER_LEN);
	UBX_nav_timeutc *nav_timeutc = (UBX_nav_timeutc *) UBX_payload;

	current->nano  = nav_timeutc->nano;
	current->year  = nav_timeutc->year;
	current->month = nav_timeutc->month;
	current->day   = nav_timeutc->day;
	current->hour  = nav_timeutc->hour;
	current->min   = nav_timeutc->min;
	current->sec   = nav_timeutc->sec;

	UBX_ReceiveMessage(UBX_MSG_TIMEUTC, nav_timeutc->iTOW);
}

static void UBX_HandleMessage(void)
{
	if ((uint8_t) (UBX_read + UBX_BUFFER_LEN) == UBX_write)
	{
		++UBX_read;
	}

	switch (UBX_msg_class)
	{
	case UBX_NAV:
		switch (UBX_msg_id)
		{
		case UBX_NAV_SOL:
			UBX_HandleNavSol();
			break;
		case UBX_NAV_POSLLH:
			UBX_HandlePosition();
			break;
		case UBX_NAV_VELNED:
			UBX_HandleVelocity();
			break;
		case UBX_NAV_TIMEUTC:
			UBX_HandleTimeUTC();
			break;
		}
		break;
	}
}

void UBX_Init(void)
{
	UBX_cfg_msg cfg_msg[] =
	{
		{UBX_NMEA, UBX_NMEA_GPGGA,  0},
		{UBX_NMEA, UBX_NMEA_GPGLL,  0},
		{UBX_NMEA, UBX_NMEA_GPGSA,  0},
		{UBX_NMEA, UBX_NMEA_GPGSV,  0},
		{UBX_NMEA, UBX_NMEA_GPRMC,  0},
		{UBX_NMEA, UBX_NMEA_GPVTG,  0},
		{UBX_NAV,  UBX_NAV_POSLLH,  1},
		{UBX_NAV,  UBX_NAV_VELNED,  1},
		{UBX_NAV,  UBX_NAV_SOL,     1},
		{UBX_NAV,  UBX_NAV_TIMEUTC, 1}
	};

	size_t n = sizeof(cfg_msg) / sizeof(UBX_cfg_msg);
	size_t i;

	UBX_cfg_rate cfg_rate =
	{
		.measRate   = UBX_rate, // Measurement rate (ms)
		.navRate    = 1,        // Navigation rate (cycles)
		.timeRef    = 0         // UTC time
	};

	UBX_cfg_rst cfg_rst =
	{
		.navBbrMask = 0x0000,   // Hot start
		.resetMode  = 0x09      // Controlled GPS start
	};
	
	UBX_cfg_nav5 cfg_nav5 =
	{
		.mask       = 0x0001,   // Apply dynamic model settings
		.dynModel   = UBX_model // Airborne with < 1 g acceleration
	};
	
	UBX_cfg_prt cfg_prt =
	{
		.portID       = 1,      // UART 1
		.reserved0    = 0,      // Reserved
		.txReady      = 0,      // no TX ready
		.mode         = 0x08d0, // 8N1
		.baudRate     = 38400,  // Baudrate in bits/second
		.inProtoMask  = 0x0001, // UBX protocol
		.outProtoMask = 0x0001, // UBX protocol
		.flags        = 0,      // Flags bit mask
		.reserved5    = 0       // Reserved, set to 0
	};

	uint8_t success = 1;
	
	uart_init(51); // 9600 baud
	
	UBX_SendMessage(UBX_CFG, UBX_CFG_PRT, sizeof(cfg_prt), &cfg_prt);

	// NOTE: We don't wait for ACK here since some FlySights will already be
	//       set to 38400 baud.
	
	while (!uart_tx_empty());

	uart_init(12); // 38400 baud

	_delay_ms(10); // wait for GPS UART to reset
	
	UBX_SendMessage(UBX_CFG, UBX_CFG_PRT, sizeof(cfg_prt), &cfg_prt);
	if (!UBX_WaitForAck(UBX_CFG, UBX_CFG_PRT, UBX_TIMEOUT)) success = 0;

	#define SEND_MESSAGE(c,m,d) { \
		UBX_SendMessage(c,m,sizeof(d),&d); \
		if (!UBX_WaitForAck(c,m,UBX_TIMEOUT)) success = 0; }

	for (i = 0; i < n; ++i)
	{
		SEND_MESSAGE(UBX_CFG, UBX_CFG_MSG, cfg_msg[i]);
	}
	
	SEND_MESSAGE(UBX_CFG, UBX_CFG_RATE, cfg_rate);
	SEND_MESSAGE(UBX_CFG, UBX_CFG_NAV5, cfg_nav5);
	SEND_MESSAGE(UBX_CFG, UBX_CFG_RST,  cfg_rst);
	
	#undef SEND_MESSAGE

	if (!success)
	{
		LEDs_ChangeLEDs(LEDS_ALL_LEDS, LEDS_RED);
		while (1);
	}

	// Initialize cryptography library
	md5_init(&UBX_md5_ctx);
}

void UBX_Task(void)
{
	unsigned int ch;

	UBX_saved_t *current;
	char *ptr;

	while (!((ch = uart_getc()) & UART_NO_DATA))
	{
		if (UBX_HandleByte(ch))
		{
			UBX_HandleMessage();
		}
	}
	
	switch (UBX_state)
	{
	case st_idle:
		if (disk_is_ready() && UBX_read != UBX_write)
		{
			current = UBX_saved + (UBX_read % UBX_BUFFER_LEN);

			Power_Hold();

			ptr = UBX_buf + sizeof(UBX_buf);

			*(--ptr) = 0;
			*(--ptr) = '\n';

			ptr = Log_WriteInt32ToBuf(ptr, current->numSV,     0, 0, '\r');
			ptr = Log_WriteInt32ToBuf(ptr, current->gpsFix,    0, 0, ',');
			ptr = Log_WriteInt32ToBuf(ptr, current->cAcc,   5, 1, ',');
			ptr = Log_WriteInt32ToBuf(ptr, current->heading, 5, 1, ',');
			ptr = Log_WriteInt32ToBuf(ptr, current->sAcc,   2, 1, ',');
			ptr = Log_WriteInt32ToBuf(ptr, current->vAcc,  3, 1, ',');
			ptr = Log_WriteInt32ToBuf(ptr, current->hAcc,  3, 1, ',');
			ptr = Log_WriteInt32ToBuf(ptr, current->velD,   2, 1, ',');
			ptr = Log_WriteInt32ToBuf(ptr, current->velE,   2, 1, ',');
			ptr = Log_WriteInt32ToBuf(ptr, current->velN,   2, 1, ',');
			ptr = Log_WriteInt32ToBuf(ptr, current->hMSL,  3, 1, ',');
			ptr = Log_WriteInt32ToBuf(ptr, current->lon,   7, 1, ',');
			ptr = Log_WriteInt32ToBuf(ptr, current->lat,   7, 1, ',');
			*(--ptr) = ',';
			ptr = Log_WriteInt32ToBuf(ptr, (current->nano + 5000000) / 10000000, 2, 0, 'Z');
			ptr = Log_WriteInt32ToBuf(ptr, current->sec,   2, 0, '.');
			ptr = Log_WriteInt32ToBuf(ptr, current->min,   2, 0, ':');
			ptr = Log_WriteInt32ToBuf(ptr, current->hour,  2, 0, ':');
			ptr = Log_WriteInt32ToBuf(ptr, current->day,   2, 0, 'T');
			ptr = Log_WriteInt32ToBuf(ptr, current->month, 2, 0, '-');
			ptr = Log_WriteInt32ToBuf(ptr, current->year,  4, 0, '-');
			++UBX_read;

			f_puts(ptr, &Main_file);
			UBX_UpdateSignature(ptr, UBX_buf + sizeof(UBX_buf) - ptr - 1);

			UBX_state = st_flush_1;
		}
		break;
	case st_flush_1:
		if (disk_is_ready())
		{
			f_sync_1(&Main_file);
			UBX_state = st_flush_2;
		}
		break;
	case st_flush_2:
		if (disk_is_ready())
		{
			f_sync_2(&Main_file);
			UBX_state = st_flush_3;
		}
		break;
	case st_flush_3:
		if (disk_is_ready())
		{
			f_sync_3(&Main_file);
			UBX_state = st_signature;
		}
		break;
	case st_signature:
		if (disk_is_ready())
		{
			ptr = UBX_buf + sizeof(UBX_buf);

			*(--ptr) = 0;
			*(--ptr) = '\n';
			*(--ptr) = '\r';

			ptr = Log_WriteHexToBuf(ptr, &UBX_md5_hash, MD5_HASH_BYTES);
			f_lseek(&Main_signature_file, 0);
			f_puts(ptr, &Main_signature_file);

			UBX_state = st_flush_4;
		}
		break;
	case st_flush_4:
		if (disk_is_ready())
		{
			f_sync_1(&Main_signature_file);
			UBX_state = st_flush_5;
		}
		break;
	case st_flush_5:
		if (disk_is_ready())
		{
			f_sync_2(&Main_signature_file);
			UBX_state = st_flush_6;
		}
		break;
	case st_flush_6:
		if (disk_is_ready())
		{
			f_sync_3(&Main_signature_file);
			Power_Release();
			UBX_state = st_idle;
		}
		break;
	}
}
