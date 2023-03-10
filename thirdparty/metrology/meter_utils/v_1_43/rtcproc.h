/* =================================================================== */
#ifndef RTCPROC_H_INCLUDED
#define RTCPROC_H_INCLUDED

#include "compiler.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

#define RTC_12_HOUR_MODE  1
#define RTC_24_HOUR_MODE  0

typedef struct {
	uint32_t mode;  /* in 12H mode. 0:AM, 1:PM */
	uint32_t hour;  /* RTC hour */
	uint32_t minute; /* RTC minute */
	uint32_t second; /* RTC second */
} rtc_time_t;

typedef struct {
	uint32_t format; /* Format 0: English; 1:International. */
	uint32_t month; /* RTC month. */
	uint32_t day;   /* RTC day. */
	uint32_t year;  /* RTC year. */
	uint32_t week;  /* RTC day-of-the-week. */
} date_t, *p_date;

typedef struct {
	rtc_time_t time;
	date_t date;
} rtc_t;

typedef struct {
	uint32_t mode;  /* in 12H mode. 0:AM, 1:PM */
	uint32_t hour;  /* RTC hour */
	uint32_t minute; /* RTC minute */
	uint32_t second; /* RTC second */
} rtc_time_str;

typedef struct {
	uint32_t format; /* Format 0: English; 1:International. */
	uint32_t month; /* RTC month. */
	uint32_t day;   /* RTC day. */
	uint32_t year;  /* RTC year. */
	uint32_t week;  /* RTC day-of-the-week. */
} date_str;

typedef struct {
	rtc_time_t time;
	date_t date;

	uint8_t second;
	uint8_t minute;
	uint8_t hour;
	uint8_t day;
	uint8_t month;
	uint8_t year;
	uint8_t week;
} rtc_str;
#define RTC_SIZE (sizeof(rtc_t) / sizeof(uint32_t))

/* ---------------------variable declare------------------------------ */
extern const uint8_t TB_MonthDayTable[];
extern const uint8_t Month_WeekTable[];
extern rtc_str VRTC;
/* static rtc_t sx_rtc; */
/* static rtc_t *spx_rtc; */
/* =================================================================== */
uint8_t BCD_Verify( uint8_t BCDData );
uint8_t N_Byte_BCDVerify( uint8_t *ptr, uint8_t len );
uint8_t Time_FormatChk( uint8_t *PP_Time );
uint8_t Date_FormatChk( uint8_t *PP_Date );
uint8_t RTCData_Verify( uint8_t *ptr );

void    ReadRTCProcess( void );
void    rtc_init( void );

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */

#endif /* DISPLAY_H_INCLUDED */
