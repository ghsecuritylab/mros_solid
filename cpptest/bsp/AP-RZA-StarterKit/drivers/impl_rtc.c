#include "solid_type.h"
#include "solid_rtc.h"
#include "impl_rtc.h"

#include "dev_drv.h"
#include "devdrv_rtc.h"

// AP-RZA-1A では、クロック供給されないので RTC は動かない
// set した値がそのまま返る
/*******************************************************************************************/
/* Initialize RTC */
int IMPL_RTC_Init()
{
	int ret;
	(void)ret;
	rtc_time_t r_time = { {00, RTC_ENABLE}, {00, RTC_ENABLE}, {00, RTC_ENABLE},
							{0, RTC_ENABLE},
							{1, RTC_ENABLE}, {1, RTC_ENABLE}, {2001,RTC_ENABLE}};

	// initialize and start operation
	R_RTC_Init();
	R_RTC_Open();

	// initialize RTC values
	ret = R_RTC_SetCnt(&r_time);

	return SOLID_ERR_OK;
}

/* Read RTC registers */
int IMPL_RTC_ReadTime(SOLID_RTC_TIME * time)
{
	rtc_time_t r_time;
	int32_t ret;
	int result = SOLID_ERR_OK;

	r_time.second.enable = RTC_ENABLE;
	r_time.minute.enable = RTC_ENABLE;
	r_time.hour.enable = RTC_ENABLE;
	r_time.day.enable = RTC_ENABLE;
	r_time.month.enable = RTC_ENABLE;
	r_time.year.enable = RTC_ENABLE;
	r_time.week.enable = RTC_ENABLE;

	ret = R_RTC_GetCnt(&r_time);

	if (ret == DEVDRV_SUCCESS)
	{
		time->tm_sec = r_time.second.value;
		time->tm_min = r_time.minute.value;
		time->tm_hour = r_time.hour.value;
		time->tm_mday = r_time.day.value;
		time->tm_mon = r_time.month.value;
		time->tm_year = r_time.year.value;
		time->tm_wday = r_time.week.value;
	} else {
		result = SOLID_ERR_NORES;
	}

	return result;
}

/* Set RTC registers */
int IMPL_RTC_SetTime(SOLID_RTC_TIME * time)
{
	rtc_time_t r_time;
	int32_t ret;
	int result = SOLID_ERR_OK;

	r_time.second.value = time->tm_sec;
	r_time.minute.value = time->tm_min;
	r_time.hour.value = time->tm_hour;
	r_time.day.value = time->tm_mday;
	r_time.month.value = time->tm_mon;
	r_time.year.value = time->tm_year;
	r_time.week.value = time->tm_wday;
	r_time.second.enable = RTC_ENABLE;
	r_time.minute.enable = RTC_ENABLE;
	r_time.hour.enable = RTC_ENABLE;
	r_time.day.enable = RTC_ENABLE;
	r_time.month.enable = RTC_ENABLE;
	r_time.year.enable = RTC_ENABLE;
	r_time.week.enable = RTC_ENABLE;

	ret = R_RTC_SetCnt(&r_time);

	if (ret != DEVDRV_SUCCESS)
		result = SOLID_ERR_NORES;

	return result;
}
