#include 	"lwip/sys.h"

#include	<solid_timer.h>

u32_t
sys_now(void)
{
	return (u32_t)(0xffffffff & (( SOLID_TIMER_ToUsec(SOLID_TIMER_GetCurrentTick()) ) /1000 ));
}
