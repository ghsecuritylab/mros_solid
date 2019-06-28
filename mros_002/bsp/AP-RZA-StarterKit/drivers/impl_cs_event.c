#include "solid_cs_assert.h"
#include "impl_cs_event.h"

extern int IMPL_SYSTEM_IO_Initialize();

/* コアサービス内のイベントを通知します */
void IMPL_CS_EventNotify(enum SOLID_CS_EVENTID evtId, void* pInfo)
{
	switch (evtId)
	{
	case INIT_ERR:
		{
			SOLID_CS_EVENT_INFO_INIT_ERR* pErrInfo = (SOLID_CS_EVENT_INFO_INIT_ERR*)pInfo;
			solid_cs_assert(pErrInfo->ercd == SOLID_ERR_OK);
		} break;
	case INIT_IORES:
		IMPL_SYSTEM_IO_Initialize();
		break;

	default:
		break;
	}

}
