#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "impl_net_dev.h"

void IMPL_NET_DEV_get_macaddr(uint8_t macaddr[], size_t len)
{
	const static uint8_t	_myaddr[] = {0x00, 0x0c, 0x7b, 0x3b, 0x01, 0x0c };
	memcpy( macaddr, _myaddr, len);
}

