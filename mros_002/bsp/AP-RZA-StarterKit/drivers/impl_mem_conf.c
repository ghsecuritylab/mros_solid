#include <impl_mem_conf.h>

#include "memory_map.c"

void IMPL_MEM_GetConfig(SOLID_MEM_CONF **pp_list, int *p_len)
{
	int n = sizeof(myMemories) / sizeof(SOLID_MEM_CONF);
	*pp_list = (SOLID_MEM_CONF *)&myMemories;
	*p_len = n;
}
