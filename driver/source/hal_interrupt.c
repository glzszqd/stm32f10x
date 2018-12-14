
#include <platform.h>
#include <hal.h>


#define VECTOR_MAX			(60)


hal_interrupt_desc_t interrupt_desc[VECTOR_MAX];


static void hal_inteerupt_dummy(int32_t vec, void *para)
{
	
}

int32_t hal_interrupt_init(void)
{
	int32_t i = 0;
	
	for(i = 0;i < VECTOR_MAX;i++)
	{
		interrupt_desc[i].fun = hal_inteerupt_dummy;
		interrupt_desc[i].para = NULL;
	}
	
	return 0;
}
int32_t hal_interrupt_mask(int32_t vec)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	if(vec >= VECTOR_MAX || vec < 0)
		return -1;
	
	NVIC_InitStructure.NVIC_IRQChannel = vec;  //嵌套中断通道为vec
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //使能中断
	NVIC_Init(&NVIC_InitStructure);  
	
	return 0;
}
int32_t hal_interrupt_umask(int32_t vec)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	if(vec >= VECTOR_MAX || vec < 0)
		return -1;
	
	NVIC_InitStructure.NVIC_IRQChannel = vec;  //嵌套中断通道为vec
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;     //禁止中断
	NVIC_Init(&NVIC_InitStructure);  

	return 0;
}
int32_t hal_interrupt_install(int32_t vec, hal_interrupt_t handler,void *para)
{
	if(vec >= VECTOR_MAX || vec < 0)
		return -1;
	
	interrupt_desc[vec].fun = handler;
	interrupt_desc[vec].para = para;
	
	return 0;
}

