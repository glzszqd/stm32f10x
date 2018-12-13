
#include <hal/hal.h>

#define MCU_PCLK2		72000000


#define BLDC_PORT0_PWM_TIM			TIM8
#define GPIO_FullRemap_BLDC_PORT0_PWM_TIM	
#define BLDC_PORT0_PWM_RCC_APBPeriph_TIM		RCC_APB2Periph_TIM8
#define BLDC_PORT0_PWM_RCC_APBPeriph_GPIO		RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
#define BLDC_PORT0_PWM_GPIO_OC1		GPIOC 
#define BLDC_PORT0_PWM_GPIO_OC1N		GPIOA
#define BLDC_PORT0_PWM_GPIO_OC2		GPIOC 
#define BLDC_PORT0_PWM_GPIO_OC2N		GPIOB
#define BLDC_PORT0_PWM_GPIO_OC3		GPIOC 
#define BLDC_PORT0_PWM_GPIO_OC3N		GPIOB
#define BLDC_PORT0_PWM_GPIO_BKIN		GPIOA
#define BLDC_PORT0_PWM_GPIO_PIN_OC1		GPIO_Pin_6 
#define BLDC_PORT0_PWM_GPIO_PIN_OC1N		GPIO_Pin_7
#define BLDC_PORT0_PWM_GPIO_PIN_OC2		GPIO_Pin_7 
#define BLDC_PORT0_PWM_GPIO_PIN_OC2N		GPIO_Pin_0 
#define BLDC_PORT0_PWM_GPIO_PIN_OC3		GPIO_Pin_8 
#define BLDC_PORT0_PWM_GPIO_PIN_OC3N		GPIO_Pin_1
#define BLDC_PORT0_PWM_GPIO_PIN_BKIN		GPIO_Pin_6
#define BLDC_PORT0_PWM_FREQ		16000
#define BLDC_PORT0_PWM_DEADTIME		0x88
#define BLDC_PORT0_PWM_TIM_FLAG		TIM_FLAG_COM
#define BLDC_PORT0_PWM_TIM_IT		TIM_IT_COM
#define BLDC_PORT0_PWM_TIM_IRQ				TIM8_TRG_COM_IRQn


#deifne BLDC_PORT0_HALL_TIM		TIM4
#deifne BLDC_PORT0_HALL_GPIO_PORT0	GPIOB
#define BLDC_PORT0_HALL_RCC_APBPeriph_TIM		RCC_APB1Periph_TIM4
#define BLDC_PORT0_HALL_RCC_APBPeriph_GPIO		RCC_APB2Periph_GPIOB
#deifne BLDC_PORT0_HALL_GPIO_PORT0	GPIOB
#define BLDC_PORT0_HALL_GPIO_PIN_A	GPIO_Pin_6
#define BLDC_PORT0_HALL_GPIO_PIN_B	GPIO_Pin_7
#define BLDC_PORT0_HALL_GPIO_PIN_C	GPIO_Pin_8
#define BLDC_PORT0_HALL_TIM_PRESCALER	287
#define BLDC_PORT0_HALL_TIM_PERIOD	65535
#define BLDC_PORT0_HALL_TIM_FLAG		 TIM_FLAG_CC1|TIM_FLAG_CC2
#define BLDC_PORT0_HALL_TIM_IT		 TIM_IT_CC1|TIM_IT_CC2
#define BLDC_PORT0_HALL_TIM_IRQ		TIM4_IRQn


#define BLDC_PORT1_PWM_TIM			TIM1
#define BLDC_PORT1_PWM_GPIO_FullRemap_TIM	GPIO_FullRemap_TIM1
#define BLDC_PORT1_PWM_RCC_APBPeriph_TIM		RCC_APB2Periph_TIM1
#define BLDC_PORT1_PWMRCC_APBPeriph__GPIO		RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO
#define BLDC_PORT1_PWM_GPIO_OC1		GPIOE 
#define BLDC_PORT1_PWM_GPIO_OC1N		GPIOE
#define BLDC_PORT1_PWM_GPIO_OC2		GPIOE 
#define BLDC_PORT1_PWM_GPIO_OC2N		GPIOE
#define BLDC_PORT1_PWM_GPIO_OC3		GPIOE 
#define BLDC_PORT1_PWM_GPIO_OC3N		GPIOE
#define BLDC_PORT1_PWM_GPIO_BKIN		GPIOE
#define BLDC_PORT1_PWM_GPIO_PIN_OC1		GPIO_Pin_8 
#define BLDC_PORT1_PWM_GPIO_PIN_OC1N		GPIO_Pin_9
#define BLDC_PORT1_PWM_GPIO_PIN_OC2		GPIO_Pin_10 
#define BLDC_PORT1_PWM_GPIO_PIN_OC2N		GPIO_Pin_11
#define BLDC_PORT1_PWM_GPIO_PIN_OC3		GPIO_Pin_12 
#define BLDC_PORT1_PWM_GPIO_PIN_OC3N		GPIO_Pin_13
#define BLDC_PORT1_PWM_GPIO_PIN_BKIN		GPIO_Pin_15
#define BLDC_PORT1_PWM_FREQ				16000
#define BLDC_PORT1_PWM_PPR		 (CALC_PWM_FREQ_CENTER_ALIGNED(MCU_PCLK2,BLDC_PORT1_PWM_FREQ))
#define BLDC_PORT1_PWM_DEADTIME		0x88
#define BLDC_PORT1_PWM_TIM_FLAG		TIM_FLAG_Update //TIM_FLAG_COM
#define BLDC_PORT1_PWM_TIM_IT		TIM_IT_Update // TIM_IT_COM
#define BLDC_PORT1_PWM_TIM_IRQ		TIM1_UP_IRQn // TIM1_TRG_COM_IRQn

#define BLDC_PORT1_HALL_TIM		TIM5
#define BLDC_PORT1_HALL_RCC_APBPeriph_TIM		RCC_APB1Periph_TIM5
#define BLDC_PORT1_HALL_RCC_APBPeriph_GPIO		RCC_APB2Periph_GPIOA
#define BLDC_PORT1_HALL_GPIO		GPIOA
#define BLDC_PORT1_HALL_GPIO_PIN_A	GPIO_Pin_0
#define BLDC_PORT1_HALL_GPIO_PIN_B	GPIO_Pin_1
#define BLDC_PORT1_HALL_GPIO_PIN_C	GPIO_Pin_2
#define BLDC_PORT1_HALL_TIM_PRESCALER	287
#define BLDC_PORT1_HALL_TIM_PERIOD	65535
#define BLDC_PORT1_HALL_TIM_FLAG		TIM_FLAG_CC1|TIM_FLAG_CC2
#define BLDC_PORT1_HALL_TIM_IT		TIM_IT_CC1|TIM_IT_CC2
#define BLDC_PORT1_HALL_TIM_IRQ		TIM5_IRQn


typedef struct 
{
	TIM_TypeDef *pwm;
	TIM_TypeDef *hall;
	GPIO_TypeDef *hall_gpio;
	uint8_t initialized;
}bldc_handle_t;

#define bldc_get_handle(bldc)	(bldc_handle_t*)(bldc->priv)
#define bldc_set_priv(bldc,priv)	{bldc->priv = priv;}

static bldc_handle_t bldc_handle_t[BLDC_PORT_MAX_NUM]=
{
	{BLDC_PORT0_PWM_TIM,BLDC_PORT0_HALL_TIM,BLDC_PORT0_HALL_GPIO,0},
	{BLDC_PORT1_PWM_TIM,BLDC_PORT1_HALL_TIM,BLDC_PORT1_HALL_GPIO,0},
}

static const hal_bldc_phase_t bldc_hall_phase_anticlockwise_table[]=
{
	//INDEX HA 	HB 	HC
PHASE_ERROR1,	// 0		0    	0   	0   ERR
PHASE_WU,		// 1		1    	0   	0
PHASE_UV,		// 2		0    	1   	0
PHASE_WV,		// 3		1    	1   	0
PHASE_VW,		// 4		0    	0   	1
PHASE_VU,		// 5		1    	0   	1
PHASE_UW,		// 6		0    	1   	1
PHASE_ERROR2,	// 7		1    	1   	1  ERR
};

static const hal_bldc_phase_t bldc_hall_phase_clockwise_table[]=
{
	//INDEX HA 	HB 	HC
PHASE_ERROR1,	// 0		0    	0   	0   ERR
PHASE_UW,		// 1		1    	0   	0
PHASE_VU,		// 2		0    	1   	0
PHASE_VW,		// 3		1    	1   	0
PHASE_WV,		// 4		0    	0   	1
PHASE_UV,		// 5		1    	0   	1
PHASE_WU,		// 6		0    	1   	1
PHASE_ERROR2,	// 7		1    	1   	1  ERR
};


static int32_t bldc_hall_init(bldc_dev_t *bldc)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;                      //定义结构体变量
	TIM_OCInitTypeDef  TIM_OCInitStructure;                     //输出结构体变量定义
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TypeDef *hall_tim;
	GPIO_TypeDef *hall_gpio;
	
	bldc_handle_t *bldc_handle = bldc_get_handle(bldc);
	
	switch(bldc->port)
	{
		case BLDC_PORT0:
			hall_tim = BLDC_PORT0_HALL_TIM;
			hall_gpio = BLDC_PORT0_HALL_GPIO;
			
			//配置
			RCC_APB1PeriphClockCmd(BLDC_PORT0_HALL_RCC_APBPeriph_TIM, ENABLE);
			//启动GPIO
			RCC_APB2PeriphClockCmd(BLDC_PORT0_HALL_RCC_APBPeriph_GPIO, ENABLE);
			
			//GPIO做相应设置，为AF输出	
			GPIO_InitStructure.GPIO_Pin = BLDC_PORT0_HALL_GPIO_PIN_A | BLDC_PORT0_HALL_GPIO_PIN_B | BLDC_PORT0_HALL_GPIO_PIN_C;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(hall_gpio, &GPIO_InitStructure);
			
			TIM_DeInit(hall_tim);

			// 0.25 MHZ  maxTime:262.144ms
			TIM_TimeBaseStructure.TIM_Prescaler = BLDC_PORT0_HALL_TIM_PRESCALER;                                   //TIM基本初始化
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
			TIM_TimeBaseStructure.TIM_Period =BLDC_PORT0_HALL_TIM_PERIOD;
			TIM_TimeBaseStructure.TIM_ClockDivision = 0;
			TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

			TIM_TimeBaseInit(hall_tim,&TIM_TimeBaseStructure);     

			TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;            //选择通道1
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //输入上升沿捕获  
			TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_TRC;  //配置通道为输入，并映射到哪里
			TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;       //输入捕获预分频值
			TIM_ICInitStructure.TIM_ICFilter = 0;                      //输入滤波器带宽设置

			TIM_ICInit(hall_tim, &TIM_ICInitStructure);                     //输入通道配置

			TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;  
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                     //TIM输出通道初始化
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;             
			TIM_OCInitStructure.TIM_Pulse = 2; // ch2 输出比较值
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      
			
			TIM_OC2Init(hall_tim,&TIM_OCInitStructure);
			
			//TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;                     //TIM输出通道初始化
			//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;             
			//TIM_OCInitStructure.TIM_Pulse =65535; 
			//TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      

			//TIM_OC4Init(hall_tim,&TIM_OCInitStructure);


			TIM_SelectHallSensor(hall_tim,ENABLE);                          //使能TIMx的霍尔传感器接口

			TIM_SelectInputTrigger(hall_tim, TIM_TS_TI1F_ED);               //输入触发源选择   

			TIM_SelectSlaveMode(hall_tim, TIM_SlaveMode_Reset);             //从模式选择

			TIM_SelectMasterSlaveMode(hall_tim, TIM_MasterSlaveMode_Enable);//主从模式选择        

			//TIM_SelectOutputTrigger(hall_tim, TIM_TRGOSource_OC2Ref);      //选择输出触发模式(TRGO端)

			// Enable ARR preload 
			TIM_ARRPreloadConfig(hall_tim, ENABLE); 

			NVIC_InitStructure.NVIC_IRQChannel = BLDC_PORT0_HALL_TIM_IRQ;  //嵌套中断通道为TIM1
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级为1
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    //响应优先级为0
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //使能中断
			NVIC_Init(&NVIC_InitStructure);  
			
			TIM_ClearFlag(hall_tim, BLDC_PORT0_HALL_TIM_FLAG);  
			TIM_ITConfig(hall_tim, BLDC_PORT0_HALL_TIM_IT, ENABLE);      //开定时器中断 
			
			TIM_Cmd(hall_tim,DISABLE);
			break;
		case BLDC_PORT1:
			hall_tim = BLDC_PORT1_HALL_TIM;
			hall_gpio = BLDC_PORT1_HALL_GPIO;
			
			//配置
			RCC_APB1PeriphClockCmd(BLDC_PORT1_HALL_RCC_APBPeriph_TIM, ENABLE);
			//启动GPIO
			RCC_APB2PeriphClockCmd(BLDC_PORT1_HALL_RCC_APBPeriph_GPIO, ENABLE);
			
			//GPIO做相应设置，为AF输出	
			GPIO_InitStructure.GPIO_Pin = BLDC_PORT1_HALL_GPIO_PIN_A | BLDC_PORT1_HALL_GPIO_PIN_B | BLDC_PORT1_HALL_GPIO_PIN_C;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(hall_gpio, &GPIO_InitStructure);
			
			TIM_DeInit(hall_tim);

			// 0.25 MHZ  maxTime:262.144ms
			TIM_TimeBaseStructure.TIM_Prescaler = BLDC_PORT0_HALL_TIM_PRESCALER;                                   //TIM基本初始化
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
			TIM_TimeBaseStructure.TIM_Period =BLDC_PORT0_HALL_TIM_PERIOD;
			TIM_TimeBaseStructure.TIM_ClockDivision = 0;
			TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

			TIM_TimeBaseInit(hall_tim,&TIM_TimeBaseStructure);     

			TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;            //选择通道1
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //输入上升沿捕获  
			TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_TRC;  //配置通道为输入，并映射到哪里
			TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;       //输入捕获预分频值
			TIM_ICInitStructure.TIM_ICFilter = 0;                      //输入滤波器带宽设置

			TIM_ICInit(hall_tim, &TIM_ICInitStructure);                     //输入通道配置

			TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;  
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                     //TIM输出通道初始化
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;             
			TIM_OCInitStructure.TIM_Pulse = 2; // ch2 输出比较值
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      
			
			TIM_OC2Init(hall_tim,&TIM_OCInitStructure);
			
			//TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;                     //TIM输出通道初始化
			//TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;             
			//TIM_OCInitStructure.TIM_Pulse =65535; 
			//TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      

			//TIM_OC4Init(bldc_handle->hall,&TIM_OCInitStructure);


			TIM_SelectHallSensor(hall_tim,ENABLE);                          //使能TIMx的霍尔传感器接口

			TIM_SelectInputTrigger(hall_tim, TIM_TS_TI1F_ED);               //输入触发源选择   

			TIM_SelectSlaveMode(hall_tim, TIM_SlaveMode_Reset);             //从模式选择

			TIM_SelectMasterSlaveMode(hall_tim, TIM_MasterSlaveMode_Enable);//主从模式选择        

			//TIM_SelectOutputTrigger(hall_tim, TIM_TRGOSource_OC2Ref);      //选择输出触发模式(TRGO端)

			// Enable ARR preload 
			TIM_ARRPreloadConfig(hall_tim, ENABLE); 

			NVIC_InitStructure.NVIC_IRQChannel = BLDC_PORT1_HALL_TIM_IRQ;  //嵌套中断通道为TIM1
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级为1
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    //响应优先级为0
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //使能中断
			NVIC_Init(&NVIC_InitStructure);  
			
			TIM_ClearFlag(hall_tim, BLDC_PORT1_HALL_TIM_FLAG);  
			TIM_ITConfig(hall_tim, BLDC_PORT1_HALL_TIM_IT, ENABLE);      //开定时器中断 
			
			TIM_Cmd(hall_tim,DISABLE);
			break;
		default:
		
			return -1;
	}
	bldc_handle->hall = hall_tim;
	bldc_handle->hall_gpio = hall_gpio;
	
	return 0;
}

static int32_t hal_read_phase(bldc_dev_t *bldc)
{
	bldc_handle_t *bldc_handle = bldc_get_handle(bldc);
	hal_bldc_phase_t hall_status;

	if(bldc_handle->hall == TIM4){
		hall_status = ((GPIO_ReadInputData(bldc_handle->hall_gpio)&0x1c0) >> 6);
	}else if(bldc_handle->hall == TIM5){
		hall_status = GPIO_ReadInputData(bldc_handle->hall_gpio)&0x07;
	}

	if(bldc->config->rotate == ROTATE_CLOCKWISE){
		bldc->config->phase = bldc_hall_phase_clockwise_table[hall_status];
	}else{
		bldc->config->phase = bldc_hall_phase_anticlockwise_table[hall_status];
	}
	
	return 0;
}


static int32_t bldc_pwm_init(bldc_dev_t *bldc)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;                      //定义结构体变量
	TIM_OCInitTypeDef  TIM_OCInitStructure;                     //输出结构体变量定义
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TypeDef *pwm_tim;
	GPIO_TypeDef *pwm_gpio;
	
	bldc_handle_t *bldc_handle = bldc_get_handle(bldc);
	
	switch(bldc->port)
	{
		case BLDC_PORT0:
			pwm_tim = BLDC_PORT0_PWM_TIM;
			
			
			//启动TIM8
			//开启TIM和相应端口时钟
			//启动TIM8
			RCC_APB2PeriphClockCmd(BLDC_PORT0_PWM_RCC_APBPeriph_TIM, ENABLE);
			//启动GPIO
			RCC_APB2PeriphClockCmd(BLDC_PORT0_PWM_RCC_APBPeriph_GPIO , ENABLE);
				
			//GPIO做相应设置，为AF输出								
			/* TIM8无法重映射，使用IO具有唯一性
			PC6 ---OC1
			PC7 ---OC2
			PC8 ---OC3

			PA7 ---OC1N
			PB0 ---OC2N
			PB1 ---OC3N

			PA6 --- TIM8_BKIN
			*/
			GPIO_InitStructure.GPIO_Pin =  BLDC_PORT0_PWM_GPIO_PIN_OC1;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(BLDC_PORT0_PWM_GPIO_OC1, &GPIO_InitStructure);

			GPIO_InitStructure.GPIO_Pin =  BLDC_PORT0_PWM_GPIO_PIN_OC1N;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(BLDC_PORT0_PWM_GPIO_OC1N, &GPIO_InitStructure);

			GPIO_InitStructure.GPIO_Pin =  BLDC_PORT0_PWM_GPIO_PIN_OC2;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(BLDC_PORT0_PWM_GPIO_OC2, &GPIO_InitStructure);

			GPIO_InitStructure.GPIO_Pin =  BLDC_PORT0_PWM_GPIO_PIN_OC2N;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(BLDC_PORT0_PWM_GPIO_OC2N, &GPIO_InitStructure);

			GPIO_InitStructure.GPIO_Pin =  BLDC_PORT0_PWM_GPIO_PIN_OC3;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(BLDC_PORT0_PWM_GPIO_OC3, &GPIO_InitStructure);

			GPIO_InitStructure.GPIO_Pin =  BLDC_PORT0_PWM_GPIO_PIN_OC3N;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(BLDC_PORT0_PWM_GPIO_OC3N, &GPIO_InitStructure);

			GPIO_InitStructure.GPIO_Pin =  BLDC_PORT0_PWM_GPIO_PIN_BKIN;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(BLDC_PORT0_PWM_GPIO_BKIN, &GPIO_InitStructure);

			
			//Step3. TIM模块初始化
			//TIM1基本计数器设置（设置PWM频率）
			//频率=TIM1_CLK/ARR/2
			TIM_TimeBaseStructure.TIM_Period = CALC_PWM_FREQ_CENTER_ALIGNED(MCU_PCLK2,BLDC_PORT0_PWM_FREQ);	   //32000HZ
			TIM_TimeBaseStructure.TIM_Prescaler = 0;  //72MHZ
			TIM_TimeBaseStructure.TIM_ClockDivision = 0;
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;
			TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
			TIM_TimeBaseInit(pwm_tim, &TIM_TimeBaseStructure);

			
			//TIM8_OC1模块设置（设置1通道占空比）下管
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;        //设置为pwm1输出模式,高电平脉宽以载波峰值为中心
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
			TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
			TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;             //死区后输出状态
			TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;		  //死区后互补端输出状态	TIM_OCInitStructure.TIM_Pulse = 1500;
			TIM_OCInitStructure.TIM_Pulse = 0;
			TIM_OC1Init(pwm_tim, &TIM_OCInitStructure);

			//启用CCR1寄存器的影子寄存器（直到产生更新事件才更改设置）  
			TIM_OC1PreloadConfig(pwm_tim, TIM_OCPreload_Enable);
			
			//TIM1_OC2模块设置（设置1通道占空比）
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                      //设置为pwm1输出模式,高电平脉宽以载波波谷为中心
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
			TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
			TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;             //死区后输出状态
			TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;		  //死区后互补端输出状态	TIM_OCInitStructure.TIM_Pulse = 1500;
			TIM_OCInitStructure.TIM_Pulse = 0;
			TIM_OC2Init(pwm_tim, &TIM_OCInitStructure);

			//启用CCR2寄存器的影子寄存器（直到产生更新事件才更改设置）
			TIM_OC2PreloadConfig(pwm_tim, TIM_OCPreload_Enable);

			//TIM1_OC3模块设置（设置1通道占空比）
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                      //设置为pwm1输出模式,高电平脉宽以载波波谷为中心
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
			TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
			TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;             //死区后输出状态
			TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;		  //死区后互补端输出状态	TIM_OCInitStructure.TIM_Pulse = 1500;
			TIM_OCInitStructure.TIM_Pulse = 0;
			TIM_OC3Init(pwm_tim, &TIM_OCInitStructure);

			//启用CCR2寄存器的影子寄存器（直到产生更新事件才更改设置）
			TIM_OC3PreloadConfig(pwm_tim, TIM_OCPreload_Enable);
			
			//死区设置
			TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
			TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
			TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
			TIM_BDTRInitStructure.TIM_DeadTime = BLDC_PORT0_PWM_DEADTIME; //这里调整死区大小0-0xff	    死区1.16us
			TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;		//TIM_Break_Disable
			TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;		//	TIM_BreakPolarity_High
			TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
			TIM_BDTRConfig(pwm_tim, &TIM_BDTRInitStructure);


			TIM_CCPreloadControl(pwm_tim, ENABLE);
			TIM_SelectCOM(pwm_tim, ENABLE );  
			
			//TIM_SelectInputTrigger(pwm_tim, TIM_TS_ITR2); 
			//TIM_SelectSlaveMode(pwm_tim,TIM_SlaveMode_Trigger);


			//TIM_SelectSlaveMode(pwm_tim, TIM_SlaveMode_Reset);             //从模式选择

			//TIM_SelectMasterSlaveMode(pwm_tim, TIM_MasterSlaveMode_Enable);//主从模式选择        

			NVIC_InitStructure.NVIC_IRQChannel = BLDC_PORT0_PWM_TIM_IRQ;  //嵌套中断通道为TIM1
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级为1
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    //响应优先级为0
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //使能中断
			NVIC_Init(&NVIC_InitStructure);  
			//中断源
			TIM_ClearFlag(pwm_tim, BLDC_PORT0_PWM_TIM_FLAG);			 
			TIM_ITConfig(pwm_tim, BLDC_PORT0_PWM_TIM_IT, DISABLE);

			//启用ARR的影子寄存器（直到产生更新事件才更改设置）
			TIM_ARRPreloadConfig(pwm_tim, ENABLE);

			
			
			//TIM8开启
			TIM_Cmd(pwm_tim, ENABLE);
			//TIM_Cmd(pwm_tim, DISABLE);
			//TIM8_OC通道输出PWM（一定要加）
			TIM_CtrlPWMOutputs(pwm_tim, ENABLE);
			TIM_CCxCmd(pwm_tim, TIM_Channel_1, TIM_CCx_Disable);
			TIM_CCxNCmd(pwm_tim, TIM_Channel_1, TIM_CCxN_Disable);
			TIM_CCxCmd(pwm_tim, TIM_Channel_2, TIM_CCx_Disable);
			TIM_CCxNCmd(pwm_tim, TIM_Channel_2, TIM_CCxN_Disable);
			TIM_CCxCmd(pwm_tim, TIM_Channel_3, TIM_CCx_Disable);
			TIM_CCxNCmd(pwm_tim, TIM_Channel_3, TIM_CCxN_Disable);

			TIM_SetCompare1(pwm_tim, 0);
			TIM_SetCompare2(pwm_tim, 0);
			TIM_SetCompare3(pwm_tim, 0);
			
			break;
		case BLDC_PORT1:
			pwm_tim = BLDC_PORT1_PWM_TIM;
			
			//启动TIM1
			RCC_APB2PeriphClockCmd(BLDC_PORT1_PWM_RCC_APBPeriph_TIM, ENABLE);

			//开启TIM和相应端口时钟
			//启动GPIO	启动AFIO	启动TIM1	
			RCC_APB2PeriphClockCmd(BLDC_PORT1_PWM_RCC_APBPeriph_GPIO, ENABLE);

				
			//配置inv pwm
			GPIO_PinRemapConfig(BLDC_PORT1_PWM_GPIO_FullRemap_TIM, ENABLE);

			//GPIO做相应设置，为AF输出								
			/*
			PE8 ---OC1
			PE9 ---OC1N
			PE10 ---OC2
			PE11 ---OC2N
			PE12 ---OC3
			PE13 ---OC3N

			PE15 --- TIM1_BKIN
			*/
			GPIO_InitStructure.GPIO_Pin =  BLDC_PORT1_PWM_GPIO_PIN_OC1;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(BLDC_PORT1_PWM_GPIO_OC1, &GPIO_InitStructure);

			GPIO_InitStructure.GPIO_Pin =  BLDC_PORT1_PWM_GPIO_PIN_OC1N;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(BLDC_PORT1_PWM_GPIO_OC1N, &GPIO_InitStructure);

			GPIO_InitStructure.GPIO_Pin =  BLDC_PORT1_PWM_GPIO_PIN_OC2;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(BLDC_PORT1_PWM_GPIO_OC2, &GPIO_InitStructure);

			GPIO_InitStructure.GPIO_Pin =  BLDC_PORT1_PWM_GPIO_PIN_OC2N;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(BLDC_PORT1_PWM_GPIO_OC2N, &GPIO_InitStructure);

			GPIO_InitStructure.GPIO_Pin =  BLDC_PORT1_PWM_GPIO_PIN_OC3;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(BLDC_PORT1_PWM_GPIO_OC3, &GPIO_InitStructure);

			GPIO_InitStructure.GPIO_Pin =  BLDC_PORT1_PWM_GPIO_PIN_OC3N;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(BLDC_PORT1_PWM_GPIO_OC3N, &GPIO_InitStructure);

			GPIO_InitStructure.GPIO_Pin =  BLDC_PORT1_PWM_GPIO_PIN_BKIN;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(BLDC_PORT1_PWM_GPIO_BKIN, &GPIO_InitStructure);
			
			//Step3. TIM模块初始化
			//TIM1基本计数器设置（设置PWM频率）
			//频率=TIM1_CLK/ARR/2
			TIM_TimeBaseStructure.TIM_Period = CALC_PWM_FREQ_CENTER_ALIGNED(MCU_PCLK2,BLDC_PORT1_PWM_FREQ);	   //32000HZ
			TIM_TimeBaseStructure.TIM_Prescaler = 0;  //72MHZ
			TIM_TimeBaseStructure.TIM_ClockDivision = 0;
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;
			//TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
			TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
			TIM_TimeBaseInit(pwm_tim, &TIM_TimeBaseStructure);

			//TIM1_OC1模块设置（设置1通道占空比）
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                      //设置为pwm1输出模式,高电平脉宽以载波波谷为中心
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
			TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
			TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;             //死区后输出状态
			TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;		  //死区后互补端输出状态	TIM_OCInitStructure.TIM_Pulse = 1500;
			TIM_OCInitStructure.TIM_Pulse = 0;
			TIM_OC1Init(pwm_tim, &TIM_OCInitStructure);

			//启用CCR1寄存器的影子寄存器（直到产生更新事件才更改设置）  
			TIM_OC1PreloadConfig(pwm_tim, TIM_OCPreload_Enable);
			
			//TIM1_OC2模块设置（设置1通道占空比）
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                      //设置为pwm1输出模式,高电平脉宽以载波波谷为中心
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
			TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
			TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;             //死区后输出状态
			TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;		  //死区后互补端输出状态	TIM_OCInitStructure.TIM_Pulse = 1500;
			TIM_OCInitStructure.TIM_Pulse = 0;
			TIM_OC2Init(pwm_tim, &TIM_OCInitStructure);

			//启用CCR2寄存器的影子寄存器（直到产生更新事件才更改设置）
			TIM_OC2PreloadConfig(pwm_tim, TIM_OCPreload_Enable);


			//TIM1_OC3模块设置（设置1通道占空比）
			TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                      //设置为pwm1输出模式,高电平脉宽以载波波谷为中心
			TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
			TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
			TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
			TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
			TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;             //死区后输出状态
			TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;		  //死区后互补端输出状态	TIM_OCInitStructure.TIM_Pulse = 1500;
			TIM_OCInitStructure.TIM_Pulse = 0;
			TIM_OC3Init(pwm_tim, &TIM_OCInitStructure);

			//启用CCR2寄存器的影子寄存器（直到产生更新事件才更改设置）
			TIM_OC3PreloadConfig(pwm_tim, TIM_OCPreload_Enable);
			
			//死区设置
			TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
			TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
			TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
			TIM_BDTRInitStructure.TIM_DeadTime = BLDC_PORT1_PWM_DEADTIME; //这里调整死区大小0-0xff	    死区1.16us
			TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;		//TIM_Break_Disable
			TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;		//	TIM_BreakPolarity_High
			TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
			TIM_BDTRConfig(pwm_tim, &TIM_BDTRInitStructure);

			NVIC_InitStructure.NVIC_IRQChannel = BLDC_PORT1_PWM_TIM_IRQ;  //嵌套中断通道为TIM1
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级为1
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    //响应优先级为0
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //使能中断
			NVIC_Init(&NVIC_InitStructure);  

			//中断源
			TIM_ClearFlag(pwm_tim, BLDC_PORT1_PWM_TIM_FLAG);	 
			//TIM_ITConfig(pwm_tim, BLDC_PORT1_PWM_TIM_IT, DISABLE);
			TIM_ITConfig(pwm_tim, BLDC_PORT1_PWM_TIM_IT, ENABLE);

			//启用ARR的影子寄存器（直到产生更新事件才更改设置）
			TIM_ARRPreloadConfig(pwm_tim, ENABLE);

			//TIM1开启
			TIM_Cmd(pwm_tim, ENABLE);
			//TIM1_OC通道输出PWM（一定要加）
			TIM_CtrlPWMOutputs(pwm_tim, ENABLE);
			TIM_CCxCmd(pwm_tim, TIM_Channel_1, TIM_CCx_Disable);
			TIM_CCxNCmd(pwm_tim, TIM_Channel_1, TIM_CCxN_Disable);
			TIM_CCxCmd(pwm_tim, TIM_Channel_2, TIM_CCx_Disable);
			TIM_CCxNCmd(pwm_tim, TIM_Channel_2, TIM_CCxN_Disable);
			TIM_CCxCmd(pwm_tim, TIM_Channel_3, TIM_CCx_Disable);
			TIM_CCxNCmd(pwm_tim, TIM_Channel_3, TIM_CCxN_Disable);

			TIM_SetCompare1(pwm_tim, 0);
			TIM_SetCompare2(pwm_tim, 0);
			TIM_SetCompare3(pwm_tim, 0);
			
			break;
		default:
		
			return -1;
	}
	bldc_handle->pwm = pwm_tim;
	
	return 0;
}

int32_t  hal_bldc_init(bldc_dev_t *bldc)
{
	bldc_config_t *config = bldc->config;
	uint8_t port = bldc->port;
	bldc_handle_t *bldc_handle;
	
	if(port >= BLDC_PORT_MAX_NUM){
		return -1;
	}
	bldc->config->mode = MODE_STOP;
	
	bldc_handle = &stm32_bldc_handle[port];
	bldc_handle->initialized = 1;
	
	bldc_set_priv(bldc,bldc_handle);
	
	bldc_hall_init(bldc);
	hal_read_phase(bldc);
	bldc_pwm_init(bldc);
	
	
	return 0;
}
int32_t  hal_bldc_finalize(bldc_dev_t *bldc)
{
	bldc_handle_t *bldc_handle = bldc_get_handle(bldc);                                         
	
	if(bldc->port >= BLDC_PORT_MAX_NUM){
		return -1;
	}
	
	if(bldc_handle->initialized){
		bldc_handle->initialized = 0;
		bldc->config->mode = MODE_STOP;
		TIM_Cmd(bldc_handle->pwm,DISABLE);
		TIM_Cmd(bldc_handle->hall,DISABLE);
	}
	return 0;
}
int32_t  hal_bldc_start(bldc_dev_t *bldc)
{
	bldc_handle_t *bldc_handle = bldc_get_handle(bldc);
	
	if(bldc->port >= BLDC_PORT_MAX_NUM){
		return -1;
	}
	hal_read_phase(bldc);
	TIM_Cmd(bldc_handle->pwm,ENABLE);
	TIM_Cmd(bldc_handle->hall,ENABLE);
	bldc->config->mode = MODE_START;
	
	return 0;
}
int32_t  hal_bldc_stop(bldc_dev_t *bldc)
{
	bldc_handle_t *bldc_handle = bldc_get_handle(bldc);
	
	if(bldc->port >= BLDC_PORT_MAX_NUM){
		return -1;
	}
	bldc->config->mode = MODE_STOP;
	TIM_Cmd(bldc_handle->pwm,DISABLE);
	TIM_Cmd(bldc_handle->hall,DISABLE);
	return 0;
}

int32_t  hal_bldc_update_pwm(bldc_dev_t *bldc)
{
	uint16_t duty = bldc->config->duty_cycle;
	bldc_handle_t *bldc_handle = bldc_get_handle(bldc);
	
	if(bldc->config->mode == MODE_STOP){
		return -1;
	}
	
	TIM_SetCompare1(bldc_handle->pwm, duty);
	TIM_SetCompare2(bldc_handle->pwm, duty);
	TIM_SetCompare3(bldc_handle->pwm, duty);
	return 0;
}
int32_t  hal_bldc_update_phase(bldc_dev_t *bldc)
{
	bldc_handle_t *bldc_handle = bldc_get_handle(bldc);
	
	if(bldc->config->mode == MODE_STOP){
		return -1;
	}
	
	hal_read_phase(bldc);
	
	switch(bldc->config->phase)
	{
		case PHASE_UV:
			TIM_SelectOCxM(bldc_handle->pwm, TIM_Channel_1, TIM_OCMode_PWM1);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_1, TIM_CCx_Enable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_1, TIM_CCxN_Enable);

			TIM_ForcedOC2Config(bldc_handle->pwm, TIM_ForcedAction_Active);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_2, TIM_CCx_Disable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_2, TIM_CCxN_Enable);

			TIM_SelectOCxM(bldc_handle->pwm, TIM_Channel_3, TIM_OCMode_PWM1);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_3, TIM_CCx_Disable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_3, TIM_CCxN_Disable);
			break;
		case PHASE_UW:
			TIM_SelectOCxM(bldc_handle->pwm, TIM_Channel_1, TIM_OCMode_PWM1);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_1, TIM_CCx_Enable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_1, TIM_CCxN_Enable);

			TIM_SelectOCxM(bldc_handle->pwm, TIM_Channel_2, TIM_OCMode_PWM1);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_2, TIM_CCx_Disable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_2, TIM_CCxN_Disable);

			TIM_ForcedOC3Config(bldc_handle->pwm, TIM_ForcedAction_Active);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_3, TIM_CCx_Disable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_3, TIM_CCxN_Enable);
			break;
		case PHASE_VW:
			TIM_SelectOCxM(bldc_handle->pwm, TIM_Channel_1, TIM_OCMode_PWM1);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_1, TIM_CCx_Disable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_1, TIM_CCxN_Disable);

			TIM_SelectOCxM(bldc_handle->pwm, TIM_Channel_2, TIM_OCMode_PWM1);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_2, TIM_CCx_Enable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_2, TIM_CCxN_Enable);

			TIM_ForcedOC3Config(bldc_handle->pwm, TIM_ForcedAction_Active);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_3, TIM_CCx_Disable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_3, TIM_CCxN_Enable);
			break;
		case PHASE_VU:
			TIM_ForcedOC1Config(bldc_handle->pwm, TIM_ForcedAction_Active);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_1, TIM_CCx_Disable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_1, TIM_CCxN_Enable);

			TIM_SelectOCxM(bldc_handle->pwm, TIM_Channel_2, TIM_OCMode_PWM1);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_2, TIM_CCx_Enable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_2, TIM_CCxN_Enable);

			TIM_SelectOCxM(bldc_handle->pwm, TIM_Channel_3, TIM_OCMode_PWM1);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_3, TIM_CCx_Disable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_3, TIM_CCxN_Disable);
			break;
		case PHASE_WU:
			TIM_ForcedOC1Config(bldc_handle->pwm, TIM_ForcedAction_Active);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_1, TIM_CCx_Disable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_1, TIM_CCxN_Enable);

			TIM_SelectOCxM(bldc_handle->pwm, TIM_Channel_2, TIM_OCMode_PWM1);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_2, TIM_CCx_Disable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_2, TIM_CCxN_Disable);

			TIM_SelectOCxM(bldc_handle->pwm, TIM_Channel_3, TIM_OCMode_PWM1);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_3, TIM_CCx_Enable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_3, TIM_CCxN_Enable);
			break;
		case PHASE_WV:
			TIM_SelectOCxM(bldc_handle->pwm, TIM_Channel_1, TIM_OCMode_PWM1);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_1, TIM_CCx_Disable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_1, TIM_CCxN_Disable);

			TIM_ForcedOC2Config(bldc_handle->pwm, TIM_ForcedAction_Active);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_2, TIM_CCx_Disable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_2, TIM_CCxN_Enable);

			TIM_SelectOCxM(bldc_handle->pwm, TIM_Channel_3, TIM_OCMode_PWM1);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_3, TIM_CCx_Enable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_3, TIM_CCxN_Enable);
			break;
		default:
			TIM_ForcedOC1Config(bldc_handle->pwm, TIM_ForcedAction_Active);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_1, TIM_CCx_Disable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_1, TIM_CCxN_Enable);

			TIM_ForcedOC2Config(bldc_handle->pwm, TIM_ForcedAction_Active);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_2, TIM_CCx_Disable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_2, TIM_CCxN_Enable);

			TIM_ForcedOC3Config(bldc_handle->pwm, TIM_ForcedAction_Active);
			TIM_CCxCmd(bldc_handle->pwm, TIM_Channel_3, TIM_CCx_Disable);
			TIM_CCxNCmd(bldc_handle->pwm, TIM_Channel_3, TIM_CCxN_Enable);
			break;
	}
	// 产生从定时器COM事件，更新3对互补输出控制
	TIM_GenerateEvent(bldc_handle->pwm, TIM_EventSource_COM);
	
	return 0;
}
