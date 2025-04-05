#include "ultrasonic.h"
#include "stm32f4xx_hal.h"  // 根据实际使用的STM32型号修改

// 定义静态变量
static TIM_HandleTypeDef htim2;  // 用于精确延时的定时器
static uint32_t last_measurement_time = 0;
static int last_distance = 0;

/**
 * 初始化超声波模块
 * 配置GPIO和定时器
 */
void Ultrasonic_Init(void)
{
    // 配置GPIO
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能GPIO时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // 配置触发引脚为输出
    GPIO_InitStruct.Pin = TRIG_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TRIG_PORT, &GPIO_InitStruct);
    
    // 配置回响引脚为输入
    GPIO_InitStruct.Pin = ECHO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(ECHO_PORT, &GPIO_InitStruct);
    
    // 初始化定时器（用于精确延时）
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 84-1;  // 84MHz时钟，分频后为1MHz
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);
    
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);
    
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
    
    // 启动定时器
    HAL_TIM_Base_Start(&htim2);
    
    // 初始化完成后，确保触发引脚为低电平
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
}

/**
 * 触发测量
 * 发送10us的高电平脉冲
 */
void Ultrasonic_TriggerMeasurement(void)
{
    // 发送触发脉冲
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    delay_us(10);  // 延时10us
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
}

/**
 * 等待并测量回响时间
 * @return 回响时间（微秒），如果超时返回0
 */
uint32_t Ultrasonic_WaitEcho(void)
{
    uint32_t start_time, echo_time;
    
    // 等待回响信号变高（开始）
    start_time = HAL_GetTick();
    while(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET)
    {
        if(HAL_GetTick() - start_time > 10)  // 10ms超时
            return 0;
    }
    
    // 开始计时
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    
    // 等待回响信号变低（结束）
    while(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
    {
        if(__HAL_TIM_GET_COUNTER(&htim2) > TIMEOUT_US)
            return 0;
    }
    
    // 获取计时结果
    echo_time = __HAL_TIM_GET_COUNTER(&htim2);
    
    return echo_time;
}

/**
 * 读取距离
 * @return 距离（毫米），如果测量失败返回上次的有效值
 */
int Read_Ultrasonic(void)
{
    uint32_t echo_time;
    int distance;
    uint32_t current_time = HAL_GetTick();
    
    // 限制测量频率，至少间隔60ms
    if(current_time - last_measurement_time < 60)
    {
        return last_distance;
    }
    
    // 触发新的测量
    Ultrasonic_TriggerMeasurement();
    
    // 等待并获取回响时间
    echo_time = Ultrasonic_WaitEcho();
    
    // 如果测量超时，返回上次的有效值
    if(echo_time == 0)
    {
        return last_distance;
    }
    
    // 计算距离（毫米）
    distance = (int)(echo_time * US_TO_MM);
    
    // 检查是否在有效范围内
    if(distance < MIN_DISTANCE || distance > MAX_DISTANCE)
    {
        return last_distance;
    }
    
    // 更新状态
    last_measurement_time = current_time;
    last_distance = distance;
    
    return distance;
}

/**
 * 微秒级延时函数
 * @param us 延时时间（微秒）
 */
static void delay_us(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while(__HAL_TIM_GET_COUNTER(&htim2) < us);
} 