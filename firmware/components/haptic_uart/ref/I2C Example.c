/*
本demo使用GPIO模拟软件IIC方式，已实现对应接口

关于接口说明：
1. 需要修改使用的GPIO引脚
2. 需要自行实现对应延时函数： I2C_DELAY, 对应i2c频率，最大不超过200Khz
3. 已实现GD和ST平台的硬件相关宏函数，如果是这两个平台，USE_CHIP设置为GD32或STM32即可
   其他平台需要自行实现对应宏：
		SDA_MODE_OUTPUT: SDA设置未输出模式
		SDA_MODE_INPUT：	SDA设置未输入模式
		SDA_LOW：		SDA输出低电平
		SDA_HIGH：		SDA输出高电平
		SDA_GET：		获取SDA电平
		SCL_LOW：		SCL输出高电平
		SCL_HIGH：		SCL输出高电平
*/

#define GD32        0
#define STM32       1

/* 选择对应平台: 0-GD32, 1-STM32, 3-其他 */
#define USE_CHIP    STM32

/* GPIO引脚, 要改成自己对应的引脚号*/
#define I2C_SCL_PORT        GPIOE
#define I2C_SCL_PIN         GPIO_PIN_0
#define I2C_SCL_POS         0			/* gpio_pin_0, so pos is 0*/

#define I2C_SDA_PORT        GPIOE
#define I2C_SDA_PIN         GPIO_PIN_1
#define I2C_SDA_POS         1			/* gpio_pin_1, so pos is 1*/

/* delay函数，需要实现自己的延时函数，最大sclk频率不超过200Khz*/
#define I2C_DELAY()         __NOP()  	//max scl clock: 200KHz 

#if (USE_CHIP == GD32)
#define SDA_MODE_OUTPUT()   {GPIO_CTL(I2C_SDA_PORT)  &= (~(0x03 << (2*I2C_SDA_POS)));\
                             GPIO_CTL(I2C_SDA_PORT)  |= (0x01 << (2*I2C_SDA_POS));}
#define SDA_MODE_INPUT()    {GPIO_CTL(I2C_SDA_PORT)  &= (~(0x03 << (2*I2C_SDA_POS)));}
#define SDA_HIGH()          {GPIO_BOP(I2C_SDA_PORT) = (uint32_t)I2C_SDA_PIN;}
#define SDA_LOW()           {GPIO_BC(I2C_SDA_PORT) = (uint32_t)I2C_SDA_PIN;}
#define SDA_GET()           ((uint32_t)RESET != (GPIO_ISTAT(I2C_SDA_PORT) & (I2C_SDA_PIN)))

#define SCL_HIGH()          {GPIO_BOP(I2C_SCL_PORT) = (uint32_t)I2C_SCL_PIN;}
#define SCL_LOW()           {GPIO_BC(I2C_SCL_PORT) = (uint32_t)I2C_SCL_PIN;}

#elif (USE_CHIP == STM32)
#define SDA_MODE_OUTPUT()   {I2C_SDA_PORT->MODER  &= (~(0x03 << (2 * I2C_SDA_POS)));\
                             I2C_SDA_PORT->MODER  |= (0x01 << (2 * I2C_SDA_POS));}
#define SDA_MODE_INPUT()    {I2C_SDA_PORT->MODER  &= (~(0x03 << (2*I2C_SDA_POS)));}
#define SDA_LOW()           (I2C_SDA_PORT->BSRR = ((uint32_t)I2C_SDA_PIN) << 16)
#define SDA_HIGH()          (I2C_SDA_PORT->BSRR = ((uint32_t)I2C_SDA_PIN))
#define SDA_GET()           ((uint32_t)GPIO_PIN_RESET != (I2C_SDA_PORT->IDR & I2C_SDA_PIN))

#define SCL_LOW()           (I2C_SCL_PORT->BSRR = ((uint32_t)I2C_SCL_PIN) << 16)
#define SCL_HIGH()          (I2C_SCL_PORT->BSRR = ((uint32_t)I2C_SCL_PIN))

#else

/* other platforms:  todo */
#define SDA_MODE_OUTPUT()    (0U)
#define SDA_MODE_INPUT()     (0U)
#define SDA_LOW()            (0U)
#define SDA_HIGH()           (0U)
#define SDA_GET()            (0U)

#define SCL_LOW()        
#define SCL_HIGH()       

#endif

static void i2c_start(void);
static void i2c_stop(void);
static uint8_t i2c_read_byre(uint8_t nack);
static uint8_t i2c_send_byte(uint8_t byte);
static uint8_t i2c_sendaddress(uint8_t addr, uint8_t rw);
static uint8_t i2c_read_byre(uint8_t nack);
static uint8_t i2c_send_byte(uint8_t byte);
static uint8_t lrc_cal(const uint8_t* ptr, uint32_t len);

int16_t i2c_soft_write(uint8_t *buff, int16_t len)
{
    int16_t ret = 0;
    uint8_t byte = 0;
    
    /* start */
    i2c_start();

    /* send addr/write + data */
    for(int16_t i=0; i < len; i++)
    {
        byte = buff[i];
        if (0 != i2c_send_byte(byte))
        {
            /* nack */
            i2c_stop();
            ret = (-1);
            break;
        }
    }
    /* stop */
    i2c_stop();
    
    return ret;
}

int16_t i2c_soft_read(uint8_t addr, uint8_t *buff, int16_t len)
{
    int16_t ret = 0;
    
    /* start */
    i2c_start();
    /* send address + read */
    if (0 != i2c_sendaddress(addr, 1))
    {
        ret = (-1);
    }
    /* read buff */
    for(int16_t i=0; i<len; i++)
    {
        buff[i] = i2c_read_byre(i == (len-1));
    }
    /* stop */
    i2c_stop();
    
    return ret;
}



void i2c_test(void)
{
    uint8_t addr = 0x03;
	
	/* read version */
    uint8_t trs_buf[9] = {0};
    uint8_t rev_buf[100] = {0};
	
	trs_buf[0] = (addr << 1) | 0;
	trs_buf[1] = 0xFB;
	trs_buf[2] = 0x84;
	trs_buf[3] = 0x17;
	trs_buf[4] = 0x00;
	trs_buf[5] = 0x00;
	trs_buf[6] = 0x50;
	trs_buf[7] = 0x00;
	trs_buf[8] = lrc_cal(trs_buf, sizeof(trs_buf)-1);
	
    i2c_soft_write(trs_buf, sizeof(trs_buf));
	/* at least 25us */
    delay_us(25);
	
    i2c_soft_read(addr, rev_buf, 80);
}

static uint8_t lrc_cal(const uint8_t* ptr, uint32_t len)
{
    uint8_t end;
    uint32_t i;

    end = 0;
    for(i = 0; i < len; i++)
    {
        end += ptr[i];
    }
    end = ~end + 1;
	
    return end;
}

static void i2c_start(void)
{
    SDA_MODE_OUTPUT();
    SDA_HIGH();
    SCL_HIGH();
    I2C_DELAY();
    SDA_LOW();
    I2C_DELAY();
    SCL_LOW();
}

static void i2c_stop(void)
{
    SDA_MODE_OUTPUT();
    I2C_DELAY();
    SDA_LOW();
    SCL_HIGH();
    I2C_DELAY();
    SDA_HIGH();
}

static uint8_t i2c_send_byte(uint8_t byte)
{
    uint8_t ack;
    
    SDA_MODE_OUTPUT();
    for(uint8_t b=0; b<8; b++)
    {
        I2C_DELAY();
        SCL_LOW();
        if(((byte >> (7-b)) & 0x01))
        {
            SDA_HIGH();
        }
        else
        {
            SDA_LOW();
        }
        I2C_DELAY();
        SCL_HIGH();
    }
    /* read ack */
    I2C_DELAY();
    SCL_LOW();
    SDA_MODE_INPUT();
    I2C_DELAY();
    SCL_HIGH();
    I2C_DELAY();
    ack = SDA_GET();
    SCL_LOW();
    return ack;
}

static uint8_t i2c_read_byre(uint8_t nack)
{
    uint8_t data = 0;
    
    SDA_MODE_INPUT();
    I2C_DELAY();
    /* read byte */
    for(uint8_t i=0; i<8; i++)
    {
        SCL_HIGH();
        I2C_DELAY();
        data <<=1;
        if(SDA_GET())
        {
            data |= 0x01;
        }
        SCL_LOW();
        I2C_DELAY();
    }
    
    /* send ack/nack */
    SDA_MODE_OUTPUT();
    if(nack)
    {
        SDA_HIGH();
    }
    else
    {
        SDA_LOW();
    }
    I2C_DELAY();
    SCL_HIGH();
    I2C_DELAY();
    SCL_LOW();
    
    return data;
}

static uint8_t i2c_sendaddress(uint8_t addr, uint8_t rw)
{
    return i2c_send_byte((addr << 1) | rw);
}
