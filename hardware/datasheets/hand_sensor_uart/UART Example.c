#define FIN_PCL_UART_HEAD_LEN				13						//通信协议消耗字节数
#define TX_RX_BUFFER_SIZE					4096					//数据发送接收长度
#define SLAVE_DEVICE_ID						0x01					//0x01~0x06
#define UART_HANDLE							huart1					//需适配MCU
#define UART_DMA_RX_HANDLE					hdma_usart1_rx			//需适配MCU
#define CMD_REV_HEAD                        "\xAA\x55"				//接收帧头
#define CMD_REV_HEAD_LEN                    2						//0xAA 0x55 共2个Byte
#define	OS_DELAY(val)						HAL_Delay(val)			//延时函数

static uint8_t 	s_uart_tx_dma_buffer[TX_RX_BUFFER_SIZE];		/*需放在DMA可访问RAM区*/
static uint8_t 	s_uart_rx_dma_buffer[TX_RX_BUFFER_SIZE];		/*需放在DMA可访问RAM区*/
static uint8_t 	s_fm_tx_buff[TX_RX_BUFFER_SIZE];
static uint8_t 	s_fm_rx_buff[TX_RX_BUFFER_SIZE];
static bool		g_uart_received = false;


/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
***********************************需MCU 接口适配函数区**********************************
****************************此区域部分函数，直接调用芯片厂商库函数操作外设********************
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void uart_receive_proc(void)
{
    uint16_t number = __HAL_DMA_GET_COUNTER(&UART_DMA_RX_HANDLE);
    uint16_t rev_num = sizeof(s_uart_rx_dma_buffer) - number;

    if(rev_num >= FIN_PCL_UART_HEAD_LEN && rev_num < TX_RX_BUFFER_SIZE)
    {
        HAL_UART_AbortReceive(&UART_HANDLE);
		
        uint16_t cal_crc_len = CMD_REV_HEAD_LEN + 2 + (s_uart_rx_dma_buffer[2] + (s_uart_rx_dma_buffer[3] << 8));
        
		/*  2Byte 帧头 + 2Byte 接收长度 + 1Byte 设备地址 + 1Byte 预留 + 1Byte (R+功能码) + 4Byte起始地址 +  2Byte据长度 +nByte传感器数据 + 1Byte校验*/
         if(g_uart_received == false &&
			0 == memcmp(s_uart_rx_dma_buffer, CMD_REV_HEAD, CMD_REV_HEAD_LEN) && 
             (s_uart_rx_dma_buffer[rev_num - 1] == lrc_cal(s_uart_rx_dma_buffer, cal_crc_len)))
        {
            memcpy(s_fm_rx_buff, s_uart_rx_dma_buffer, rev_num);
            g_uart_received = true;
        }
		
        HAL_UART_Receive_DMA(&UART_HANDLE, s_uart_rx_dma_buffer, sizeof(s_uart_rx_dma_buffer));
        __HAL_UART_ENABLE_IT(&UART_HANDLE, UART_IT_IDLE);
    }
	else
	{
		g_uart_received = false;
		HAL_UART_AbortReceive(&UART_HANDLE);
		HAL_UART_Receive_DMA(&UART_HANDLE, s_uart_rx_dma_buffer, sizeof(s_uart_rx_dma_buffer));
        __HAL_UART_ENABLE_IT(&UART_HANDLE, UART_IT_IDLE);
	}

}

void USART1_IRQHandler(void)
{
    if(__HAL_UART_GET_FLAG(&UART_HANDLE, UART_FLAG_IDLE) != RESET)
    {
      __HAL_UART_CLEAR_IDLEFLAG(&UART_HANDLE);
      uart_receive_proc();
    }
    HAL_UART_IRQHandler(&UART_HANDLE);
}

/*************************************************************************************
  * @brief 使用dma发送串口数据
  * @param pdata - 待发送的buffer
           num - 待发送的字节数
  * @retval 校验值
**************************************************************************************/
static int8_t uart_dma_send(uint8_t* pdata, uint32_t num)
{
	int8_t ret = 0;
    memcpy(s_uart_tx_dma_buffer, pdata, num);
	if (HAL_UART_Transmit_DMA(&UART_HANDLE, s_uart_tx_dma_buffer, num) != HAL_OK)
	{
		ret = (-1);
	}
	
	return ret;
}

/*************************************************************************************
  * @brief 使用dma接收串口数据
  * @param timeout_ms-超时时间
  * @retval 校验值
**************************************************************************************/
static int8_t uart_dma_receive(uint32_t timeout_ms)
{
	int8_t ret = 0;
    uint32_t tick = HAL_GetTick();

	/*等待数据接收完成，超时退出*/
	while ((false == g_uart_received) && ((HAL_GetTick() - tick) < timeout_ms));
	
	if(g_uart_received == true)
	{
		g_uart_received = false;
	}
    else
    {
        ret = (-1);
    }
	
	return ret;
}
/*=<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<=*/





/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
**************************************数据读写函数区*************************************
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
/*************************************************************************************
  * @brief LRC校验值计算
  * @param ptr - 待计算的buffer
           len - 计算的字节数
  * @retval 校验值
**************************************************************************************/
uint8_t lrc_cal(const uint8_t* ptr, uint32_t len)
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

/**************************************************************************************
 * @bridf: 读数据
 * @param func_code - 功能码
 * @param addr - 起始地址
 * @param len - 读取的字节数
 * @return NULL:失败;
 *         非NULL:有效数据的buffer，其中第一个字节是status,第二个字节开始是有效数据
 **************************************************************************************/
uint8_t *fin_master_ex_read(uint8_t func_code, uint32_t addr, uint16_t len)
{
	uint32_t start_addr = addr;
	uint16_t datalen = len;
    uint16_t total;
	uint8_t crc = 0;
    uint8_t *ptr = NULL;	
	total = FIN_PCL_UART_HEAD_LEN + 1;

	if(total > sizeof(s_fm_tx_buff))
	{
		return NULL;
	}
	/*数据填充*/
	s_fm_tx_buff[0] = 0x55;
	s_fm_tx_buff[1] = 0xAA;
	s_fm_tx_buff[2] = ((total-5) & 0xff);
	s_fm_tx_buff[3] = ((total-5) >> 8);
	s_fm_tx_buff[4] = SLAVE_DEVICE_ID;
	s_fm_tx_buff[5] = 0x00;
	s_fm_tx_buff[6] = func_code | (1<<7);
	*(uint32_t*)(s_fm_tx_buff + 7) = start_addr;
	*(uint16_t*)(s_fm_tx_buff + 11) = datalen;
	
	crc = lrc_cal(s_fm_tx_buff, total - 1);
	s_fm_tx_buff[FIN_PCL_UART_HEAD_LEN] = crc;

	if(len > (sizeof(s_fm_tx_buff) - 10))
	{
		return NULL;
	}
	/*数据发送*/
	g_uart_received = false;
	uart_dma_send(s_fm_tx_buff, total);
	
	/*数据接收*/
	total = FIN_PCL_UART_HEAD_LEN + 1 + len + 1;
	if (uart_dma_receive(500) < 0)
	{
		return NULL;
	}
	/*对接收数据进行校验*/
	if(lrc_cal(s_fm_rx_buff, total-1) != s_fm_rx_buff[total-1])
	{
		printf("crc fail 1");
	}
	else
	{
		ptr = s_fm_rx_buff;
	}

    return ptr;
}


/**************************************************************************************
 * @bridf: 写数据
 * @param func_code - 功能码
 * @param addr - 起始地址
 * @param len - 写入的字节数
 * @param *pdata  - 待写入的数据
 * @return	0:成功
		   -1:失败
 **************************************************************************************/
int32_t fin_master_ex_write(uint8_t func_code, uint32_t addr, uint16_t len, int8_t *pdata)
{
    uint16_t total;
    uint8_t crc = 0;
    int32_t ret = 0;

	uint32_t startaddr = addr;
	uint16_t datalen = len;
	total = FIN_PCL_UART_HEAD_LEN + len + 1;

	if(total > sizeof(s_fm_tx_buff))
	{
		return (-1);
	}
	/*数据填充*/
	s_fm_tx_buff[0] = 0x55;
	s_fm_tx_buff[1] = 0xAA;
	s_fm_tx_buff[2] = ((total-5) & 0xff);
	s_fm_tx_buff[3] = ((total-5) >> 8);
	s_fm_tx_buff[4] = SLAVE_DEVICE_ID;
	s_fm_tx_buff[5] = 0x00;
	s_fm_tx_buff[6] = func_code & (~(1<<7));
	*(uint32_t*)(s_fm_tx_buff + 7) = startaddr;
	*(uint16_t*)(s_fm_tx_buff + 11) = datalen;

	memcpy(s_fm_tx_buff + FIN_PCL_UART_HEAD_LEN, pdata, len);
	crc = lrc_cal(s_fm_tx_buff, total - 1);
	s_fm_tx_buff[total-1] = crc;
	
	/*数据发送*/
	g_uart_received = false;
	uart_dma_send( s_fm_tx_buff, total);
	
	/*数据接收， 写入状态判断*/
	total = FIN_PCL_UART_HEAD_LEN + 1 + 1;
	if (uart_dma_receive(500) < 0 || s_fm_rx_buff[FIN_PCL_UART_HEAD_LEN] != 0)
	{
		ret = (-1);
	}

    return ret;
}

/**************************************************************************************
 * @bridf: 读写测试
 * @param:
 * @return	0:成功
		   -1:失败
 **************************************************************************************/
int32_t read_write_test(void)
{
	int32_t ret = 0;
	uint8_t func_code = 0x79;
	uint16_t addr = 0x00;
	uint8_t reg_val = 0x02;
	uint8_t *rev_buffer = NULL;
	uint16_r len = sizeof(reg_val);

	if (fin_master_ex_write(func_code, addr, len, &reg_val) < 0)
	{
		ret = (-1);
		printf("read_write_test error");
	}
	
	func_code = 0x7B;
	addr = 0x040E;
	len = 0x20;
	rev_buffer = fin_master_ex_read(func_code, addr, len);
	if( NULL == rev_buffer)
	{
		ret = (-1);
		printf("read_write_test error");
	}
}
/*=<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<**<=*/



/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
**************************************OTA升级函数区*************************************
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
/*************************************************************************************/
/*升级标志: 开始/升级中：1, 结束或失败：0*/
uint8_t upgrade_flag = 0;
/*升级类型*/
uint8_t upgrade_type = 0;
/*升级文件总长度*/
uint32_t upgrade_total_len = 0;
/*已经接收的长度*/
uint32_t upgrade_have_recv_len = 0;


/**************************************************************************************
 * @bridf: 进入OTA模式
 * @param type - 文件类型
 * @param total_len - 文件总长度
 * @return	0:成功
		   -1:失败
 **************************************************************************************/
int32_t enter_ota_mode(uint8_t type, uint32_t total_len)
{
	int32_t ret = 0;
	uint8_t func_code = 0x7a;
	uint16_t addr = 0x00;
	uint8_t reg_val = 0x01;
	
	/*发送命令，进入OTA模式*/
	if (fin_master_ex_write(func_code, addr, sizeof(reg_val), &reg_val) < 0)
	{
		ret = (-1);
		printf("read_write_test error");
	}
	else
	{
		/*设置文件类型，文件总长度，重置已接收文件长度，更新升级标识*/
		upgrade_type = type;
		upgrade_total_len = total_len;
		upgrade_have_recv_len = 0;
		upgrade_flag = 1;
	}
	
	return ret;
}


/*************************************************************************************
  * @brief OTA升级文件接口
  * @param  buffer - 需要写入的数据
            len - 长度
  * @retval 0 - 成功； 非0 - 失败
**************************************************************************************/
int32_t ota_write(uint8_t *buffer, uint16_t len)
{
	int32_t ret = 0;;
    int32_t ack = (-1);
    uint32_t retry = 0;

    if(0 >= len)
    {
        ret = (-1);
		upgrade_flag = 0;
		goto __exit__;
    }

    if(1 != upgrade_flag)
    {
        ret = (-1);
		upgrade_flag = 0;
		goto __exit__;
    }

    if(1 == upgrade_type)
    {
        do
        {
			/*写文件*/
            ack = fin_master_ex_write(0x78, upgrade_have_recv_len, len, buffer);
			retry++;
			OS_DELAY(5);
            if(retry > 1000)
            {
				/*失败次数过多，升级失败，退出*/
				ret = (-1);
				upgrade_flag = 0;
                goto __exit__;
            }
        }while(ack == (-1));
    }

	if(0 == ack)
	{
		/*文件写入成功，记录已接收文件长度*/
		upgrade_have_recv_len += len;
	}
         
    if(upgrade_total_len == upgrade_have_recv_len)
    {
		/*写入文件长度等于总长度，升级传输完毕*/
        if(1 == upgrade_type)
        {
			/*等待升级完成*/
            OS_DELAY(500);
            OS_DELAY(500);
            OS_DELAY(500);
            OS_DELAY(500);
			upgrade_flag = 0;
        }
    }

__exit__:

    return ret;
}

