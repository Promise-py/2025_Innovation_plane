#include "ex_05a_main.h"
#include <string.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi.h"
#include "DWMport.h"
#include "flash.h"
#include "spi.h"
#include "trilateration.h"

#define RNG_DELAY_MS 10      

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16460


//各个序列包
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};//已验证
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8 tx_poll_msg_station2[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'I', 'O', 'N', 0x21, 0, 0};//已验证
static uint8 rx_resp_msg_station2[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'N', 'O', 'T', 'I', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg_station2[] = {0x41, 0x88, 0, 0xCA, 0xDE,'T', 'I', 'O', 'N', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	
static uint8 tx_poll_msg_station3[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'F', 'I', 'G', 'H', 0x21, 0, 0};//已验证
static uint8 rx_resp_msg_station3[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'H', 'G', 'I', 'F', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg_station3[] = {0x41, 0x88, 0, 0xCA, 0xDE,'F', 'I', 'G', 'H', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};	
	
static uint8 tx_poll_msg_station4[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'H', 'I', 'T', 'E', 0x21, 0, 0};//已验证
static uint8 rx_resp_msg_station4[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'E', 'H', 'I', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg_station4[] = {0x41, 0x88, 0, 0xCA, 0xDE,'H', 'I', 'T', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};	


/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
uint8 frame_seq_nb_station1 = 0;//记录通信次数
uint8 frame_seq_nb_station2 = 0;
uint8 frame_seq_nb_station3 = 0;//帧序列号
uint8 frame_seq_nb_station4 = 0;

uint8 copy_frame1 = 0;
uint8 copy_frame2 = 0;
uint8 copy_frame3 = 0;
uint8 copy_frame4 = 0;
	
static uint8 check_UWB1_work = 0;
static uint8 check_UWB2_work = 0;
static uint8 check_UWB3_work = 0;
static uint8 check_UWB4_work = 0;

uint16_t dis1,dis2,dis3,dis4;//添加四个接收的变量和指针，然后添加接收的语句
uint16_t *p1 = &dis1;
uint16_t *p2 = &dis2;
uint16_t *p3 = &dis3;
uint16_t *p4 = &dis4;

float real_dis1;
float real_dis2;
float real_dis3;
float real_dis4;

int int_dis1, int_dis2, int_dis3, int_dis4;

//uint8_t  DISTANCE[6];

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];//可能需要定义4个接收数组来接收

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;


/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 祍 and 1 祍 = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 300

/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100

/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 2700

/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 8

#define buffer_size sizeof(finish_jiaozhun)

/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);
void change_float_u16Arry(uint16_t *u16Arry, float *floatdata1, float *floatdata2, float *floatdata3, float *floatdata4);
float sliding_average_filter1(float distance, uint8_t n);
float sliding_average_filter2(float distance, uint8_t n);
float sliding_average_filter3(float distance, uint8_t n);
float sliding_average_filter4(float distance, uint8_t n);

float delta_dis1 = 0.2;//测试
float delta_dis2 = 0.2;
float delta_dis3 = 0.2;
float delta_dis4 = 0.2;

float test_dis1 = 0.0;
float test_dis2 = 0.0;
float test_dis3 = 0.0;
float test_dis4 = 0.0;


float delta_dis1_sum = 0.0;
float delta_dis2_sum = 0.0;
float delta_dis3_sum = 0.0;
float delta_dis4_sum = 0.0;

uint8_t cnt_num1 = 0;
uint8_t cnt_num2 = 0;
uint8_t cnt_num3 = 0;
uint8_t cnt_num4 = 0;

uint8_t cnt_num[50] = {0};
void float_u8(uint8_t *u8Arry, float *floatdata1, float *floatdata2, float *floatdata3, float *floatdata4);
void change_u16Arry_float(uint16_t *u16Arry, float *delta_dis1, float *delta_dis2, float *delta_dis3, float *delta_dis4);

uint8_t flag = 0x02;


uint16_t TEXT_Buffer_1[20] = {0};
uint16_t TEXT_Buffer_2[20] = {0};

void dw_init(void)
{
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    port_set_dw1000_slowrate();
		
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        while (1)
        {
					//加个延时，超时退出
		};
    }
		
    port_set_dw1000_fastrate();
    dwt_configure(&config);//只有这个函数是必须的

    dwt_setrxantennadelay(RX_ANT_DLY);//天线延迟时间，即到天线的时间
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);//接收信号（数据）延迟时间
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);
}

int dw_Receive(void)
{
	switch (flag){
			
		    case 0x01:  //校准用的
			{
                //基站1
				tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb_station1;
				dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{
							//加个延时，超时退出
						};


				if (status_reg & SYS_STATUS_RXFCG)
				{
					uint32 frame_len;
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0); 
					}

					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
					{
						uint32 final_tx_time;
						int ret;
									
						*p1 = rx_buffer[11]*256 + rx_buffer[12]; //接收测距数据以uint16_t的形式存储
						real_dis1 = ((float) dis1)/100.0; //转换成float  单位: m
															
						poll_tx_ts = get_tx_timestamp_u64();
						resp_rx_ts = get_rx_timestamp_u64();

						final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(final_tx_time);
						final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

						final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
						final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
						final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

						tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb_station1;
						dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
						dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
						ret = dwt_starttx(DWT_START_TX_DELAYED);

						if (ret == DWT_SUCCESS)
						{
							while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{ 
													
												};

							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
							frame_seq_nb_station1++;
						}
					}
				}
				else
				{
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
					dwt_rxreset();
				}
				
				
				real_dis1 = sliding_average_filter1(real_dis1,10);
								
				if(real_dis1 > 10.0)  check_UWB1_work = 0;
				else  check_UWB1_work = 1;
				
				if(copy_frame1 == frame_seq_nb_station1)
					check_UWB1_work = 0;
				
				copy_frame1 = frame_seq_nb_station1;
				

				if(frame_seq_nb_station1 > 20 && check_UWB1_work == 1)
				{
			        delta_dis1_sum += real_dis1;
                    cnt_num1++;
					
				}
				
				//基站2
				tx_poll_msg_station2[ALL_MSG_SN_IDX] = frame_seq_nb_station2;
				dwt_writetxdata(sizeof(tx_poll_msg_station2), tx_poll_msg_station2, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_poll_msg_station2), 0, 1); /* Zero offset in TX buffer, ranging. */
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{ 
							
						};

				if (status_reg & SYS_STATUS_RXFCG)
				{
					uint32 frame_len;

					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0);
					}

					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_resp_msg_station2, ALL_MSG_COMMON_LEN) == 0)
					{
						uint32 final_tx_time;
						int ret;
									
						*p2 = rx_buffer[11]*256+rx_buffer[12];//接收测距数据以uint16_t的形式存储
						real_dis2 = ((float) dis2)/100.0;//转换成float
									
						poll_tx_ts = get_tx_timestamp_u64();
						resp_rx_ts = get_rx_timestamp_u64();

						final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(final_tx_time);
						final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

						final_msg_set_ts(&tx_final_msg_station2[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
						final_msg_set_ts(&tx_final_msg_station2[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
						final_msg_set_ts(&tx_final_msg_station2[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

						tx_final_msg_station2[ALL_MSG_SN_IDX] = frame_seq_nb_station2;
						dwt_writetxdata(sizeof(tx_final_msg_station2), tx_final_msg_station2, 0); /* Zero offset in TX buffer. */
						dwt_writetxfctrl(sizeof(tx_final_msg_station2), 0, 1); /* Zero offset in TX buffer, ranging. */
						ret = dwt_starttx(DWT_START_TX_DELAYED);

						if (ret == DWT_SUCCESS)
						{
							while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{ 
													
												};
							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
							frame_seq_nb_station2++;
						}
										
					}
				}
				else
				{
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
					dwt_rxreset();
				}
										
				real_dis2 = sliding_average_filter2(real_dis2,10);
				
				if(real_dis2 > 10.0)  check_UWB2_work = 0;
				else  check_UWB2_work = 1;
		
				if(copy_frame2 == frame_seq_nb_station2)
					check_UWB2_work = 0;
				
		        copy_frame2 = frame_seq_nb_station2;    
				
				if(frame_seq_nb_station2 > 20 && check_UWB2_work == 1)	
				{
					delta_dis2_sum += real_dis2;				
					cnt_num2++;
				}

				
				//基站3 
				tx_poll_msg_station3[ALL_MSG_SN_IDX] = frame_seq_nb_station3;
				dwt_writetxdata(sizeof(tx_poll_msg_station3), tx_poll_msg_station3, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_poll_msg_station3), 0, 1); /* Zero offset in TX buffer, ranging. */
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{ 
							
						};

				if (status_reg & SYS_STATUS_RXFCG)
				{
					uint32 frame_len;

					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0);
					}

					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_resp_msg_station3, ALL_MSG_COMMON_LEN) == 0)
					{
						uint32 final_tx_time;
						int ret;
									
						*p3 = rx_buffer[11]*256+rx_buffer[12];//接收测距数据以uint16_t的形式存储
						real_dis3 = ((float) dis3)/100.0;//转换成float
									
						poll_tx_ts = get_tx_timestamp_u64();
						resp_rx_ts = get_rx_timestamp_u64();

						final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(final_tx_time);
						final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

						final_msg_set_ts(&tx_final_msg_station3[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
						final_msg_set_ts(&tx_final_msg_station3[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
						final_msg_set_ts(&tx_final_msg_station3[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

						tx_final_msg_station3[ALL_MSG_SN_IDX] = frame_seq_nb_station3;
						dwt_writetxdata(sizeof(tx_final_msg_station3), tx_final_msg_station3, 0); /* Zero offset in TX buffer. */
						dwt_writetxfctrl(sizeof(tx_final_msg_station3), 0, 1); /* Zero offset in TX buffer, ranging. */
						ret = dwt_starttx(DWT_START_TX_DELAYED);

						if (ret == DWT_SUCCESS)
						{
							while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{
													
												};

							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
							frame_seq_nb_station3++;
						}

					}
				}
				else
				{
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
					dwt_rxreset();
				}

				real_dis3 = sliding_average_filter3(real_dis3,10);
				
	    		if(real_dis3 > 10.0)  check_UWB3_work = 0;
				else  check_UWB3_work = 1;	    
		
				if(copy_frame3 == frame_seq_nb_station3)
					check_UWB3_work = 0;
				
		        copy_frame3 = frame_seq_nb_station3;
				
				if(frame_seq_nb_station3 > 20 && check_UWB3_work == 1)
				{
					delta_dis3_sum += real_dis3;				
					cnt_num3++;
					
				}
				
				//基站4
				tx_poll_msg_station4[ALL_MSG_SN_IDX] = frame_seq_nb_station4;
				dwt_writetxdata(sizeof(tx_poll_msg_station4), tx_poll_msg_station4, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_poll_msg_station4), 0, 1); /* Zero offset in TX buffer, ranging. */

				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{ 
							
						};


				if (status_reg & SYS_STATUS_RXFCG)
				{
					uint32 frame_len;

					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0);
					}

					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_resp_msg_station4, ALL_MSG_COMMON_LEN) == 0)
					{
						uint32 final_tx_time;
						int ret;

						*p4 = rx_buffer[11]*256+rx_buffer[12];//接收测距数据以uint16_t的形式存储
						real_dis4 = ((float) dis4)/100.0;//转换成float							
									
						poll_tx_ts = get_tx_timestamp_u64();
						resp_rx_ts = get_rx_timestamp_u64();

						final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(final_tx_time);
						final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

						final_msg_set_ts(&tx_final_msg_station4[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
						final_msg_set_ts(&tx_final_msg_station4[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
						final_msg_set_ts(&tx_final_msg_station4[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

						tx_final_msg_station4[ALL_MSG_SN_IDX] = frame_seq_nb_station4;
						dwt_writetxdata(sizeof(tx_final_msg_station4), tx_final_msg_station4, 0); /* Zero offset in TX buffer. */
						dwt_writetxfctrl(sizeof(tx_final_msg_station4), 0, 1); /* Zero offset in TX buffer, ranging. */
						ret = dwt_starttx(DWT_START_TX_DELAYED);

						if (ret == DWT_SUCCESS)
						{
							while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{
													
												};
							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
							frame_seq_nb_station4++;
						}
					}
				}
				else
				{
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
					dwt_rxreset();
				}
				
				real_dis4 = sliding_average_filter4(real_dis4,10);
				
				if(real_dis4 > 10.0)  check_UWB4_work = 0;
				else  check_UWB4_work = 1;
		
				if(copy_frame4 == frame_seq_nb_station4)
					check_UWB4_work = 0;
				
		        copy_frame4 = frame_seq_nb_station4;
				
			    if(frame_seq_nb_station4 > 20 && check_UWB4_work == 1)
				{
				    delta_dis4_sum += real_dis4;
                    cnt_num4++;
					
				}

				
				if(frame_seq_nb_station4 > 100 && frame_seq_nb_station3 > 100 && frame_seq_nb_station2 > 100 && frame_seq_nb_station1 > 100)
				{
					cnt_num[0] = 0x2c;
					cnt_num[1] = 0x12;
					cnt_num[2] = frame_seq_nb_station1;
					cnt_num[3] = frame_seq_nb_station2;
					cnt_num[4] = frame_seq_nb_station3;
					cnt_num[5] = frame_seq_nb_station4;
					cnt_num[6] = copy_frame1;					
					cnt_num[7] = copy_frame2;
					cnt_num[8] = copy_frame3;
					cnt_num[9] = copy_frame4;
					cnt_num[10] = cnt_num1;
                    cnt_num[11] = cnt_num2;
					cnt_num[12] = cnt_num3;
					cnt_num[13] = cnt_num4;
				
					delta_dis1 = delta_dis1_sum / cnt_num1 - 2.20;  ////////根据实际距离修改
					delta_dis2 = delta_dis2_sum / cnt_num2 - 2.50;  ////////根据实际距离修改
					delta_dis3 = delta_dis3_sum / cnt_num3 - 2.28;  ////////根据实际距离修改
				    delta_dis4 = delta_dis4_sum / cnt_num4 - 2.03;  ////////根据实际距离修改
					

					change_float_u16Arry(TEXT_Buffer_1, &delta_dis1, &delta_dis2, &delta_dis3, &delta_dis4);
             		STMFLASH_Read(FLASH_SAVE_ADDR, TEXT_Buffer_2, 8);
				    change_u16Arry_float(TEXT_Buffer_2, &delta_dis1, &delta_dis2, &delta_dis3, &delta_dis4);
					
					
//					float_u8(cnt_num, &delta_dis1, &delta_dis2, &delta_dis3, &delta_dis4);			
//					HAL_UART_Transmit(&huart1, cnt_num, 30, 0xffff);

				    frame_seq_nb_station1 = frame_seq_nb_station2 = frame_seq_nb_station3 = frame_seq_nb_station4 = 0;
					copy_frame1 = copy_frame2 = copy_frame3 = copy_frame4 = 0;
					cnt_num1 = cnt_num2 = cnt_num3 = cnt_num4 = 0;
					delta_dis1_sum = delta_dis2_sum = delta_dis3_sum = delta_dis4_sum = 0.0; 
					flag = 0x02;

				}

			}
            break;
			
			case 0x02:
			{
				//基站1
				tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb_station1;
				dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{
							//加个延时，超时退出
						};


				if (status_reg & SYS_STATUS_RXFCG)
				{
					uint32 frame_len;
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0);
					}

					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
					{
						uint32 final_tx_time;
						int ret;
									
						*p1 = rx_buffer[11]*256 + rx_buffer[12]; //接收测距数据以uint16_t的形式存储
						real_dis1 = ((float) dis1)/100.0; //转换成float  单位: m
															
						poll_tx_ts = get_tx_timestamp_u64();
						resp_rx_ts = get_rx_timestamp_u64();

						final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(final_tx_time);
						final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

						final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
						final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
						final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

						tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb_station1;
						dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
						dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
						ret = dwt_starttx(DWT_START_TX_DELAYED);

						if (ret == DWT_SUCCESS)
						{
							while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{ 
													
												};

							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
							frame_seq_nb_station1++;
						}
					}
				}
				else
				{
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
					dwt_rxreset();
				}
				
				
				real_dis1 = sliding_average_filter1(real_dis1,10);
                real_dis1 -= delta_dis1;//减去校准误差
				
				int_dis1 = (int)(real_dis1*1000);  //转换成int 单位：mm
				
				
				if(real_dis1 > 10.0)  check_UWB1_work = 0;
				else  check_UWB1_work = 1;
				
				if(copy_frame1 == frame_seq_nb_station1)
					check_UWB1_work = 0;
				copy_frame1 = frame_seq_nb_station1;

				if(check_UWB1_work)
					Trilateration(int_dis1, int_dis2, int_dis3, int_dis4, real_dis1, 0);

				//基站2
				tx_poll_msg_station2[ALL_MSG_SN_IDX] = frame_seq_nb_station2;
				dwt_writetxdata(sizeof(tx_poll_msg_station2), tx_poll_msg_station2, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_poll_msg_station2), 0, 1); /* Zero offset in TX buffer, ranging. */
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{ 
							
						};

				if (status_reg & SYS_STATUS_RXFCG)
				{
					uint32 frame_len;

					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0);
					}

					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_resp_msg_station2, ALL_MSG_COMMON_LEN) == 0)
					{
						uint32 final_tx_time;
						int ret;
									
						*p2 = rx_buffer[11]*256+rx_buffer[12];//接收测距数据以uint16_t的形式存储
						real_dis2 = ((float) dis2)/100.0;//转换成float
									
						poll_tx_ts = get_tx_timestamp_u64();
						resp_rx_ts = get_rx_timestamp_u64();

						final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(final_tx_time);
						final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

						final_msg_set_ts(&tx_final_msg_station2[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
						final_msg_set_ts(&tx_final_msg_station2[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
						final_msg_set_ts(&tx_final_msg_station2[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

						tx_final_msg_station2[ALL_MSG_SN_IDX] = frame_seq_nb_station2;
						dwt_writetxdata(sizeof(tx_final_msg_station2), tx_final_msg_station2, 0); /* Zero offset in TX buffer. */
						dwt_writetxfctrl(sizeof(tx_final_msg_station2), 0, 1); /* Zero offset in TX buffer, ranging. */
						ret = dwt_starttx(DWT_START_TX_DELAYED);

						if (ret == DWT_SUCCESS)
						{
							while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{ 
													
												};
							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
							frame_seq_nb_station2++;
						}
										
					}
				}
				else
				{
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
					dwt_rxreset();
				}
										
				real_dis2 = sliding_average_filter2(real_dis2,10);
				real_dis2 -= delta_dis2;
				int_dis2 = (int)(real_dis2*1000);  //转换成int 单位：mm
				
				if(real_dis2 > 10.0)  check_UWB2_work = 0;
				else  check_UWB2_work = 1;
		
				if(copy_frame2 == frame_seq_nb_station2)
					check_UWB2_work = 0;
		        copy_frame2 = frame_seq_nb_station2;    
				
				if(check_UWB2_work)	
					Trilateration(int_dis1, int_dis2, int_dis3, int_dis4, real_dis2, 1);
				
				//基站3
				tx_poll_msg_station3[ALL_MSG_SN_IDX] = frame_seq_nb_station3;
				dwt_writetxdata(sizeof(tx_poll_msg_station3), tx_poll_msg_station3, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_poll_msg_station3), 0, 1); /* Zero offset in TX buffer, ranging. */
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{ 
							
						};

				if (status_reg & SYS_STATUS_RXFCG)
				{
					uint32 frame_len;

					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0);
					}

					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_resp_msg_station3, ALL_MSG_COMMON_LEN) == 0)
					{
						uint32 final_tx_time;
						int ret;
									
						*p3 = rx_buffer[11]*256+rx_buffer[12];//接收测距数据以uint16_t的形式存储
						real_dis3 = ((float) dis3)/100.0;//转换成float
									
						poll_tx_ts = get_tx_timestamp_u64();
						resp_rx_ts = get_rx_timestamp_u64();

						final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(final_tx_time);
						final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

						final_msg_set_ts(&tx_final_msg_station3[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
						final_msg_set_ts(&tx_final_msg_station3[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
						final_msg_set_ts(&tx_final_msg_station3[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

						tx_final_msg_station3[ALL_MSG_SN_IDX] = frame_seq_nb_station3;
						dwt_writetxdata(sizeof(tx_final_msg_station3), tx_final_msg_station3, 0); /* Zero offset in TX buffer. */
						dwt_writetxfctrl(sizeof(tx_final_msg_station3), 0, 1); /* Zero offset in TX buffer, ranging. */
						ret = dwt_starttx(DWT_START_TX_DELAYED);

						if (ret == DWT_SUCCESS)
						{
							while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{
													
												};

							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
							frame_seq_nb_station3++;
						}

					}
				}
				else
				{
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
					dwt_rxreset();
				}

				real_dis3 = sliding_average_filter3(real_dis3,10);
				real_dis3 -= delta_dis3;
				int_dis3 = (int)(real_dis3*1000);  //转换成int 单位：mm
								
	    		if(real_dis3 > 10.0)  check_UWB3_work = 0;
				else  check_UWB3_work = 1;	    
		
				if(copy_frame3 == frame_seq_nb_station3)
					check_UWB3_work = 0;
		        copy_frame3 = frame_seq_nb_station3;
				

				if(check_UWB3_work)
					Trilateration(int_dis1, int_dis2, int_dis3, int_dis4, real_dis3, 2);
				
				//基站4
				tx_poll_msg_station4[ALL_MSG_SN_IDX] = frame_seq_nb_station4;
				dwt_writetxdata(sizeof(tx_poll_msg_station4), tx_poll_msg_station4, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_poll_msg_station4), 0, 1); /* Zero offset in TX buffer, ranging. */

				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{ 
							
						};


				if (status_reg & SYS_STATUS_RXFCG)
				{
					uint32 frame_len;

					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0);
					}

					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_resp_msg_station4, ALL_MSG_COMMON_LEN) == 0)
					{
						uint32 final_tx_time;
						int ret;

						*p4 = rx_buffer[11]*256+rx_buffer[12];//接收测距数据
						real_dis4 = ((float) dis4)/100.0;//转换成float							
									
						poll_tx_ts = get_tx_timestamp_u64();
						resp_rx_ts = get_rx_timestamp_u64();

						final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(final_tx_time);
						final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

						final_msg_set_ts(&tx_final_msg_station4[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
						final_msg_set_ts(&tx_final_msg_station4[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
						final_msg_set_ts(&tx_final_msg_station4[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

						tx_final_msg_station4[ALL_MSG_SN_IDX] = frame_seq_nb_station4;
						dwt_writetxdata(sizeof(tx_final_msg_station4), tx_final_msg_station4, 0); /* Zero offset in TX buffer. */
						dwt_writetxfctrl(sizeof(tx_final_msg_station4), 0, 1); /* Zero offset in TX buffer, ranging. */
						ret = dwt_starttx(DWT_START_TX_DELAYED);

						if (ret == DWT_SUCCESS)
						{
							while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{
													
												};
							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
							frame_seq_nb_station4++;
						}
					}
				}
				else
				{
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
					dwt_rxreset();
				}
				
				real_dis4 = sliding_average_filter4(real_dis4,10);
				real_dis4 -= delta_dis4;
				int_dis4 = (int)(real_dis4*1000);  //转换成int 单位：mm
								
				
				if(real_dis4 > 10.0)  check_UWB4_work = 0;
				else  check_UWB4_work = 1;
		
				if(copy_frame4 == frame_seq_nb_station4)
					check_UWB4_work = 0;
		        copy_frame4 = frame_seq_nb_station4;
		
			    if(check_UWB4_work)
					Trilateration(int_dis1, int_dis2, int_dis3, int_dis4, real_dis4, 3);

				if(check_UWB4_work & check_UWB3_work & check_UWB2_work & check_UWB1_work)
					Trilateration(int_dis1, int_dis2, int_dis3, int_dis4, real_dis4, 3);
			}
            break;
			
			default: break;
		}
}

int dw_main(void)
{
//	STMFLASH_Read(FLASH_SAVE_ADDR, TEXT_Buffer_2, 8);
//	change_u16Arry_float(TEXT_Buffer_2, &delta_dis1, &delta_dis2, &delta_dis3, &delta_dis4);
	
	
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    port_set_dw1000_slowrate();
		
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        while (1)
        {
					//加个延时，超时退出
		};
    }
		
    port_set_dw1000_fastrate();
    dwt_configure(&config);//只有这个函数是必须的

    dwt_setrxantennadelay(RX_ANT_DLY);//天线延迟时间，即到天线的时间
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);//接收信号（数据）延迟时间
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

		
    while (1)
    {

		switch (flag){
			
		    case 0x01:  //校准用的
			{
                //基站1
				tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb_station1;
				dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{
							//加个延时，超时退出
						};


				if (status_reg & SYS_STATUS_RXFCG)
				{
					uint32 frame_len;
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0); 
					}

					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
					{
						uint32 final_tx_time;
						int ret;
									
						*p1 = rx_buffer[11]*256 + rx_buffer[12]; //接收测距数据以uint16_t的形式存储
						real_dis1 = ((float) dis1)/100.0; //转换成float  单位: m
															
						poll_tx_ts = get_tx_timestamp_u64();
						resp_rx_ts = get_rx_timestamp_u64();

						final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(final_tx_time);
						final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

						final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
						final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
						final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

						tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb_station1;
						dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
						dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
						ret = dwt_starttx(DWT_START_TX_DELAYED);

						if (ret == DWT_SUCCESS)
						{
							while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{ 
													
												};

							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
							frame_seq_nb_station1++;
						}
					}
				}
				else
				{
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
					dwt_rxreset();
				}
				
				
				real_dis1 = sliding_average_filter1(real_dis1,10);
								
				if(real_dis1 > 10.0)  check_UWB1_work = 0;
				else  check_UWB1_work = 1;
				
				if(copy_frame1 == frame_seq_nb_station1)
					check_UWB1_work = 0;
				
				copy_frame1 = frame_seq_nb_station1;
				

				if(frame_seq_nb_station1 > 20 && check_UWB1_work == 1)
				{
			        delta_dis1_sum += real_dis1;
                    cnt_num1++;
					
				}
				
				//基站2
				tx_poll_msg_station2[ALL_MSG_SN_IDX] = frame_seq_nb_station2;
				dwt_writetxdata(sizeof(tx_poll_msg_station2), tx_poll_msg_station2, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_poll_msg_station2), 0, 1); /* Zero offset in TX buffer, ranging. */
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{ 
							
						};

				if (status_reg & SYS_STATUS_RXFCG)
				{
					uint32 frame_len;

					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0);
					}

					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_resp_msg_station2, ALL_MSG_COMMON_LEN) == 0)
					{
						uint32 final_tx_time;
						int ret;
									
						*p2 = rx_buffer[11]*256+rx_buffer[12];//接收测距数据以uint16_t的形式存储
						real_dis2 = ((float) dis2)/100.0;//转换成float
									
						poll_tx_ts = get_tx_timestamp_u64();
						resp_rx_ts = get_rx_timestamp_u64();

						final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(final_tx_time);
						final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

						final_msg_set_ts(&tx_final_msg_station2[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
						final_msg_set_ts(&tx_final_msg_station2[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
						final_msg_set_ts(&tx_final_msg_station2[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

						tx_final_msg_station2[ALL_MSG_SN_IDX] = frame_seq_nb_station2;
						dwt_writetxdata(sizeof(tx_final_msg_station2), tx_final_msg_station2, 0); /* Zero offset in TX buffer. */
						dwt_writetxfctrl(sizeof(tx_final_msg_station2), 0, 1); /* Zero offset in TX buffer, ranging. */
						ret = dwt_starttx(DWT_START_TX_DELAYED);

						if (ret == DWT_SUCCESS)
						{
							while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{ 
													
												};
							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
							frame_seq_nb_station2++;
						}
										
					}
				}
				else
				{
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
					dwt_rxreset();
				}
										
				real_dis2 = sliding_average_filter2(real_dis2,10);
				
				if(real_dis2 > 10.0)  check_UWB2_work = 0;
				else  check_UWB2_work = 1;
		
				if(copy_frame2 == frame_seq_nb_station2)
					check_UWB2_work = 0;
				
		        copy_frame2 = frame_seq_nb_station2;    
				
				if(frame_seq_nb_station2 > 20 && check_UWB2_work == 1)	
				{
					delta_dis2_sum += real_dis2;				
					cnt_num2++;
				}

				
				//基站3 
				tx_poll_msg_station3[ALL_MSG_SN_IDX] = frame_seq_nb_station3;
				dwt_writetxdata(sizeof(tx_poll_msg_station3), tx_poll_msg_station3, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_poll_msg_station3), 0, 1); /* Zero offset in TX buffer, ranging. */
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{ 
							
						};

				if (status_reg & SYS_STATUS_RXFCG)
				{
					uint32 frame_len;

					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0);
					}

					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_resp_msg_station3, ALL_MSG_COMMON_LEN) == 0)
					{
						uint32 final_tx_time;
						int ret;
									
						*p3 = rx_buffer[11]*256+rx_buffer[12];//接收测距数据以uint16_t的形式存储
						real_dis3 = ((float) dis3)/100.0;//转换成float
									
						poll_tx_ts = get_tx_timestamp_u64();
						resp_rx_ts = get_rx_timestamp_u64();

						final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(final_tx_time);
						final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

						final_msg_set_ts(&tx_final_msg_station3[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
						final_msg_set_ts(&tx_final_msg_station3[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
						final_msg_set_ts(&tx_final_msg_station3[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

						tx_final_msg_station3[ALL_MSG_SN_IDX] = frame_seq_nb_station3;
						dwt_writetxdata(sizeof(tx_final_msg_station3), tx_final_msg_station3, 0); /* Zero offset in TX buffer. */
						dwt_writetxfctrl(sizeof(tx_final_msg_station3), 0, 1); /* Zero offset in TX buffer, ranging. */
						ret = dwt_starttx(DWT_START_TX_DELAYED);

						if (ret == DWT_SUCCESS)
						{
							while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{
													
												};

							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
							frame_seq_nb_station3++;
						}

					}
				}
				else
				{
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
					dwt_rxreset();
				}

				real_dis3 = sliding_average_filter3(real_dis3,10);
				
	    		if(real_dis3 > 10.0)  check_UWB3_work = 0;
				else  check_UWB3_work = 1;	    
		
				if(copy_frame3 == frame_seq_nb_station3)
					check_UWB3_work = 0;
				
		        copy_frame3 = frame_seq_nb_station3;
				
				if(frame_seq_nb_station3 > 20 && check_UWB3_work == 1)
				{
					delta_dis3_sum += real_dis3;				
					cnt_num3++;
					
				}
				
				//基站4
				tx_poll_msg_station4[ALL_MSG_SN_IDX] = frame_seq_nb_station4;
				dwt_writetxdata(sizeof(tx_poll_msg_station4), tx_poll_msg_station4, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_poll_msg_station4), 0, 1); /* Zero offset in TX buffer, ranging. */

				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{ 
							
						};


				if (status_reg & SYS_STATUS_RXFCG)
				{
					uint32 frame_len;

					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0);
					}

					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_resp_msg_station4, ALL_MSG_COMMON_LEN) == 0)
					{
						uint32 final_tx_time;
						int ret;

						*p4 = rx_buffer[11]*256+rx_buffer[12];//接收测距数据以uint16_t的形式存储
						real_dis4 = ((float) dis4)/100.0;//转换成float							
									
						poll_tx_ts = get_tx_timestamp_u64();
						resp_rx_ts = get_rx_timestamp_u64();

						final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(final_tx_time);
						final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

						final_msg_set_ts(&tx_final_msg_station4[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
						final_msg_set_ts(&tx_final_msg_station4[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
						final_msg_set_ts(&tx_final_msg_station4[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

						tx_final_msg_station4[ALL_MSG_SN_IDX] = frame_seq_nb_station4;
						dwt_writetxdata(sizeof(tx_final_msg_station4), tx_final_msg_station4, 0); /* Zero offset in TX buffer. */
						dwt_writetxfctrl(sizeof(tx_final_msg_station4), 0, 1); /* Zero offset in TX buffer, ranging. */
						ret = dwt_starttx(DWT_START_TX_DELAYED);

						if (ret == DWT_SUCCESS)
						{
							while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{
													
												};
							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
							frame_seq_nb_station4++;
						}
					}
				}
				else
				{
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
					dwt_rxreset();
				}
				
				real_dis4 = sliding_average_filter4(real_dis4,10);
				
				if(real_dis4 > 10.0)  check_UWB4_work = 0;
				else  check_UWB4_work = 1;
		
				if(copy_frame4 == frame_seq_nb_station4)
					check_UWB4_work = 0;
				
		        copy_frame4 = frame_seq_nb_station4;
				
			    if(frame_seq_nb_station4 > 20 && check_UWB4_work == 1)
				{
				    delta_dis4_sum += real_dis4;
                    cnt_num4++;
					
				}

				
				if(frame_seq_nb_station4 > 100 && frame_seq_nb_station3 > 100 && frame_seq_nb_station2 > 100 && frame_seq_nb_station1 > 100)
				{
					cnt_num[0] = 0x2c;
					cnt_num[1] = 0x12;
					cnt_num[2] = frame_seq_nb_station1;
					cnt_num[3] = frame_seq_nb_station2;
					cnt_num[4] = frame_seq_nb_station3;
					cnt_num[5] = frame_seq_nb_station4;
					cnt_num[6] = copy_frame1;					
					cnt_num[7] = copy_frame2;
					cnt_num[8] = copy_frame3;
					cnt_num[9] = copy_frame4;
					cnt_num[10] = cnt_num1;
                    cnt_num[11] = cnt_num2;
					cnt_num[12] = cnt_num3;
					cnt_num[13] = cnt_num4;
				
					delta_dis1 = delta_dis1_sum / cnt_num1 - 2.20;  ////////根据实际距离修改
					delta_dis2 = delta_dis2_sum / cnt_num2 - 2.50;  ////////根据实际距离修改
					delta_dis3 = delta_dis3_sum / cnt_num3 - 2.28;  ////////根据实际距离修改
				    delta_dis4 = delta_dis4_sum / cnt_num4 - 2.03;  ////////根据实际距离修改
					

					change_float_u16Arry(TEXT_Buffer_1, &delta_dis1, &delta_dis2, &delta_dis3, &delta_dis4);
             		STMFLASH_Read(FLASH_SAVE_ADDR, TEXT_Buffer_2, 8);
				    change_u16Arry_float(TEXT_Buffer_2, &delta_dis1, &delta_dis2, &delta_dis3, &delta_dis4);
					
					
//					float_u8(cnt_num, &delta_dis1, &delta_dis2, &delta_dis3, &delta_dis4);			
//					HAL_UART_Transmit(&huart1, cnt_num, 30, 0xffff);

				    frame_seq_nb_station1 = frame_seq_nb_station2 = frame_seq_nb_station3 = frame_seq_nb_station4 = 0;
					copy_frame1 = copy_frame2 = copy_frame3 = copy_frame4 = 0;
					cnt_num1 = cnt_num2 = cnt_num3 = cnt_num4 = 0;
					delta_dis1_sum = delta_dis2_sum = delta_dis3_sum = delta_dis4_sum = 0.0; 
					flag = 0x02;

				}

			}
            break;
			
			
			
			
			
			
			
			
			


			case 0x02:
			{
				//基站1
				tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb_station1;
				dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{
							//加个延时，超时退出
						};


				if (status_reg & SYS_STATUS_RXFCG)
				{
					uint32 frame_len;
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0);
					}

					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
					{
						uint32 final_tx_time;
						int ret;
									
						*p1 = rx_buffer[11]*256 + rx_buffer[12]; //接收测距数据以uint16_t的形式存储
						real_dis1 = ((float) dis1)/100.0; //转换成float  单位: m
															
						poll_tx_ts = get_tx_timestamp_u64();
						resp_rx_ts = get_rx_timestamp_u64();

						final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(final_tx_time);
						final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

						final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
						final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
						final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

						tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb_station1;
						dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
						dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
						ret = dwt_starttx(DWT_START_TX_DELAYED);

						if (ret == DWT_SUCCESS)
						{
							while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{ 
													
												};

							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
							frame_seq_nb_station1++;
						}
					}
				}
				else
				{
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
					dwt_rxreset();
				}
				
				
				real_dis1 = sliding_average_filter1(real_dis1,10);
                real_dis1 -= delta_dis1;//减去校准误差
				
				int_dis1 = (int)(real_dis1*1000);  //转换成int 单位：mm
				
				
				if(real_dis1 > 10.0)  check_UWB1_work = 0;
				else  check_UWB1_work = 1;
				
				if(copy_frame1 == frame_seq_nb_station1)
					check_UWB1_work = 0;
				copy_frame1 = frame_seq_nb_station1;

				if(check_UWB1_work)
					Trilateration(int_dis1, int_dis2, int_dis3, int_dis4, real_dis1, 0);

				//基站2
				tx_poll_msg_station2[ALL_MSG_SN_IDX] = frame_seq_nb_station2;
				dwt_writetxdata(sizeof(tx_poll_msg_station2), tx_poll_msg_station2, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_poll_msg_station2), 0, 1); /* Zero offset in TX buffer, ranging. */
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{ 
							
						};

				if (status_reg & SYS_STATUS_RXFCG)
				{
					uint32 frame_len;

					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0);
					}

					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_resp_msg_station2, ALL_MSG_COMMON_LEN) == 0)
					{
						uint32 final_tx_time;
						int ret;
									
						*p2 = rx_buffer[11]*256+rx_buffer[12];//接收测距数据以uint16_t的形式存储
						real_dis2 = ((float) dis2)/100.0;//转换成float
									
						poll_tx_ts = get_tx_timestamp_u64();
						resp_rx_ts = get_rx_timestamp_u64();

						final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(final_tx_time);
						final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

						final_msg_set_ts(&tx_final_msg_station2[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
						final_msg_set_ts(&tx_final_msg_station2[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
						final_msg_set_ts(&tx_final_msg_station2[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

						tx_final_msg_station2[ALL_MSG_SN_IDX] = frame_seq_nb_station2;
						dwt_writetxdata(sizeof(tx_final_msg_station2), tx_final_msg_station2, 0); /* Zero offset in TX buffer. */
						dwt_writetxfctrl(sizeof(tx_final_msg_station2), 0, 1); /* Zero offset in TX buffer, ranging. */
						ret = dwt_starttx(DWT_START_TX_DELAYED);

						if (ret == DWT_SUCCESS)
						{
							while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{ 
													
												};
							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
							frame_seq_nb_station2++;
						}
										
					}
				}
				else
				{
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
					dwt_rxreset();
				}
										
				real_dis2 = sliding_average_filter2(real_dis2,10);
				real_dis2 -= delta_dis2;
				int_dis2 = (int)(real_dis2*1000);  //转换成int 单位：mm
				
				if(real_dis2 > 10.0)  check_UWB2_work = 0;
				else  check_UWB2_work = 1;
		
				if(copy_frame2 == frame_seq_nb_station2)
					check_UWB2_work = 0;
		        copy_frame2 = frame_seq_nb_station2;    
				
				if(check_UWB2_work)	
					Trilateration(int_dis1, int_dis2, int_dis3, int_dis4, real_dis2, 1);
				
				//基站3
				tx_poll_msg_station3[ALL_MSG_SN_IDX] = frame_seq_nb_station3;
				dwt_writetxdata(sizeof(tx_poll_msg_station3), tx_poll_msg_station3, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_poll_msg_station3), 0, 1); /* Zero offset in TX buffer, ranging. */
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{ 
							
						};

				if (status_reg & SYS_STATUS_RXFCG)
				{
					uint32 frame_len;

					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0);
					}

					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_resp_msg_station3, ALL_MSG_COMMON_LEN) == 0)
					{
						uint32 final_tx_time;
						int ret;
									
						*p3 = rx_buffer[11]*256+rx_buffer[12];//接收测距数据以uint16_t的形式存储
						real_dis3 = ((float) dis3)/100.0;//转换成float
									
						poll_tx_ts = get_tx_timestamp_u64();
						resp_rx_ts = get_rx_timestamp_u64();

						final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(final_tx_time);
						final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

						final_msg_set_ts(&tx_final_msg_station3[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
						final_msg_set_ts(&tx_final_msg_station3[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
						final_msg_set_ts(&tx_final_msg_station3[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

						tx_final_msg_station3[ALL_MSG_SN_IDX] = frame_seq_nb_station3;
						dwt_writetxdata(sizeof(tx_final_msg_station3), tx_final_msg_station3, 0); /* Zero offset in TX buffer. */
						dwt_writetxfctrl(sizeof(tx_final_msg_station3), 0, 1); /* Zero offset in TX buffer, ranging. */
						ret = dwt_starttx(DWT_START_TX_DELAYED);

						if (ret == DWT_SUCCESS)
						{
							while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{
													
												};

							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
							frame_seq_nb_station3++;
						}

					}
				}
				else
				{
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
					dwt_rxreset();
				}

				real_dis3 = sliding_average_filter3(real_dis3,10);
				real_dis3 -= delta_dis3;
				int_dis3 = (int)(real_dis3*1000);  //转换成int 单位：mm
								
	    		if(real_dis3 > 10.0)  check_UWB3_work = 0;
				else  check_UWB3_work = 1;	    
		
				if(copy_frame3 == frame_seq_nb_station3)
					check_UWB3_work = 0;
		        copy_frame3 = frame_seq_nb_station3;
				

				if(check_UWB3_work)
					Trilateration(int_dis1, int_dis2, int_dis3, int_dis4, real_dis3, 2);
				
				//基站4
				tx_poll_msg_station4[ALL_MSG_SN_IDX] = frame_seq_nb_station4;
				dwt_writetxdata(sizeof(tx_poll_msg_station4), tx_poll_msg_station4, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_poll_msg_station4), 0, 1); /* Zero offset in TX buffer, ranging. */

				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{ 
							
						};


				if (status_reg & SYS_STATUS_RXFCG)
				{
					uint32 frame_len;

					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0);
					}

					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_resp_msg_station4, ALL_MSG_COMMON_LEN) == 0)
					{
						uint32 final_tx_time;
						int ret;

						*p4 = rx_buffer[11]*256+rx_buffer[12];//接收测距数据
						real_dis4 = ((float) dis4)/100.0;//转换成float							
									
						poll_tx_ts = get_tx_timestamp_u64();
						resp_rx_ts = get_rx_timestamp_u64();

						final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
						dwt_setdelayedtrxtime(final_tx_time);
						final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

						final_msg_set_ts(&tx_final_msg_station4[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
						final_msg_set_ts(&tx_final_msg_station4[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
						final_msg_set_ts(&tx_final_msg_station4[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

						tx_final_msg_station4[ALL_MSG_SN_IDX] = frame_seq_nb_station4;
						dwt_writetxdata(sizeof(tx_final_msg_station4), tx_final_msg_station4, 0); /* Zero offset in TX buffer. */
						dwt_writetxfctrl(sizeof(tx_final_msg_station4), 0, 1); /* Zero offset in TX buffer, ranging. */
						ret = dwt_starttx(DWT_START_TX_DELAYED);

						if (ret == DWT_SUCCESS)
						{
							while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{
													
												};
							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
							frame_seq_nb_station4++;
						}
					}
				}
				else
				{
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
					dwt_rxreset();
				}
				
				real_dis4 = sliding_average_filter4(real_dis4,10);
				real_dis4 -= delta_dis4;
				int_dis4 = (int)(real_dis4*1000);  //转换成int 单位：mm
								
				
				if(real_dis4 > 10.0)  check_UWB4_work = 0;
				else  check_UWB4_work = 1;
		
				if(copy_frame4 == frame_seq_nb_station4)
					check_UWB4_work = 0;
		        copy_frame4 = frame_seq_nb_station4;
		
			    if(check_UWB4_work)
					Trilateration(int_dis1, int_dis2, int_dis3, int_dis4, real_dis4, 3);

				if(check_UWB4_work & check_UWB3_work & check_UWB2_work & check_UWB1_work)
					Trilateration(int_dis1, int_dis2, int_dis3, int_dis4, real_dis4, 3);
			}
            break;
			
			default: break;
		}

   }
				
}
		

void float_u8(uint8_t *u8Arry, float *floatdata1, float *floatdata2, float *floatdata3, float *floatdata4)
{
  uint8_t farray1[4], farray2[4], farray3[4], farray4[4];
	
  *(float *)farray1 = *floatdata1;
  *(float *)farray2 = *floatdata2;
  *(float *)farray3 = *floatdata3;
  *(float *)farray4 = *floatdata4;
	
  u8Arry[29] = farray4[0];
  u8Arry[28] = farray4[1];
  u8Arry[27] = farray4[2];
  u8Arry[26] = farray4[3];	
  u8Arry[25] = farray3[0];
  u8Arry[24] = farray3[1];
  u8Arry[23] = farray3[2];
  u8Arry[22] = farray3[3];	
  u8Arry[21] = farray2[0];
  u8Arry[20] = farray2[1];
  u8Arry[19] = farray2[2];
  u8Arry[18] = farray2[3];	
  u8Arry[17] = farray1[0];
  u8Arry[16] = farray1[1];
  u8Arry[15] = farray1[2];
  u8Arry[14] = farray1[3];

}

void change_float_u16Arry(uint16_t *u16Arry, float *floatdata1, float *floatdata2, float *floatdata3, float *floatdata4)
{
		uint8_t farray1[4], farray2[4], farray3[4], farray4[4];
	    uint8_t u8Arry[20];
		*(float *)farray1 = *floatdata1;
		*(float *)farray2 = *floatdata2;
		*(float *)farray3 = *floatdata3;
		*(float *)farray4 = *floatdata4;

			
		u8Arry[15] = farray1[0];
		u8Arry[14] = farray1[1];
		u8Arry[13] = farray1[2];
		u8Arry[12] = farray1[3];
	
		u8Arry[11] = farray2[0];
		u8Arry[10] = farray2[1];
		u8Arry[9] = farray2[2];
		u8Arry[8] = farray2[3];		
	
	    u8Arry[7] = farray3[0];
		u8Arry[6] = farray3[1];
		u8Arry[5] = farray3[2];
		u8Arry[4] = farray3[3];		
		
	    u8Arry[3] = farray4[0];
		u8Arry[2] = farray4[1];
		u8Arry[1] = farray4[2];
		u8Arry[0] = farray4[3];
		

		
		u16Arry[0] = u8Arry[1]*256 + u8Arry[0];
		u16Arry[1] = u8Arry[3]*256 + u8Arry[2];
		
		u16Arry[2] = u8Arry[5]*256 + u8Arry[4];
		u16Arry[3] = u8Arry[7]*256 + u8Arry[6];
				
		u16Arry[4] = u8Arry[9]*256 + u8Arry[8];
		u16Arry[5] = u8Arry[11]*256 + u8Arry[10];
		
		u16Arry[6] = u8Arry[13]*256 + u8Arry[12];
		u16Arry[7] = u8Arry[15]*256 + u8Arry[14];
		
		STMFLASH_Write(FLASH_SAVE_ADDR, u16Arry, 8);
		

}		


void change_u16Arry_float(uint16_t *u16Arry, float *delta_dis1, float *delta_dis2, float *delta_dis3, float *delta_dis4)
{
	uint8_t u8Arry[20];
	uint8_t dis1[4], dis2[4], dis3[4], dis4[4];
	
	u8Arry[0] = (uint8_t)u16Arry[0];
	u8Arry[1] = (uint8_t)(u16Arry[0] >> 8);
	u8Arry[2] = (uint8_t)u16Arry[1];
	u8Arry[3] = (uint8_t)(u16Arry[1] >> 8);
	
	u8Arry[4] = (uint8_t)u16Arry[2];
	u8Arry[5] = (uint8_t)(u16Arry[2] >> 8);
	u8Arry[6] = (uint8_t)u16Arry[3];
	u8Arry[7] = (uint8_t)(u16Arry[3] >> 8);
	
	u8Arry[8] = (uint8_t)u16Arry[4];
	u8Arry[9] = (uint8_t)(u16Arry[4] >> 8);
	u8Arry[10] = (uint8_t)u16Arry[5];
	u8Arry[11] = (uint8_t)(u16Arry[5] >> 8);
	
	u8Arry[12] = (uint8_t)u16Arry[6];
	u8Arry[13] = (uint8_t)(u16Arry[6] >> 8);
	u8Arry[14] = (uint8_t)u16Arry[7];
	u8Arry[15] = (uint8_t)(u16Arry[7] >> 8);
	
	
	dis4[3] = u8Arry[0];
	dis4[2] = u8Arry[1];
	dis4[1] = u8Arry[2];
	dis4[0] = u8Arry[3];
	
    dis3[3] = u8Arry[4];
	dis3[2] = u8Arry[5];
	dis3[1] = u8Arry[6];
	dis3[0] = u8Arry[7];
	
	dis2[3] = u8Arry[8];
	dis2[2] = u8Arry[9];
	dis2[1] = u8Arry[10];
	dis2[0] = u8Arry[11];
	
    dis1[3] = u8Arry[12];
	dis1[2] = u8Arry[13];
	dis1[1] = u8Arry[14];
	dis1[0] = u8Arry[15];
	
	memcpy(delta_dis1, dis1, 4);
	memcpy(delta_dis2, dis2, 4);
	memcpy(delta_dis3, dis3, 4);
	memcpy(delta_dis4, dis4, 4);

}


/*********************** 滑动窗口滤波函数 **********************************/
float sliding_average_filter1(float distance, uint8_t n)
{
  static int data_num = 0;
  static float buf[20];
  float output = 0;

  if (data_num < n) //不满窗口，先填充
  {
    buf[data_num++] = distance;
    output = distance;
  }
  else
  {
    int i = 0;
    float sum = 0.0;
    memcpy(&buf[0], &buf[1], (n - 1) * 4); //将1之后的数据移到0之后，即移除头部
    buf[n-1] = distance;   //即添加尾部
                
    for (i = 0; i < n; i++)
        sum += buf[i]; 

    output = sum / n;
  }
  return output;
}
float sliding_average_filter2(float distance, uint8_t n)
{
  static int data_num = 0;
  static float buf[20];
  float output = 0;

  if (data_num < n) //不满窗口，先填充
  {
    buf[data_num++] = distance;
    output = distance;
  }
  else
  {
    int i = 0;
    float sum = 0.0;
    memcpy(&buf[0], &buf[1], (n - 1) * 4); //将1之后的数据移到0之后，即移除头部
    buf[n-1] = distance;   //即添加尾部
                
    for (i = 0; i < n; i++)
        sum += buf[i]; 

    output = sum / n;
  }
  return output;
}
float sliding_average_filter3(float distance, uint8_t n)
{
  static int data_num = 0;
  static float buf[20];
  float output = 0;

  if (data_num < n) //不满窗口，先填充
  {
    buf[data_num++] = distance;
    output = distance;
  }
  else
  {
    int i = 0;
    float sum = 0.0;
    memcpy(&buf[0], &buf[1], (n - 1) * 4); //将1之后的数据移到0之后，即移除头部
    buf[n-1] = distance;   //即添加尾部
                
    for (i = 0; i < n; i++)
        sum += buf[i]; 

    output = sum / n;
  }
  return output;
}
float sliding_average_filter4(float distance, uint8_t n)
{
  static int data_num = 0;
  static float buf[20];
  float output = 0;

  if (data_num < n) //不满窗口，先填充
  {
    buf[data_num++] = distance;
    output = distance;
  }
  else
  {
    int i = 0;
    float sum = 0.0;
    memcpy(&buf[0], &buf[1], (n - 1) * 4); //将1之后的数据移到0之后，即移除头部
    buf[n-1] = distance;   //即添加尾部
                
    for (i = 0; i < n; i++)
        sum += buf[i]; 

    output = sum / n;
  }
  return output;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}
