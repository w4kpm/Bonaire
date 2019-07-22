// internal reference voltage calibration  0x1FFF F7BA - 0x1FFF F7BB

// TS_CAL1 TS_CAL2 = 0x1FFF F7B8 - 0x1FFF F7B9 and 0x1FFF F7C2 - 0x1FFF F7C3




/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#define CHPRINTF_USE_FLOAT   TRUE
#include "ch.h"
#include "hal.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
//#include "chprintf.h"
#include "hal_queues.h"
#include <string.h>
#include "stm32f3xx.h"
//#include "font.h"
//#include "fontbig.h"
#include "bme680.h"


static uint8_t txbuf[32];
static uint8_t rxbuf[32];
static uint8_t my_address;
static uint8_t save_address;
static uint8_t baud_rate;
static uint8_t save_baud_rate;
static uint8_t reset =0;

static char text[255];
static char metrics[9][12];

static uint8_t vbuf2[32][128];
static uint8_t vbuf[32][128];
static uint8_t rainHistory[10] = {0};
static float rainRate,lifetimeRain = 0.0;
static  float irradiance;
static  float irradiance2;
static  float irradiance3;
static  int displaymetric;
static  float pt100temp1,pt100temp2,pt100temp3,pt100temp4,pt100temp5;
static  float amps,windspeed,windspeedout,opamp4,snow,snowoutput,windamps,winddir;

static uint8_t oled_current_row;
static uint8_t oled_current_column;


static uint16_t *flash1 = 0x803F000;
static uint16_t *flash2 = 0x803e800;
static uint16_t settings;

#define ADC_GRP1_NUM_CHANNELS   2
#define ADC_GRP2_NUM_CHANNELS   6
#define ADC_GRP1_BUF_DEPTH      1
#define ADC_GRP2_BUF_DEPTH      1
static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
static adcsample_t samples2[ADC_GRP2_NUM_CHANNELS * ADC_GRP2_BUF_DEPTH];
size_t nx = 0, ny = 0;



#define RST 12
#define SPISELECT 11
#define CK 13
#define MISO 14
#define MOSI 15
#define DC 14

struct bme680_dev gas_sensor;







static const SPIConfig std_spicfg3 = {
  NULL,
  NULL,
  GPIOB,                                                        /*port of CS  */
  SPISELECT,                                                /*pin of CS   */
  //SPI_CR1_CPOL|	SPI_CR1_CPHA |		\
  //SPI_CR1_SPE|SPI_CR1_MSTR|SPI_CR1_BR_2,SPI_CR1_BR_1,
  0,
  SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0                    /*CR2 register*/
};







uint32_t checksum()
{
    uint32_t checksum;
    int i;
    int j;
    checksum =0;
    for (i=0;i<32;i++)
	for(j=0;j<128;j++)
	    checksum = checksum+vbuf[i][j];
    return checksum;
}






void spi_write(location,data)
{
  //palClearPad(GPIOB,SPISELECT);
    spiStart(&SPID2,&std_spicfg3);
    spiSelect(&SPID2);
    txbuf[0] = location;
    txbuf[1] = data;
    spiSend(&SPID2,2,&txbuf);
    spiUnselect(&SPID2);
    spiStop(&SPID2);
    //palSetPad(GPIOB,SPISELECT);
}

uint8_t spi_read(location)
{
  //    palClearPad(GPIOB,SPISELECT);
    spiStart(&SPID2,&std_spicfg3);
    spiSelect(&SPID2);
    txbuf[0] = location | 0x80;
    spiSend(&SPID2,1,&txbuf);
    spiReceive(&SPID2,1,&rxbuf);
    spiUnselect(&SPID2);
    spiStop(&SPID2);
    //   palSetPad(GPIOB,SPISELECT);
    return rxbuf[0];
}


void init_spi()
{

  //  palSetPadMode(GPIOB, RST, PAL_MODE_OUTPUT_PUSHPULL);
  //palSetPadMode(GPIOB, DC, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOB, SPISELECT, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPad(GPIOB,SPISELECT);
  //palSetPad(GPIOB,RST);

}










// this should set a timeout of .625 seconds (LSI = 40k / (64 * 1000))

static const WDGConfig wdgcfg = {
  STM32_IWDG_PR_64,
  STM32_IWDG_RL(2000),
  STM32_IWDG_WIN_DISABLED};

// This got redefined in a later version of Chibios for this board
#define GPIOA_PIN0 0



//static uint8_t vbuf[64][128];
//static uint8_t vbuf2[64][128];

static mailbox_t RxMbx;
static mailbox_t RxMbx2;
#define MAILBOX_SIZE 25
static msg_t RxMbxBuff[MAILBOX_SIZE];
static msg_t RxMbxBuff2[MAILBOX_SIZE];
static char current_key;
static  uint16_t step =0;
static  int16_t deg,speed =0;

static char rx_text[32][48];
static char rx_text2[32][48];
static char rx_text3[32][48];
static int rx_queue_pos=0;
static int rx_queue_num=0;
static int8_t heatValue;
static uint8_t res_heat_x;
static uint8_t heatRange;
static double par_g1,par_g2,par_g3,var1,var2,var3,var4,var5,target_temp,amb_temp,res_heat_range,res_heat_val;
static SerialConfig uartCfg =
{
    115200,// bit rate
    0,
    0,
    0,
};


//static SerialConfig uartCfg2 =
//{
//    19200,// bit rate
//    0x1400,
//    0,
//    0,
//};
//

static SerialConfig uartCfg2 =
{
    9600,// bit rate
    0,
    0,
    0,
};



static SerialConfig uartCfg3 =
{
    19200,// bit rate
    0,
    0,
    0,
};


dbg(char *string)
{
    chprintf((BaseSequentialStream*)&SD1,string);
    chThdSleepMilliseconds(100);
}


/*
 * Application entry point.
 */


/* ******************
stolen from http://www.modbustools.com/modbus.html#crc
*/
uint16_t CRC16 (const char *nData, uint16_t wLength)
{
static const uint16_t wCRCTable[] = {
   0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
   0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
   0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
   0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
   0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
   0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
   0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
   0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
   0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
   0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
   0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
   0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
   0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
   0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
   0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
   0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
   0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
   0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
   0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
   0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
   0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
   0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
   0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
   0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
   0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
   0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
   0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
   0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
   0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
   0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
   0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
   0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

char nTemp;
uint16_t wCRCWord = 0xFFFF;

   while (wLength--)
   {
      nTemp = *nData++ ^ wCRCWord;
      wCRCWord >>= 8;
      wCRCWord  ^= wCRCTable[nTemp];
   }
   return wCRCWord;
} // End: CRC16





static THD_WORKING_AREA(waThread3, 512);
static THD_FUNCTION(Thread3, arg) {
  (void)arg;
  int pass = 0;
  msg_t b = 0;

  chRegSetThreadName("serial");
  while(TRUE)
      {
	  
	  b = sdGetTimeout(&SD2,TIME_MS2I(2));


	  if ((b!= Q_TIMEOUT) && (rx_queue_pos < 63))
	      {
		palSetPad(GPIOA,1);
		  chprintf((BaseSequentialStream*)&SD2,"got char: %x\r\n",b);
		  chprintf((BaseSequentialStream*)&SD1,"got char: %x\r\n",b);
		  rx_text[rx_queue_num][rx_queue_pos++]=b;
		  chThdSleepMilliseconds(100);
		  palClearPad(GPIOA,1);
		

		
	      }
	  if ((b == Q_TIMEOUT) && (rx_queue_pos > 0))
	      {

		  rx_text[rx_queue_num][rx_queue_pos] = 0;

		  chMBPostTimeout(&RxMbx,(rx_queue_num<<8)|rx_queue_pos,TIME_INFINITE); // let our mailbox know
		  chMBPostTimeout(&RxMbx2,(rx_queue_num<<8)|rx_queue_pos,TIME_INFINITE); // let our mailbox know
		  rx_queue_pos = 0;
		  // we have a new entry
		  rx_queue_num = (++rx_queue_num)%32;
		  memset(rx_text[rx_queue_num],0,5);
	      }
	  
  
      }

  return MSG_OK;
}

uint8_t decode_pos(char pos)
{
  return pos - 32;
}
uint16_t buildint(char *buf,int pos){
  return *(buf+pos)<<8|*(buf+pos+1);
}


static THD_WORKING_AREA(waThread4, 2048);
static THD_FUNCTION(Thread4, arg) {
    int charnum;
    int index;
    msg_t key;
    int x;
    char *starttxt;
    char text[255];
    char lcltext[32];
    uint8_t command;
    int row;
    int col;
    int len;
    uint16_t error;
    uint16_t code;
    msg_t rxRow;
    msg_t rxPos;
    msg_t response;
    uint8_t skip_next;
    uint16_t reg;
    int16_t value;

    uint16_t pm1_0_atm,
      pm2_5_atm,
      pm10_0_atm,
      pm1_0_cf_1,
      pm2_5_cf_1,
      pm10_0_cf_1,
      p_0_3_um,
      p_0_5_um,
      p_1_0_um,
      p_2_5_um,
      p_5_0_um,
      p_10_0_um;

    
    while (TRUE)
	{
	    error = 0;
	    // the skip is because the way I have it hooked up right now
	    // causes it to read whatever we send.
	    chMBFetchTimeout(&RxMbx,&rxRow,TIME_INFINITE);
	    rxPos = rxRow & 0xFF;
	    rxRow = rxRow >> 8;
	    memcpy(lcltext,rx_text[rxRow],rxPos);
	    // if the message is for us and the CRC matches - otherwise -
	    // ignore.
	    if (buildint(lcltext,0) != 0x424d)
	      continue;
	    if (buildint(lcltext,2) != 0x001c)
	      continue;
	    
	    
	    value = 0;
	    for (x=0;x<rxPos;x++)
	      {
		//chprintf((BaseSequentialStream*)&SD1,"%x ",lcltext[x]);
		if (x<30)
		  value = value + lcltext[x];
	      }
	    if (buildint(lcltext,30) != value)
	      continue;

	    pm1_0_atm = buildint(lcltext,10);
	    pm2_5_atm = buildint(lcltext,12);
	    pm10_0_atm = buildint(lcltext,14);
	    pm1_0_cf_1 = buildint(lcltext,4);
	    pm2_5_cf_1 = buildint(lcltext,6);
	    pm10_0_cf_1 = buildint(lcltext,8);
	    p_0_3_um = buildint(lcltext,16);
	    p_0_5_um = buildint(lcltext,18);
	    p_1_0_um = buildint(lcltext,20);
	    p_2_5_um = buildint(lcltext,22);
	    p_5_0_um = buildint(lcltext,24);
	    p_10_0_um = buildint(lcltext,26);

	    chprintf(&SD1,"A: pm1_0_atm = %d,",pm1_0_atm);
	    chprintf(&SD1,"pm2_5_atm = %d,",pm2_5_atm);
	    chprintf(&SD1,"pm10_0_atm = %d,",pm10_0_atm);
	    chprintf(&SD1,"pm1_0_cf_1 = %d,",pm1_0_cf_1);
	    chprintf(&SD1,"pm2_5_cf_1 = %d,",pm2_5_cf_1);
	    chprintf(&SD1,"pm10_0_cf_1 = %d,",pm10_0_cf_1);
	    chprintf(&SD1,"p_0_3_um = %d,",p_0_3_um);
	    chprintf(&SD1,"p_0_5_um = %d,",p_0_5_um);
	    chprintf(&SD1,"p_1_0_um = %d,",p_1_0_um);
	    chprintf(&SD1,"p_2_5_um = %d,",p_2_5_um);
	    chprintf(&SD1,"p_5_0_um = %d,",p_5_0_um);
	    chprintf(&SD1,"p_10_0_um = %d,\r\n",p_10_0_um);


	      
	    

	}

}

static THD_WORKING_AREA(waThread5, 512);
static THD_FUNCTION(Thread5, arg) {
    msg_t rxRow;
    while (TRUE)
	{
	    // the skip is because the way I have it hooked up right now
	    // causes it to read whatever we send.
	    chMBFetchTimeout(&RxMbx2,&rxRow,TIME_INFINITE);
	    palSetPad(GPIOE,1);
	    chThdSleepMilliseconds(5);
	    palClearPad(GPIOE,1);
	}
}



uint8_t set_required_settings;



void adcSTM32EnableTSVREFE(void) {

  ADC12_COMMON->CCR |= ADC12_CCR_VREFEN;
  ADC34_COMMON->CCR |= ADC34_CCR_VREFEN;
}



void feedWatchdog(){
  return;
    if (!reset)
	wdgReset(&WDGD1);
}


void restart_modbus(){
    sdStop(&SD2);
    if (baud_rate == 1)
	sdStart(&SD2, &uartCfg3);
    else
	sdStart(&SD2, &uartCfg2);
}



void user_delay_ms(uint32_t period)
{

  feedWatchdog();
  chThdSleepMilliseconds(period);
  
}


int8_t user_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */
    //chprintf((BaseSequentialStream*)&SD1,"User read %X, %d\r\n",reg_addr,len);
    user_delay_ms(100);
    spiStart(&SPID2,&std_spicfg3);
    spiSelect(&SPID2);
    txbuf[0] = reg_addr;
    spiSend(&SPID2,1,&txbuf);
    spiReceive(&SPID2,len,reg_data);
    spiUnselect(&SPID2);
    spiStop(&SPID2);

    //chprintf((BaseSequentialStream*)&SD1,"DID User read %X, %X %X %X \r\n",reg_data[0],reg_data[1],reg_data[2],reg_data[3]);
    user_delay_ms(100);

    
    /*
     * Data on the bus should be like
     * |----------------+---------------------+-------------|
     * | MOSI           | MISO                | Chip Select |
     * |----------------+---------------------|-------------|
     * | (don't care)   | (don't care)        | HIGH        |
     * | (reg_addr)     | (don't care)        | LOW         |
     * | (don't care)   | (reg_data[0])       | LOW         |
     * | (....)         | (....)              | LOW         |
     * | (don't care)   | (reg_data[len - 1]) | LOW         |
     * | (don't care)   | (don't care)        | HIGH        |
     * |----------------+---------------------|-------------|
     */

    return rslt;
}

int8_t user_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */
    //chprintf((BaseSequentialStream*)&SD1,"User write %X, %d\r\n",reg_addr,len);
    user_delay_ms(100);
    spiStart(&SPID2,&std_spicfg3);
    spiSelect(&SPID2);
    txbuf[0] = reg_addr;
    memcpy(txbuf+1,reg_data,len);
      //txbuf[1] = data;
    spiSend(&SPID2,len+1,&txbuf);
    spiUnselect(&SPID2);
    spiStop(&SPID2);


    
    /*
     * Data on the bus should be like
     * |---------------------+--------------+-------------|
     * | MOSI                | MISO         | Chip Select |
     * |---------------------+--------------|-------------|
     * | (don't care)        | (don't care) | HIGH        |
     * | (reg_addr)          | (don't care) | LOW         |
     * | (reg_data[0])       | (don't care) | LOW         |
     * | (....)              | (....)       | LOW         |
     * | (reg_data[len - 1]) | (don't care) | LOW         |
     * | (don't care)        | (don't care) | HIGH        |
     * |---------------------+--------------|-------------|
     */

    return rslt;
}


   struct bme680_field_data data;

int main(void) {
  unsigned i;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
 

  
  palSetPad(GPIOB, 5);
  //wdgStart(&WDGD1, &wdgcfg);
  //wdgReset(&WDGD1);
  chMBObjectInit(&RxMbx,&RxMbxBuff,MAILBOX_SIZE);
  chMBObjectInit(&RxMbx2,&RxMbxBuff2,MAILBOX_SIZE);
  adcStart(&ADCD1, NULL);
  //  adcStart(&ADCD4, NULL);
  // I think this needs to go after the start - even though it worked fine before

  // I had a problem in another context
  adcSTM32EnableTSVREFE();
  adcSTM32EnableTS(&ADCD1);
  



  /*
   * SPI1 I/O pins setup.
   */



  
  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(7));  //USART1 - console  
  palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 1, PAL_MODE_OUTPUT_PUSHPULL); // tx/rx
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));  //USART2 - MODBUS
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  palSetPadMode(GPIOD, 8, PAL_MODE_ALTERNATE(7));  //USART3 - particle sensor
  palSetPadMode(GPIOD, 9, PAL_MODE_ALTERNATE(7));

  
  palSetPadMode(GPIOB, 11, PAL_MODE_OUTPUT_PUSHPULL);      // spi2

  palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(5)|PAL_STM32_OSPEED_HIGHEST);


  
  sdStart(&SD1,&uartCfg);
  sdStart(&SD3,&uartCfg2);

  



      



  restart_modbus();
  chprintf((BaseSequentialStream*)&SD1,"Hello World - I am # %d,%d\r\n",my_address,baud_rate);
  palSetPad(GPIOA, 1);     // Recieve Enable RS485
  chprintf((BaseSequentialStream*)&SD2,"modbuschannel\r\n");

  init_spi();
  chprintf((BaseSequentialStream*)&SD1,"SPI init\r\n");
  


  
  feedWatchdog();
  chThdSleepMilliseconds(1000);
  palClearPad(GPIOA, 1);     // Recieve Enable RS485
  palClearPad(GPIOE, 0);     // Disable TX Light
  palClearPad(GPIOE, 1);     // Disable RX Light
  feedWatchdog();
  chThdSleepMilliseconds(1000);
  feedWatchdog();



  chprintf((BaseSequentialStream*)&SD1,"HelloA\r\n")  ;
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, Thread3, NULL);
  chprintf((BaseSequentialStream*)&SD1,"HelloB\r\n")  ;
  chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO, Thread4, NULL);
  chprintf((BaseSequentialStream*)&SD1,"HelloC\r\n")  ;
  chThdCreateStatic(waThread5, sizeof(waThread5), NORMALPRIO, Thread5, NULL);
;  
  uint32_t x,y;
  float VDD;
  float outsideTemp;
  float internalTemp;
  

  //Default OPAMP4 CSR 10880000
  
  gas_sensor.dev_id = 0;
  gas_sensor.intf = BME680_SPI_INTF;
  gas_sensor.read = user_spi_read;
  gas_sensor.write = user_spi_write;
  gas_sensor.delay_ms = user_delay_ms;
  gas_sensor.amb_temp = 25;
  int8_t rslt = BME680_OK;

rslt = bme680_init(&gas_sensor);

    gas_sensor.tph_sett.os_hum = BME680_OS_2X;
    gas_sensor.tph_sett.os_pres = BME680_OS_4X;
    gas_sensor.tph_sett.os_temp = BME680_OS_8X;
    gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

    /* Set the remaining gas sensor settings and link the heating profile */
    gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    /* Create a ramp heat waveform in 3 steps */
    gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
    gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */

    /* Select the power mode */
    /* Must be set before writing the sensor configuration */


  
  



    while (TRUE)
      {
	  feedWatchdog();

	  step = (step +1)%288;
	 
	  feedWatchdog();


	  // datasheet RM0316 VDDA = 3.3 V ₓ VREFINT_CAL / VREFINT_DATA
	  
	  //chprintf(&SD1,"calibrated at 3.3 %d\r\n",*(uint16_t*)0x1FFFF7BA);
	  //chprintf((BaseSequentialStream*)&SD1,"ADC4 %d %d %d %d %d\r\n",samples2[0],samples2[1],samples2[2],samples2[3],samples2[4]);
	  chThdSleepMilliseconds(1000);
    gas_sensor.power_mode = BME680_FORCED_MODE; 

    /* Set the required sensor settings needed */
    set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL 
        | BME680_GAS_SENSOR_SEL;

	  /* Set the desired sensor configuration */
    rslt = bme680_set_sensor_settings(set_required_settings,&gas_sensor);

    /* Set the power mode */
    rslt = bme680_set_sensor_mode(&gas_sensor);
	  
	  rslt = bme680_get_sensor_data(&data, &gas_sensor);
	  chprintf(&SD1,"T: %.2f degC, P: %.2f hPa, H %.2f %%rH ", data.temperature / 100.0f,
            data.pressure / 100.0f, data.humidity / 1000.0f );
        /* Avoid using measurements from an unstable heating setup */
        if(data.status & BME680_GASM_VALID_MSK)
	  chprintf(&SD1,", G: %d ohms", data.gas_resistance);
	chprintf(&SD1,"\r\n");
	
       }

  return 0;
}
