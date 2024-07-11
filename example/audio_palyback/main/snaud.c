#include "snaud.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include <math.h>
#include "driver/i2c.h"


#define	NO	0
#define	YES	1
/* Weakly Drive mode define = 0, use headphone op to weakly drive RHP LHP , AC COUPLED VDD_SPK_5V = 1mA */
/* Weakly Drive mode define = 0, use headphone op to weakly drive RHP LHP, VCOM BUF turn on , CAPLESS VDD_SPK_5V = 1.4mA */
/* Weakly Drive mode define = 1, use vrefspk resistor to weakly drive RHP LHP , AC COUPLED VDD_SPK_5V = 70uA */
/* Weakly Drive mode define = 1, use vrefspk resistor to weakly drive RHP LHP VCOM , CAPLESS VDD_SPK_5V = 70uA */
#define	AUD02_WEAKLY_DRIVE_MODE	1
#define	AUD02_ID	0x19    // 0X32 << 1
#define SNAUD_PRINTF_REG(str,reg,rdata)               printf("[SNAUD02] [REG:%s] Reg_Addr=0x%x Rdata=0x%x \r\n",str, reg, rdata);





/**********************I2C function*********************************/
#define SDA_PIN GPIO_NUM_4
#define SCL_PIN GPIO_NUM_5

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ 

i2c_config_t  i2c_config = {
            .mode = I2C_MODE_MASTER,//主机模式
            .sda_io_num = SDA_PIN,//sda i引脚编号
            .scl_io_num = SCL_PIN,//scl 引脚编号
            .sda_pullup_en = GPIO_PULLUP_ENABLE,//上拉使能
            .scl_pullup_en = GPIO_PULLUP_ENABLE,//上拉使能
            .master.clk_speed = 100000 // 100k
    };


void I2C_Mater_Init(void)
{
    i2c_param_config(I2C_NUM_0,&i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}


 //esp_err_t I2C_Wirte(i2c_port_t i2c_num,unit_8 WR_addr,uint8_t *pBuffer,size_t size)
 esp_err_t I2C_Wirte(i2c_port_t i2c_num,uint8_t *wr_data,size_t size)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // 在执行i2c之前，必须执行此函数 创建一个i2c 命令 链接，为之后的i2c操作执行，在执行完成之后需要销毁

    i2c_master_start(cmd);//i2c运行开始函数。注意这不是真的开始，只是是一个开始标记，初始化cmd 

    i2c_master_write_byte(cmd, AUD02_ID << 1 | WRITE_BIT, 0x01);// 向AUD02写入操作，等待从机返回数据
    i2c_master_write(cmd,wr_data,size,0x01); // 写入数据
    i2c_master_stop(cmd);//i2c停止运行。并不是真正的停止，因为此时i2c还没有真正的运行，我认为这是一个标识，当时i2c运行的时候读取到此标志就停止运行。
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS); //按照cmd中的记录的操作顺序开始运行i2c （start-> write AUD02 地址 -> 写入寄存器地址  -> 写入数据 -> stop）
    i2c_cmd_link_delete(cmd); // 操作完成 删除cmd
    return ret;
}


esp_err_t I2C_Master_Read(i2c_port_t i2c_num,uint8_t reg_addr,uint8_t *rd_Data,size_t size) 
{
    int ret;
    if(size == 0)
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); 
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AUD02_ID << 1 | WRITE_BIT, 0x01);// 向AUD02写入操作，等待从机返回数据
    i2c_master_write_byte(cmd,reg_addr, 0x01);
    //ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AUD02_ID << 1 | READ_BIT, 0x01);// 向AUD02写入操作，等待从机返回数据
    i2c_master_read_byte(cmd, rd_Data,0x01);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
	//printf("data = 0x%x",*rd_Data);
    i2c_cmd_link_delete(cmd);
    return ESP_OK;
}



/**********************aud02*********************************/

typedef enum {
	eAUD02_REG_SYS_CLK_CTRL			= 0x00,
	eAUD02_REG_FPLL_CTRL			= 0x01,
	eAUD02_REG_LDO_CTRL				= 0x02,
	eAUD02_REG_PMU_CTRL				= 0x03,
	eAUD02_REG_FPLL_REG0			= 0x04,
	eAUD02_REG_FPLL_REG1			= 0x05,
	eAUD02_REG_FPLL_REG2			= 0x06,
	eAUD02_REG_FPLL_INT				= 0x07,
	eAUD02_REG_I2S_ADC_CTRL0		= 0x08,
	eAUD02_REG_I2S_ADC_CTRL1		= 0x09,
	eAUD02_REG_I2S_DAC_CTRL0		= 0x0A,
	eAUD02_REG_I2S_DAC_CTRL1		= 0x0B,
	eAUD02_REG_I2C_ALTET_ADDR		= 0x0F,
	eAUD02_REG_ADC_MIC_CTRL			= 0x10,
	eAUD02_REG_ADC_CTRL0			= 0x11,
	eAUD02_REG_ADC_CTRL1			= 0x12,
	eAUD02_REG_ADC_CTRL2			= 0x13,
	eAUD02_REG_ADC_ZCU_CTRL		= 0x14,
	eAUD02_REG_ADC_PGA_L			= 0x17,
	eAUD02_REG_ADC_FILTER_L		= 0x18,
	eAUD02_REG_AMP_CTRL				= 0x40,
	eAUD02_REG_AMP_CTRL1      = 0x41,   //aud02
	eAUD02_REG_AMP_CTRL2      = 0x42,   //aud02

} AUD02_REGISTER;

typedef enum {
	eAUD02_ADC = 0,
	eAUD02_DAC
} AUD02_CODEC;

i2c_id_t eI2cNo;
/*ADC*/
static AUD02_MODE eAdcMode;
static AUD02_CLOCK_SOURCE eAdcClockSource;
/*DAC*/
static AUD02_MODE eDacMode;
static AUD02_CLOCK_SOURCE eDacClockSource;

static AUD02_ABILITY bWeaklyDrive = eAUD02_ENABLE;


//Prototype: 	void Write_Register(AUD02_REGISTER eRegister, unsigned short Data)
//Header File:	
//Arguments：eRegister: AUD02_REGISTER enum-type，Write AUD02 register address
//			 data:      send the data
//Return：		None
//Description：	Send data to the AUD02 register using I2C
//Notes：       None
//Example: 
static void Write_Register(AUD02_REGISTER eRegister, unsigned short Data)
{
	unsigned char RegisterAndData[2]; 
	RegisterAndData[0]= (unsigned char)eRegister;
    RegisterAndData[1]= (unsigned char)Data;

     I2C_Wirte(I2C_NUM_0,RegisterAndData,2);
	//I2C_Master_Send(AUD02_ID, RegisterAndData[0],&RegisterAndData[1]);

}

//Prototype: 	void Read_Register(AUD02_REGISTER eRegister, unsigned short* Data)
//Header File:	
//Arguments：eRegister: AUD02_REGISTER enum-type，Read AUD02 register address
//			 Data:    The read data is stored at this address   
//Return：		None
//Description：	Read AUD02 register data  using I2C
//Notes：       None
//Example: 
static void Read_Register(AUD02_REGISTER eRegister, unsigned short* Data)
{
	I2C_Master_Read(I2C_NUM_0, eRegister,(unsigned char*)Data, 1);
	//I2C_Master_Recv(AUD02_ID, &eRegister,(unsigned char*)Data, 1);
}


//Prototype: 	AUD02_ERROR_CODE System_Power_Up(AUD02_MODE	eMode,AUD02_CLOCK_SOURCE eClockSource,AUD02_CODEC eCodec,AUD02_SAMPLING_RATE eSamplingRate)
//Header File:	mid_AUD02.h
//Arguments：eMode: 0:eAUD02_SLAVE 
//				    1:eAUD02_MASTER
//		    eClockSource: 0:eAUD02_EXTERNAL_SYS_CLK
//                        1:eAUD02_INTERNAL_PLL_OUT
//			eCodec: 0:eAUD02_ADC
//                  1:eAUD02_DAC				
//          eSamplingRate: 	eAUD02_0Hz			= 0U,
//							eAUD02_8000Hz		= 8000U,
//							eAUD02_11025Hz		= 11025U,
//							eAUD02_12000Hz		= 12000U,
//							eAUD02_16000Hz		= 16000U,
//							eAUD02_22050Hz		= 22050U,
//							eAUD02_24000Hz		= 24000U,
//							eAUD02_32000Hz		= 32000U,
//							eAUD02_44100Hz		= 44100U,
//							eAUD02_48000Hz		= 48000U,
//							eAUD02_64000Hz		= 64000U,
//							eAUD02_88200Hz		= 88200U,
//							eAUD02_96000Hz		= 96000U	 
//Return：		AUD02_ERROR_CODE: 0:eAUD02_ERROR_CODE_SUCCESS 
//								  1: eAUD02_ERROR_CODE_I2C_ERROR
//								  2：eAUD02_ERROR_CODE_UNSUPPORTED_SAMPLING_RATE
//								  3：eAUD02_ERROR_CODE_AUD02_IS_VERSION
//Description：	ENABLE AND SET AUD02
//Notes：       MCLK = 12M是 eSamplingRate才准确，如果MCLK!=12M,则需要配置分频寄存器。
//Example: 
static AUD02_ERROR_CODE System_Power_Up(
	AUD02_MODE				eMode,
	AUD02_CLOCK_SOURCE		eClockSource,
	AUD02_CODEC				eCodec,
	AUD02_SAMPLING_RATE		eSamplingRate)
{
	unsigned short wdata, rdata;
	AUD02_REGISTER I2S_CTRL0, I2S_CTRL1;

	/*MCLK	* INT.FRA	/ VCO	/ BCLK	/ I2S bits*/
	/*12M	* 9.216		/ 10	/ 12		/ 64  =  16k*/

	/* Only version B has FPLL registers, use to identy which (AUD02) version is */
	rdata = 0;
	Read_Register(eAUD02_REG_FPLL_REG0, &rdata);
//	if (!rdata)
//		return eAUD02_ERROR_CODE_AUD02_IS_VERSION_A;
	
	switch(eSamplingRate)
	{
		case eAUD02_44100Hz:
			Write_Register(eAUD02_REG_FPLL_INT, 0x07);/*10M*/
			Write_Register(eAUD02_REG_FPLL_REG0, 0x0e);
			Write_Register(eAUD02_REG_FPLL_REG1, 0x72);
			Write_Register(eAUD02_REG_FPLL_REG2, 0xb0);
			break;
		case eAUD02_48000Hz:
		case eAUD02_24000Hz:
    	case eAUD02_22050Hz:
		case eAUD02_12000Hz:
		case eAUD02_11025Hz:
			/* 9 + 584406/1048576 = 9.557 */
			Write_Register(eAUD02_REG_FPLL_REG0, 0x08);/*0x08EAD6 = 584406*/
			Write_Register(eAUD02_REG_FPLL_REG1, 0xEA);
			Write_Register(eAUD02_REG_FPLL_REG2, 0xD6);
			break;
		case eAUD02_64000Hz:
		case eAUD02_32000Hz:
		case eAUD02_16000Hz:
		case eAUD02_8000Hz:
			/* 9 + 226492/1048576 = 9.216 */
			Write_Register(eAUD02_REG_FPLL_REG0, 0x03);/*0x0374BC = 226492*/
			Write_Register(eAUD02_REG_FPLL_REG1, 0x74);
			Write_Register(eAUD02_REG_FPLL_REG2, 0xBC);
			break;
		default:
			return eAUD02_ERROR_CODE_UNSUPPORTED_SAMPLING_RATE;
	}

	if (eClockSource == eAUD02_INTERNAL_PLL_OUT) // Enable FPLL and Low pass filter (1.8v)  and SET FPLL VCO
	{
		switch(eSamplingRate)
		{
			case eAUD02_64000Hz:
			case eAUD02_32000Hz:
			case eAUD02_22050Hz:
			case eAUD02_16000Hz:
			case eAUD02_11025Hz:
			case eAUD02_8000Hz:
				/* FPLL VCO / 9 */
				Write_Register(eAUD02_REG_SYS_CLK_CTRL, 0x88);
				break;

			case eAUD02_12000Hz:
				/* FPLL VCO / 10 */
				Write_Register(eAUD02_REG_SYS_CLK_CTRL, 0x98);
				break;
			
      		case eAUD02_48000Hz:
			case eAUD02_44100Hz:
			case eAUD02_24000Hz:
				/* FPLL VCO / 14 */
				Write_Register(eAUD02_REG_SYS_CLK_CTRL, 0xA8);
				break;

			default:
				return eAUD02_ERROR_CODE_UNSUPPORTED_SAMPLING_RATE;
		}

		/* wait until FPLL ready */
		rdata = 0;
		while ((rdata&0x01) != 0x01)
		{
			Read_Register(eAUD02_REG_SYS_CLK_CTRL, &rdata);	
		}
		Write_Register(eAUD02_REG_FPLL_CTRL, 0x46);//VCO gain tuning and PFD pulse delay
	}
	else //if (eClockSource == eAUD02_EXTERNAL_SYS_CLK)
	{
		Write_Register(eAUD02_REG_SYS_CLK_CTRL, 0x00);
	}
	
	Write_Register(eAUD02_REG_LDO_CTRL, 0x04);//LDO18 bandgap turn on 

	if (eCodec == eAUD02_ADC)
	{
		I2S_CTRL0 = eAUD02_REG_I2S_ADC_CTRL0;
		I2S_CTRL1 = eAUD02_REG_I2S_ADC_CTRL1;
	}
	else //if (eCodec == eAUD02_DAC)
	{
		I2S_CTRL0 = eAUD02_REG_I2S_DAC_CTRL0;
		I2S_CTRL1 = eAUD02_REG_I2S_DAC_CTRL1;
	}

	wdata = 0;

	if (eClockSource == eAUD02_INTERNAL_PLL_OUT)
	{
		wdata |= SET_BIT1;
	}
	else //if (eClockSource == eAUD02_EXTERNAL_SYS_CLK)
	{
		wdata &= ~(SET_BIT1);
	}

	if (eMode == eAUD02_MASTER)
	{
		wdata |= SET_BIT0;
	}
	else //if (eMode == eAUD02_SLAVE)
	{
		wdata &= ~(SET_BIT0);
	}
	Write_Register(I2S_CTRL0, wdata);
	switch(eSamplingRate)
	{
		case eAUD02_64000Hz:
		case eAUD02_48000Hz:
		case eAUD02_44100Hz:
			/* BCLKDIV = 2 */
			Write_Register(I2S_CTRL1, 0x00);
			break;
		case eAUD02_32000Hz:
		case eAUD02_24000Hz:
			/* BCLKDIV = 4 */
			Write_Register(I2S_CTRL1, 0x01);
			break;

		case eAUD02_22050Hz:
			/* BCLKDIV = 6 */
			Write_Register(I2S_CTRL1, 0x02);
			break;
		
		case eAUD02_16000Hz:
			/* BCLKDIV = 8 */
			Write_Register(I2S_CTRL1, 0x03);
			break;
		case eAUD02_12000Hz:
			/* BCLKDIV = 10 */
			Write_Register(I2S_CTRL1, 0x04);
			break;

		case eAUD02_11025Hz:
			/* BCLKDIV = 12 */
			 Write_Register(I2S_CTRL1, 0x05);
			break;

		case eAUD02_8000Hz:
			/* BCLKDIV = 16 */
			Write_Register(I2S_CTRL1, 0x07);
			break;

		default:
			return eAUD02_ERROR_CODE_UNSUPPORTED_SAMPLING_RATE;
	}
	
	return eAUD02_ERROR_CODE_SUCCESS;
}

//Prototype: 	AUD02_ERROR_CODE DAC_Power_Up(AUD02_EXTERNAL_CIRCUIT	eExternalCircuit,AUD02_HEADPHONE_DEPOP eHeadphoneDepop,unsigned short ChargeDelayMs,AUD02_ABILITY bAmplifier,AUD02_ABILITY bWeaklyDrive)
//Header File:	mid_AUD02.h
//Arguments：   eExternalCircuit:
//								0:eAUD02_AC_COUPLED
//								1:eAUD02_CAPLESS
//		       eHeadphoneDepop: 
//                             0~8: eAUD02_DEPOP_0 ~eAUD02_DEPOP_8
//			   ChargeDelayMs: 
//                  			
//             bAmplifier: 	0:eAUD02_DISABLE 
//							1:eAUD02_ENABLE
//			   bWeaklyDrive:   0:eAUD02_DISABLE 
//							   1:eAUD02_ENABLE
//Return：	   AUD02_ERROR_CODE: 0:eAUD02_ERROR_CODE_SUCCESS 
//								  1: eAUD02_ERROR_CODE_I2C_ERROR
//								  2：eAUD02_ERROR_CODE_UNSUPPORTED_SAMPLING_RATE
//								  3：eAUD02_ERROR_CODE_AUD02_IS_VERSION
//Description：	ENABLE AND SET AUD02_ DAC
//Notes：       None
//Example: 
static AUD02_ERROR_CODE DAC_Power_Up(
	AUD02_HEADPHONE_DEPOP	eHeadphoneDepop,
	unsigned short		ChargeDelayMs,
	AUD02_ABILITY			bAmplifier,
	AUD02_ABILITY			bWeaklyDrive)
{
//	unsigned short wdata, rdata;
	unsigned short rdata;
	if (bAmplifier == eAUD02_ENABLE)
	{
		Write_Register(eAUD02_REG_AMP_CTRL, 0x9D);
		/* wait for OPA_RDY*/
		rdata = 0;
		while ((rdata&0x01) != 0x01)
		{
			Read_Register(eAUD02_REG_AMP_CTRL, &rdata);
		}
	}
	
	/**********************************
	**add aud02 
	**********************************/
	Read_Register(eAUD02_REG_LDO_CTRL, &rdata);
	Write_Register(eAUD02_REG_LDO_CTRL,rdata | (0x01<<2));

	return eAUD02_ERROR_CODE_SUCCESS;
}

//Prototype: 	AUD02_ERROR_CODE DAC_Power_Down(AUD02_EXTERNAL_CIRCUIT	eExternalCircuit,AUD02_HEADPHONE_DEPOP eHeadphoneDepop,unsigned short ChargeDelayMs,AUD02_ABILITY bAmplifier,AUD02_ABILITY bWeaklyDrive)
//Header File:	mid_AUD02.h
//Arguments：   none
//Return：	   AUD02_ERROR_CODE: 0:eAUD02_ERROR_CODE_SUCCESS 
//Description：	DISABLE AND CLEAR AUD02_ DAC
//Notes：       None
//Example: 

static AUD02_ERROR_CODE DAC_Power_Down(
	AUD02_ABILITY			bLdo18Bandgap,
	AUD02_ABILITY			bWeaklyDrive,
	AUD02_ABILITY			bAmplifier)
{
//	unsigned short wdata, rdata;
  Write_Register(eAUD02_REG_AMP_CTRL, 0x00);
	Write_Register(eAUD02_REG_SYS_CLK_CTRL, 0x00);
	return eAUD02_ERROR_CODE_SUCCESS;
}


//Prototype: 	AUD02_ERROR_CODE ADC_Power_Up(void)
//Header File:	mid_AUD02.h
//Arguments：	None
//Return：	   AUD02_ERROR_CODE: 0:eAUD02_ERROR_CODE_SUCCESS 
//								
//Description：	ENABLE AND SET AUD02_ ADc
//Notes：       None
//Example: 
static AUD02_ERROR_CODE ADC_Power_Up(void)
{
	unsigned short rdata;
	Write_Register(eAUD02_REG_ADC_MIC_CTRL, 0xD9);
	Write_Register(eAUD02_REG_ADC_CTRL0, 0xE0);    //E0
	Write_Register(eAUD02_REG_ADC_CTRL1, 0xE0);    //E0
	Write_Register(eAUD02_REG_ADC_CTRL2, 0x39);    //30
	Write_Register(eAUD02_REG_ADC_ZCU_CTRL, 0x00);
	Write_Register(eAUD02_REG_ADC_PGA_L, 0x50);    //10
	Write_Register(eAUD02_REG_ADC_FILTER_L, 0xB7);
	Read_Register(eAUD02_REG_LDO_CTRL, &rdata);
  Write_Register(eAUD02_REG_LDO_CTRL, rdata | (0x01<<2));

	return eAUD02_ERROR_CODE_SUCCESS;
}

//Prototype: 	AUD02_ERROR_CODE ADC_Power_Down(void)
//Header File:	mid_AUD02.h
//Arguments：	None
//Return：	   AUD02_ERROR_CODE: 0:eAUD02_ERROR_CODE_SUCCESS 								
//Description：	DISABLE AND SET AUD02_ ADc
//Notes：       None
//Example: 
static AUD02_ERROR_CODE ADC_Power_Down(void)
{
	Write_Register(eAUD02_REG_ADC_MIC_CTRL, 0x00);
	Write_Register(eAUD02_REG_ADC_CTRL0, 0x10);
	Write_Register(eAUD02_REG_ADC_CTRL1, 0x10);
	Write_Register(eAUD02_REG_ADC_PGA_L, 0x00);
	return eAUD02_ERROR_CODE_SUCCESS;
}


static AUD02_ERROR_CODE LDO18_Bandgap_Switch(AUD02_ABILITY bLdo18Bandgap)
{
//	unsigned short rdata;

	return eAUD02_ERROR_CODE_SUCCESS;
}


//Prototype: 	AUD02_ERROR_CODE AUD02_Set_ADC_Channel_Gain(AUD02_CHANNEL eChannel, AUD02_BOOST_GAIN eBoostGain, AUD02_PGA_GAIN ePgaGain)
//Header File:	mid_AUD02.h
//Arguments：	eChannel:	1:eAUD02_LEFT_CHANNEL
//							2:eAUD02_RIGHT_CHANNEL
//Return：	   AUD02_ERROR_CODE: 0:eAUD02_ERROR_CODE_SUCCESS 								
//Description：	SET ADC CHANNEL AND gain(ONLY left CHANNEL)
//Notes：       None
//Example: 
AUD02_ERROR_CODE AUD02_Set_ADC_Channel_Gain(AUD02_CHANNEL eChannel, AUD02_BOOST_GAIN eBoostGain, AUD02_PGA_GAIN ePgaGain)
{
	switch(eChannel)
	{
		case eAUD02_LEFT_CHANNEL:
			Write_Register(eAUD02_REG_ADC_PGA_L, (unsigned short)(eBoostGain<<6)+(unsigned short)ePgaGain);
			break;

		case eAUD02_RIGHT_CHANNEL:
			break;
	}
		
	return eAUD02_ERROR_CODE_SUCCESS;
}


//Prototype: 	AUD02_ERROR_CODE AUD02_ADC_Init(AUD02_STRUCT* pAUD02Struct)
//Header File:	mid_AUD02.h
//Arguments：	pAUD02Struct:AUD02_STRUCT TYPE
//Return：	   AUD02_ERROR_CODE: 0:eAUD02_ERROR_CODE_SUCCESS 	
//								  1: eAUD02_ERROR_CODE_I2C_ERROR
//								  2：eAUD02_ERROR_CODE_UNSUPPORTED_SAMPLING_RATE
//								  3：eAUD02_ERROR_CODE_AUD02_IS_VERSION							
//Description：	ENABLE AND SET AUD02 AND set Adc
//Notes：       None
//Example: 

AUD02_ERROR_CODE AUD02_ADC_Init(AUD02_STRUCT* pAUD02Struct)
{
	AUD02_ERROR_CODE eErrorCode;
	
	//eI2cNo = pAUD02Struct->eI2cNo;
	eAdcMode = pAUD02Struct->eMode;
	eAdcClockSource = pAUD02Struct->eClockSource;
	
	if ((eErrorCode = System_Power_Up(pAUD02Struct->eMode, pAUD02Struct->eClockSource, eAUD02_ADC, pAUD02Struct->eSamplingRate)) != eAUD02_ERROR_CODE_SUCCESS)
		return eErrorCode;
	
	ADC_Power_Up();
	AUD02_Set_ADC_Channel_Gain(eAUD02_LEFT_CHANNEL, pAUD02Struct->ADC.eBoostGainL, pAUD02Struct->ADC.ePgaGainL);

	
	return eAUD02_ERROR_CODE_SUCCESS;
}

//Prototype: 	AUD02_ERROR_CODE AUD02_ADC_DeInit(void)
//Header File:	mid_AUD02.h
//Arguments：	NONE
//Return：	   AUD02_ERROR_CODE: 0:eAUD02_ERROR_CODE_SUCCESS 	
//								  1: eAUD02_ERROR_CODE_I2C_ERROR
//								  2：eAUD02_ERROR_CODE_UNSUPPORTED_SAMPLING_RATE
//								  3：eAUD02_ERROR_CODE_AUD02_IS_VERSION							
//Description：	ADC_Power_Down
//Notes：       None
//Example: 
AUD02_ERROR_CODE AUD02_ADC_DeInit(void)
{
	return ADC_Power_Down();
}

//Prototype: 	AUD02_ERROR_CODE AUD02_Set_ADC_Sampling_Rate(AUD02_SAMPLING_RATE eSamplingRate)
//Header File:	mid_AUD02.h
//Arguments：	NONE
//Return：	   AUD02_ERROR_CODE: 0:eAUD02_ERROR_CODE_SUCCESS 	
//								  1: eAUD02_ERROR_CODE_I2C_ERROR
//								  2：eAUD02_ERROR_CODE_UNSUPPORTED_SAMPLING_RATE
//								  3：eAUD02_ERROR_CODE_AUD02_IS_VERSION							
//Description：SET AUD02 SAMPLE RATE
//Notes：       None
//Example: 
AUD02_ERROR_CODE AUD02_Set_ADC_Sampling_Rate(AUD02_SAMPLING_RATE eSamplingRate)
{
	return System_Power_Up(eAdcMode, eAdcClockSource, eAUD02_ADC, eSamplingRate);
}

//Prototype: 	AUD02_ERROR_CODE AUD02_Set_ADC_Channel_Filter(AUD02_CHANNEL	eChannel,AUD02_ABILITY	bDcCancellation,AUD02_ABILITY bNoiseSuppression,AUD02_ABILITY	bOffsetCancellation,AUD02_HPF_GAIN	eHpfGain)
//Header File:	mid_AUD02.h
//Arguments：	NONE
//Return：	   AUD02_ERROR_CODE: 0:eAUD02_ERROR_CODE_SUCCESS 	
//								 1: eAUD02_ERROR_CODE_I2C_ERROR
//								 2：eAUD02_ERROR_CODE_UNSUPPORTED_SAMPLING_RATE
//								 3：eAUD02_ERROR_CODE_AUD02_IS_VERSION							
//Description：SET AUD02 SAMPLE RATE
//Notes：       None
//Example: 
AUD02_ERROR_CODE AUD02_Set_ADC_Channel_Filter(
	AUD02_CHANNEL	eChannel,
	AUD02_ABILITY	bDcCancellation,
	AUD02_ABILITY	bNoiseSuppression,
	AUD02_ABILITY	bOffsetCancellation,
	AUD02_HPF_GAIN	eHpfGain)
{
	unsigned short wdata;
	unsigned char DC, Noise, Offset;

	DC = (~bDcCancellation)&0x01;
	Noise = (~bNoiseSuppression)&0x01;
	Offset = (~bOffsetCancellation)&0x01;
	
	wdata = (DC<<7) | (Noise<<5) | (Offset<<4) | eHpfGain;

	switch(eChannel)
	{
		case eAUD02_LEFT_CHANNEL:
			Write_Register(eAUD02_REG_ADC_FILTER_L, wdata);
			break;

		case eAUD02_RIGHT_CHANNEL:
			break;
	}
	
	return eAUD02_ERROR_CODE_SUCCESS;
}


AUD02_ERROR_CODE AUD02_Set_AMP_Gain(
	AUD02_AMP_CHANNEL  eAmpChannel,
	AUD02_AMP_OPA_VOL  eAmpOpaVol,
	AUD02_AMP_DATA_OPT eAmpDataOpt)
{
	unsigned short rdata;

	Read_Register(eAUD02_REG_AMP_CTRL, &rdata);
	rdata &=~(1 << 4);
	//Write_Register(eAUD02_REG_AMP_CTRL,0x03);
	Write_Register(eAUD02_REG_AMP_CTRL,0x03|rdata|eAmpDataOpt);
	
	Read_Register(eAUD02_REG_AMP_CTRL1, &rdata);
    rdata &=~(7 << 2);
	Write_Register(eAUD02_REG_AMP_CTRL1,rdata|(eAmpOpaVol<<2));
	
	Read_Register(eAUD02_REG_AMP_CTRL2, &rdata);
	Write_Register(eAUD02_REG_AMP_CTRL2,0x01 |(eAmpChannel<<4));
	
	Read_Register(eAUD02_REG_LDO_CTRL, &rdata);
	Write_Register(eAUD02_REG_LDO_CTRL,rdata|(0x01<<2));
	
	return eAUD02_ERROR_CODE_SUCCESS;
}

AUD02_ERROR_CODE AUD02_DAC_Init(AUD02_STRUCT* pAUD02Struct)
{
	AUD02_ERROR_CODE eErrorCode;
	
//	eI2cNo = pAUD02Struct->eI2cNo;
	eDacMode = pAUD02Struct->eMode;
	eDacClockSource = pAUD02Struct->eClockSource;
	
	if ((eErrorCode = System_Power_Up(pAUD02Struct->eMode, pAUD02Struct->eClockSource, eAUD02_DAC, pAUD02Struct->eSamplingRate)) != eAUD02_ERROR_CODE_SUCCESS)
		return eErrorCode;
	AUD02_Set_AMP_Gain(pAUD02Struct->DAC.eAmpChannel,pAUD02Struct->DAC.eAmpOpaVol,pAUD02Struct->DAC.eAmpDataOpt);
	
	return eAUD02_ERROR_CODE_SUCCESS;
}

//Prototype:	AUD02_ERROR_CODE AUD02_DAC_DeInit(void)
//Header File:	mid_aud02.h
//Arguments:	None
//Return:	AUD02_ERROR_CODE: 
//eAUD02_ERROR_CODE_SUCCESS
//eAUD02_ERROR_CODE_I2C_ERROR
//eAUD02_ERROR_CODE_UNSUPPORTED_SAMPLING_RATE
//eAUD02_ERROR_CODE_AUD02_IS_VERSION	
//Description:	turn off  DAC
//Notes:	None */
AUD02_ERROR_CODE AUD02_DAC_DeInit(void)
{
	AUD02_ERROR_CODE eErrorCode;
	
	if (bWeaklyDrive)
	{
		/* turn on DAC weakly drive only */
		if ((eErrorCode = DAC_Power_Up(eAUD02_DEPOP_8, 1000, eAUD02_DISABLE, eAUD02_ENABLE)) != eAUD02_ERROR_CODE_SUCCESS)
			return eErrorCode;
		if ((eErrorCode = DAC_Power_Down(eAUD02_DISABLE, eAUD02_DISABLE, eAUD02_DISABLE)) != eAUD02_ERROR_CODE_SUCCESS)
			return eErrorCode;
	}
	else
	{
		if ((eErrorCode = DAC_Power_Down(eAUD02_ENABLE, eAUD02_DISABLE, eAUD02_DISABLE)) != eAUD02_ERROR_CODE_SUCCESS)
			return eErrorCode;
	}
	return eAUD02_ERROR_CODE_SUCCESS;
}


AUD02_ERROR_CODE AUD02_Set_DAC_Sampling_Rate(AUD02_SAMPLING_RATE eSamplingRate)
{
	return System_Power_Up(eDacMode, eDacClockSource, eAUD02_DAC, eSamplingRate);
}


AUD02_ERROR_CODE AUD02_Standby_Mode(void)
{

	LDO18_Bandgap_Switch(eAUD02_ENABLE);
	return eAUD02_ERROR_CODE_SUCCESS;
}

AUD02_ERROR_CODE AUD02_List_Reg(void)
{
	unsigned short  rdata;
	rdata = 0;
	Read_Register(eAUD02_REG_SYS_CLK_CTRL, &rdata);
	SNAUD_PRINTF_REG("SYS_CLK_CTRL",eAUD02_REG_SYS_CLK_CTRL,rdata);
	rdata = 0;
//	eAUD02_REG_FPLL_CTRL			= 0x01,
	Read_Register(eAUD02_REG_FPLL_CTRL, &rdata);
	SNAUD_PRINTF_REG("SYS_CLK_CTRL",eAUD02_REG_FPLL_CTRL,rdata);
	rdata = 0;
//	eAUD02_REG_LDO_CTRL				= 0x02,
	Read_Register(eAUD02_REG_LDO_CTRL, &rdata);
	SNAUD_PRINTF_REG("SYS_CLK_CTRL",eAUD02_REG_LDO_CTRL,rdata);
	rdata = 0;
//	eAUD02_REG_PMU_CTRL				= 0x03,
		Read_Register(eAUD02_REG_PMU_CTRL, &rdata);
	SNAUD_PRINTF_REG("SYS_CLK_CTRL",eAUD02_REG_PMU_CTRL,rdata);
	rdata = 0;
//	eAUD02_REG_FPLL_REG0			= 0x04,
	Read_Register(eAUD02_REG_FPLL_REG0, &rdata);
	SNAUD_PRINTF_REG("SYS_CLK_CTRL",eAUD02_REG_FPLL_REG0,rdata);
	rdata = 0;
//	eAUD02_REG_FPLL_REG1			= 0x05,
//	eAUD02_REG_FPLL_REG2			= 0x06,
//	eAUD02_REG_I2S_ADC_CTRL0		= 0x08,
	Read_Register(eAUD02_REG_I2S_ADC_CTRL0, &rdata);
	SNAUD_PRINTF_REG("SYS_CLK_CTRL",eAUD02_REG_I2S_ADC_CTRL0,rdata);
	rdata = 0;	
//	eAUD02_REG_I2S_ADC_CTRL1		= 0x09,
	Read_Register(eAUD02_REG_I2S_ADC_CTRL1, &rdata);
	SNAUD_PRINTF_REG("SYS_CLK_CTRL",eAUD02_REG_I2S_ADC_CTRL1,rdata);	
	rdata = 0;
//	eAUD02_REG_I2S_DAC_CTRL0		= 0x0A,
	Read_Register(eAUD02_REG_I2S_DAC_CTRL0, &rdata);
	SNAUD_PRINTF_REG("SYS_CLK_CTRL",eAUD02_REG_I2S_DAC_CTRL0,rdata);
	rdata = 0;	
//	eAUD02_REG_I2S_DAC_CTRL1		= 0x0B,
	Read_Register(eAUD02_REG_I2S_DAC_CTRL1, &rdata);
	SNAUD_PRINTF_REG("SYS_CLK_CTRL",eAUD02_REG_I2S_DAC_CTRL1,rdata);
	rdata = 0;
//	eAUD02_REG_I2C_ALTET_ADDR		= 0x0F,
	Read_Register(eAUD02_REG_I2C_ALTET_ADDR, &rdata);
	SNAUD_PRINTF_REG("SYS_CLK_CTRL",eAUD02_REG_I2C_ALTET_ADDR,rdata);
	rdata = 0;
//	eAUD02_REG_ADC_MIC_CTRL			= 0x10,
	Read_Register(eAUD02_REG_ADC_MIC_CTRL, &rdata);
	SNAUD_PRINTF_REG("SYS_CLK_CTRL",eAUD02_REG_ADC_MIC_CTRL,rdata);
	rdata = 0;
//	eAUD02_REG_ADC_CTRL0			= 0x11,
	Read_Register(eAUD02_REG_ADC_CTRL0, &rdata);
	SNAUD_PRINTF_REG("SYS_CLK_CTRL",eAUD02_REG_ADC_CTRL0,rdata);
	rdata = 0;
//	eAUD02_REG_ADC_CTRL1			= 0x12,
	Read_Register(eAUD02_REG_ADC_CTRL1, &rdata);
	SNAUD_PRINTF_REG("SYS_CLK_CTRL",eAUD02_REG_ADC_CTRL1,rdata);
	rdata = 0;
//	eAUD02_REG_ADC_CTRL2			= 0x13,
	Read_Register(eAUD02_REG_ADC_CTRL2, &rdata);
	SNAUD_PRINTF_REG("SYS_CLK_CTRL",eAUD02_REG_ADC_CTRL2,rdata);
	rdata = 0;
//	eAUD02_REG_ADC_ZCU_CTRL			= 0x14,
	Read_Register(eAUD02_REG_ADC_CTRL2, &rdata);
	SNAUD_PRINTF_REG("SYS_CLK_CTRL",eAUD02_REG_ADC_CTRL2,rdata);
//	eAUD02_REG_ADC_PGA_R			= 0x15,
//	eAUD02_REG_ADC_FILTER_R			= 0x16,
//	eAUD02_REG_ADC_PGA_L			= 0x17,
//	eAUD02_REG_ADC_FILTER_L			= 0x18,
//	eAUD02_REG_ADC_TEST				= 0x19,
//	eAUD02_REG_DAC_CTRL0			= 0x30,
//	eAUD02_REG_DAC_CTRL1			= 0x31,
//	eAUD02_REG_DAC_PD				= 0x32,
//	eAUD02_REG_DAC_GAIN				= 0x33,
//	eAUD02_REG_DAC_DEEMP			= 0x34,
//	eAUD02_REG_DAC_MUTE				= 0x35,
//	eAUD02_REG_AMP_CTRL				= 0x40,
//	eAUD02_REG_AMP_CTRL1      = 0x41,   //aud02
//	eAUD02_REG_AMP_CTRL2      = 0x42,   //aud02
//	eAUD02_REG_DEBUG_MODE0			= 0x50,
//	eAUD02_REG_DEBUG_MODE1			= 0x51,
//	eAUD02_REG_GPIO_IO_MODE			= 0x52,
//	eAUD02_REG_GPIO_IO_CHG			= 0x53,
//	eAUD02_REG_GPIO_DAT				= 0x54,
//	eAUD02_REG_DAC_ISO0				= 0x60,
//	eAUD02_REG_DAC_ISO1				= 0x61,
//	eAUD02_REG_DAC_ISO2				= 0x62
	
  return eAUD02_ERROR_CODE_SUCCESS;
}

AUD02_ERROR_CODE  AUD02_DAC_Mute (void)
{
	unsigned short rdata;
	Read_Register(eAUD02_REG_AMP_CTRL, &rdata);
	Write_Register(eAUD02_REG_AMP_CTRL,rdata|SET_BIT2|SET_BIT3);
	return eAUD02_ERROR_CODE_SUCCESS;
}

#if 0
void AUD02_Set_FPLL_REG1(unsigned char wdata)
{
	unsigned short rdata;
	Write_Register(eAUD02_REG_FPLL_REG0, (unsigned short)wdata);
	Read_Register(eAUD02_REG_SYS_CLK_CTRL, &rdata);
	Write_Register(eAUD02_REG_SYS_CLK_CTRL, rdata);
}
#endif