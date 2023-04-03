#include "l3gd20.h"
#include "stm32f4xx_hal.h"

static SPI_HandleTypeDef SpiHandle;
uint32_t SpixTimeout = SPIx_TIMEOUT_MAX;    /*<! Value of Timeout when SPI communication fails */

GYRO_DrvTypeDef L3gd20Drv =
{
  L3GD20_Init,
  L3GD20_DeInit,
  L3GD20_ReadID,
  L3GD20_RebootCmd,
  L3GD20_LowPower,
  L3GD20_INT1InterruptConfig,
  L3GD20_EnableIT,
  L3GD20_DisableIT,
  0,
  0,
  L3GD20_FilterConfig,
  L3GD20_FilterCmd,
  L3GD20_ReadXYZAngRate
};

/**
  * @brief  SPI MSP Init.
  * @param  hspi: SPI handle
  */
static void SPIx_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef   GPIO_InitStructure;
  
  /* Enable SPIx clock  */
  DISCOVERY_SPIx_CLOCK_ENABLE();
  
  /* Enable SPIx GPIO clock */
  DISCOVERY_SPIx_GPIO_CLK_ENABLE();
  
  /* Configure SPIx SCK, MOSI and MISO */
  GPIO_InitStructure.Pin = (DISCOVERY_SPIx_SCK_PIN | DISCOVERY_SPIx_MOSI_PIN | DISCOVERY_SPIx_MISO_PIN);
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull  = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
  GPIO_InitStructure.Alternate = DISCOVERY_SPIx_AF;
  HAL_GPIO_Init(DISCOVERY_SPIx_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  SPIx Bus initialization.
  */
static void SPIx_Init(void)
{
  if(HAL_SPI_GetState(&SpiHandle) == HAL_SPI_STATE_RESET)
  {
    /* SPI Configuration */
    SpiHandle.Instance = DISCOVERY_SPIx;
    /* SPI baudrate is set to 5.6 MHz (PCLK2/SPI_BaudRatePrescaler = 90/16 = 5.625 MHz) 
       to verify these constraints:
       ILI9341 LCD SPI interface max baudrate is 10MHz for write and 6.66MHz for read
       L3GD20 SPI interface max baudrate is 10MHz for write/read
       PCLK2 frequency is set to 90 MHz 
      */
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    SpiHandle.Init.Direction = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    SpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    SpiHandle.Init.CRCPolynomial = 7;
    SpiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.NSS = SPI_NSS_SOFT;
    SpiHandle.Init.TIMode = SPI_TIMODE_DISABLED;
    SpiHandle.Init.Mode = SPI_MODE_MASTER;
    
    SPIx_MspInit(&SpiHandle);
    HAL_SPI_Init(&SpiHandle);
  }
}

/**
  * @brief  SPIx error treatment function.
  */
static void SPIx_Error (void)
{
  /* De-initialize the SPI comunication BUS */
  HAL_SPI_DeInit(&SpiHandle);
  
  /* Re-Initiaize the SPI comunication BUS */
  SPIx_Init();
}

/**
  * @brief  Sends a Byte through the SPI interface and return the Byte received 
  *         from the SPI bus.
  * @param  Byte: Byte send.
  * @retval The received byte value
  */
static uint8_t SPIx_WriteRead(uint8_t Byte)
{
  uint8_t receivedbyte = 0;
  
  /* Send a Byte through the SPI peripheral */
  /* Read byte from the SPI bus */
  if(HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) &Byte, (uint8_t*) &receivedbyte, 1, SpixTimeout) != HAL_OK)
  {
    SPIx_Error();
  }
  
  return receivedbyte;
}

/**
  * @brief  Configures the GYRO SPI interface.
  */
void GYRO_IO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Configure the Gyroscope Control pins ------------------------------------*/
  /* Enable CS GPIO clock and  Configure GPIO PIN for Gyroscope Chip select */  
  GYRO_CS_GPIO_CLK_ENABLE();  
  GPIO_InitStructure.Pin = GYRO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(GYRO_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  GYRO_CS_HIGH();

  /* Enable INT1, INT2 GPIO clock and Configure GPIO PINs to detect Interrupts */
  GYRO_INT_GPIO_CLK_ENABLE();
  GPIO_InitStructure.Pin = GYRO_INT1_PIN | GYRO_INT2_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
  GPIO_InitStructure.Pull= GPIO_NOPULL;
  HAL_GPIO_Init(GYRO_INT_GPIO_PORT, &GPIO_InitStructure);
  
  SPIx_Init();
}

/**
  * @brief  Writes one byte to the GYRO.
  * @param  pBuffer: pointer to the buffer  containing the data to be written to the GYRO.
  * @param  WriteAddr : GYRO's internal address to write to.
  * @param  NumByteToWrite: Number of bytes to write.
  */
void GYRO_IO_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  /* Configure the MS bit: 
     - When 0, the address will remain unchanged in multiple read/write commands.
     - When 1, the address will be auto incremented in multiple read/write commands.
  */
  if(NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  GYRO_CS_LOW();
  
  /* Send the Address of the indexed register */
  SPIx_WriteRead(WriteAddr);
  
  /* Send the data that will be written into the device (MSB First) */
  while(NumByteToWrite >= 0x01)
  {
    SPIx_WriteRead(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  GYRO_CS_HIGH();
}

/**
  * @brief  Reads a block of data from the GYRO.
  * @param  pBuffer: pointer to the buffer that receives the data read from the GYRO.
  * @param  ReadAddr: GYRO's internal address to read from.
  * @param  NumByteToRead: Number of bytes to read from the GYRO.
  */
void GYRO_IO_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{  
  if(NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
    ReadAddr |= (uint8_t)READWRITE_CMD;
  }
  
  /* Set chip select Low at the start of the transmission */
  GYRO_CS_LOW();
  
  /* Send the Address of the indexed register */
  SPIx_WriteRead(ReadAddr);
  
  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to GYRO (Slave device) */
    *pBuffer = SPIx_WriteRead(DUMMY_BYTE);
    NumByteToRead--;
    pBuffer++;
  }
  
  /* Set chip select High at the end of the transmission */ 
  GYRO_CS_HIGH();
}  

/**
  * @brief  Set L3GD20 Initialization.
  * @param  L3GD20_InitStruct: pointer to a L3GD20_InitTypeDef structure 
  *         that contains the configuration setting for the L3GD20.
  * @retval None
  */
void L3GD20_Init(uint16_t InitStruct)
{  
  uint8_t ctrl = 0x00;
  
  /* Configure the low level interface */
  GYRO_IO_Init();
  
  /* Write value to MEMS CTRL_REG1 register */
  ctrl = (uint8_t) InitStruct;
  GYRO_IO_Write(&ctrl, L3GD20_CTRL_REG1_ADDR, 1);
  
  /* Write value to MEMS CTRL_REG4 register */  
  ctrl = (uint8_t) (InitStruct >> 8);
  GYRO_IO_Write(&ctrl, L3GD20_CTRL_REG4_ADDR, 1);
}

/**
  * @brief L3GD20 De-initialization
  * @param  None
  * @retval None
  */
void L3GD20_DeInit(void)
{
}

/**
  * @brief  Read ID address of L3GD20
  * @param  None
  * @retval ID name
  */
uint8_t L3GD20_ReadID(void)
{
  uint8_t tmp;
  
  /* Configure the low level interface */
  GYRO_IO_Init();
  
  /* Read WHO I AM register */
  GYRO_IO_Read(&tmp, L3GD20_WHO_AM_I_ADDR, 1);
  
  /* Return the ID */
  return (uint8_t)tmp;
}

/**
  * @brief  Reboot memory content of L3GD20
  * @param  None
  * @retval None
  */
void L3GD20_RebootCmd(void)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG5 register */
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
  
  /* Enable or Disable the reboot memory */
  tmpreg |= L3GD20_BOOT_REBOOTMEMORY;
  
  /* Write value to MEMS CTRL_REG5 register */
  GYRO_IO_Write(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
}

/**
  * @brief Set L3GD20 in low-power mode
  * @param 
  * @retval  None
  */
void L3GD20_LowPower(uint16_t InitStruct)
{  
  uint8_t ctrl = 0x00;

  /* Write value to MEMS CTRL_REG1 register */
  ctrl = (uint8_t) InitStruct;
  GYRO_IO_Write(&ctrl, L3GD20_CTRL_REG1_ADDR, 1);
}

/**
  * @brief  Set L3GD20 Interrupt INT1 configuration
  * @param  Int1Config: the configuration setting for the L3GD20 Interrupt.
  * @retval None
  */
void L3GD20_INT1InterruptConfig(uint16_t Int1Config)
{
  uint8_t ctrl_cfr = 0x00, ctrl3 = 0x00;
  
  /* Read INT1_CFG register */
  GYRO_IO_Read(&ctrl_cfr, L3GD20_INT1_CFG_ADDR, 1);
  
  /* Read CTRL_REG3 register */
  GYRO_IO_Read(&ctrl3, L3GD20_CTRL_REG3_ADDR, 1);
  
  ctrl_cfr &= 0x80;
  ctrl_cfr |= ((uint8_t) Int1Config >> 8);
  
  ctrl3 &= 0xDF;
  ctrl3 |= ((uint8_t) Int1Config);   
  
  /* Write value to MEMS INT1_CFG register */
  GYRO_IO_Write(&ctrl_cfr, L3GD20_INT1_CFG_ADDR, 1);
  
  /* Write value to MEMS CTRL_REG3 register */
  GYRO_IO_Write(&ctrl3, L3GD20_CTRL_REG3_ADDR, 1);
}

/**
  * @brief  Enable INT1 or INT2 interrupt
  * @param  IntSel: choice of INT1 or INT2 
  *      This parameter can be: 
  *        @arg L3GD20_INT1
  *        @arg L3GD20_INT2   
  * @retval None
  */
void L3GD20_EnableIT(uint8_t IntSel)
{  
  uint8_t tmpreg;
  
  /* Read CTRL_REG3 register */
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);
  
  if(IntSel == L3GD20_INT1)
  {
    tmpreg &= 0x7F;	
    tmpreg |= L3GD20_INT1INTERRUPT_ENABLE;
  }
  else if(IntSel == L3GD20_INT2)
  {
    tmpreg &= 0xF7;
    tmpreg |= L3GD20_INT2INTERRUPT_ENABLE;
  }
  
  /* Write value to MEMS CTRL_REG3 register */
  GYRO_IO_Write(&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);
}

/**
  * @brief  Disable  INT1 or INT2 interrupt
  * @param  IntSel: choice of INT1 or INT2 
  *      This parameter can be: 
  *        @arg L3GD20_INT1
  *        @arg L3GD20_INT2   
  * @retval None
  */
void L3GD20_DisableIT(uint8_t IntSel)
{  
  uint8_t tmpreg;
  
  /* Read CTRL_REG3 register */
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);
  
  if(IntSel == L3GD20_INT1)
  {
    tmpreg &= 0x7F;	
    tmpreg |= L3GD20_INT1INTERRUPT_DISABLE;
  }
  else if(IntSel == L3GD20_INT2)
  {
    tmpreg &= 0xF7;
    tmpreg |= L3GD20_INT2INTERRUPT_DISABLE;
  }
  
  /* Write value to MEMS CTRL_REG3 register */
  GYRO_IO_Write(&tmpreg, L3GD20_CTRL_REG3_ADDR, 1);
}

/**
  * @brief  Set High Pass Filter Modality
  * @param  FilterStruct: contains the configuration setting for the L3GD20.        
  * @retval None
  */
void L3GD20_FilterConfig(uint8_t FilterStruct) 
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG2 register */
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG2_ADDR, 1);
  
  tmpreg &= 0xC0;
  
  /* Configure MEMS: mode and cutoff frequency */
  tmpreg |= FilterStruct;
  
  /* Write value to MEMS CTRL_REG2 register */
  GYRO_IO_Write(&tmpreg, L3GD20_CTRL_REG2_ADDR, 1);
}

/**
  * @brief  Enable or Disable High Pass Filter
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be: 
  *         @arg: L3GD20_HIGHPASSFILTER_DISABLE 
  *         @arg: L3GD20_HIGHPASSFILTER_ENABLE          
  * @retval None
  */
void L3GD20_FilterCmd(uint8_t HighPassFilterState)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG5 register */
  GYRO_IO_Read(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
  
  tmpreg &= 0xEF;
  
  tmpreg |= HighPassFilterState;
  
  /* Write value to MEMS CTRL_REG5 register */
  GYRO_IO_Write(&tmpreg, L3GD20_CTRL_REG5_ADDR, 1);
}

/**
  * @brief  Get status for L3GD20 data
  * @param  None         
  * @retval Data status in a L3GD20 Data
  */
uint8_t L3GD20_GetDataStatus(void)
{
  uint8_t tmpreg;
  
  /* Read STATUS_REG register */
  GYRO_IO_Read(&tmpreg, L3GD20_STATUS_REG_ADDR, 1);
  
  return tmpreg;
}

/**
* @brief  Calculate the L3GD20 angular data.
* @param  pfData: Data out pointer
* @retval None
*/
void L3GD20_ReadXYZAngRate(float *pfData)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i =0;
  
  GYRO_IO_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);
  
  GYRO_IO_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);
  
  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if(!(tmpreg & L3GD20_BLE_MSB))
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    }
  }
  
  /* Switch the sensitivity value set in the CRTL4 */
  switch(tmpreg & L3GD20_FULLSCALE_SELECTION)
  {
  case L3GD20_FULLSCALE_250:
    sensitivity=L3GD20_SENSITIVITY_250DPS;
    break;
    
  case L3GD20_FULLSCALE_500:
    sensitivity=L3GD20_SENSITIVITY_500DPS;
    break;
    
  case L3GD20_FULLSCALE_2000:
    sensitivity=L3GD20_SENSITIVITY_2000DPS;
    break;
  }
  /* Divide by sensitivity */
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)(RawData[i] * sensitivity);
  }
}