#include "sensor.h"
#include "main.h"
#include "stdio.h"
#include "bmp280.h"
#include "bmp280_defs.h"


int8_t spi_reg_write ( uint8_t cs , uint8_t reg_addr , uint8_t * reg_data , uint16_t length )
 {
 /* Implement the SPI write routine according to the target machine . */
 HAL_StatusTypeDef status = HAL_OK ;
 int32_t iError = BMP280_OK ;
 uint8_t txarray [ 28 * 2 ];//28 = SPI_BUFFER_LEN   2=BMP280_ADDRESS_INDEX
 uint8_t stringpos ;
 txarray [0] = reg_addr ;
  for ( stringpos = 0; stringpos < length ; stringpos ++) {
  txarray [ stringpos + 1 ] = reg_data [ stringpos ];//1=BMP280_DATA_INDEX
  }

  HAL_GPIO_WritePin ( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_RESET );
  status = HAL_SPI_Transmit ( & hspi4 , ( uint8_t *)(& txarray ) , length *2 , 100);
  while ( hspi4 . State == HAL_SPI_STATE_BUSY ) {};
  HAL_GPIO_WritePin ( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_SET );

  if ( status != HAL_OK )
  {
  // The BMP280 API calls for 0 return value as a success ,
  // and -1 returned as failure
  iError = ( -1);
  }
  return ( int8_t ) iError ;
  }



int8_t spi_reg_read ( uint8_t cs , uint8_t reg_addr , uint8_t * reg_data , uint16_t length )
 {

 /* Implement the SPI read routine according to the target machine . */
 HAL_StatusTypeDef status = HAL_OK ;
 int32_t iError = BMP280_OK ;
 uint8_t txarray [ 30 ] = {0 ,};//30 = SPI_BUFFER_LENGTH
 uint8_t rxarray [ 30 ] = {0 ,};//30 = SPI_BUFFER_LENGTH
 uint8_t stringpos ;

 txarray [0] = reg_addr ;

 HAL_GPIO_WritePin ( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_RESET );
 status = HAL_SPI_TransmitReceive ( & hspi4 , ( uint8_t *)(& txarray ) ,
 ( uint8_t *)(& rxarray ) , length +1 , 5);
 while ( hspi4 . State == HAL_SPI_STATE_BUSY ) {};
 HAL_GPIO_WritePin ( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_SET );

 for ( stringpos = 0; stringpos < length ; stringpos ++) {
 *( reg_data + stringpos ) = rxarray [ stringpos + 1 ];//1 = BMP280_DATA_INDEX
 }

 if ( status != HAL_OK )
 {
 // The BMP280 API calls for 0 return value as a success ,
 // and -1 returned as failure
 iError = ( -1);
 }
 return ( int8_t ) iError ;
 }
