/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

/** \file
 * \brief
 * ESC hardware layer functions for LAN9252 through ESP32 SPI.
 */

#include "esc.h"
#include "esc_hw.h"

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/param.h> // For MIN macro

// ESP-IDF Includes
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h" // for esp_rom_delay_us

// -------------------------------------------------------------------------
// HARDWARE CONFIGURATION
// -------------------------------------------------------------------------
#define LAN9252_HOST        SPI2_HOST // HSPI
#define LAN9252_PIN_MISO    19
#define LAN9252_PIN_MOSI    23
#define LAN9252_PIN_CLK     18
#define LAN9252_PIN_CS      5
#define LAN9252_PIN_IRQ     4  // Connect LAN9252 IRQ pin here

#define LAN9252_SPI_CLOCK   15000000 // 15 MHz
// -------------------------------------------------------------------------

static const char *TAG = "LAN9252";
static spi_device_handle_t spi_handle;

#define BIT(x)	(1U << (x))

#define ESC_CMD_SERIAL_WRITE     0x02
#define ESC_CMD_SERIAL_READ      0x03

#define ESC_CMD_RESET_CTL        0x01F8
#define ESC_CMD_HW_CFG           0x0074
#define ESC_CMD_BYTE_TEST        0x0064
#define ESC_CMD_ID_REV           0x0050
#define ESC_CMD_IRQ_CFG          0x0054
#define ESC_CMD_INT_EN           0x005C

#define ESC_RESET_DIGITAL        0x00000001
#define ESC_RESET_ETHERCAT       0x00000040
#define ESC_RESET_CTRL_RST       (ESC_RESET_DIGITAL & ESC_RESET_ETHERCAT)
#define ESC_HW_CFG_READY         0x08000000
#define ESC_BYTE_TEST_OK         0x87654321

#define ESC_PRAM_RD_FIFO_REG     0x0000
#define ESC_PRAM_WR_FIFO_REG     0x0020
#define ESC_PRAM_RD_ADDR_LEN_REG 0x0308
#define ESC_PRAM_RD_CMD_REG      0x030C
#define ESC_PRAM_WR_ADDR_LEN_REG 0x0310
#define ESC_PRAM_WR_CMD_REG      0x0314

#define ESC_PRAM_CMD_BUSY        0x80000000
#define ESC_PRAM_CMD_ABORT       0x40000000
#define ESC_PRAM_CMD_AVAIL       0x00000001
#define ESC_PRAM_CMD_CNT(x)      (((x) >> 8) & 0x1F)
#define ESC_PRAM_SIZE(x)         ((x) << 16)
#define ESC_PRAM_ADDR(x)         ((x) << 0)

#define ESC_CSR_DATA_REG         0x0300
#define ESC_CSR_CMD_REG          0x0304

#define ESC_CSR_CMD_BUSY         0x80000000
#define ESC_CSR_CMD_READ         (0x80000000 | 0x40000000)
#define ESC_CSR_CMD_WRITE        0x80000000
#define ESC_CSR_CMD_SIZE(x)      ((x) << 16)

/* Helper: Perform SPI Transfer */
static void esp32_spi_transfern(uint8_t *data, size_t len)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len * 8; // length is in bits
    t.tx_buffer = data;
    t.rx_buffer = data; // Read back into the same buffer (full duplex replacement)
    
    // Note: CS is handled automatically by the driver
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI Transfer failed");
    }
}

/* Helper: Single 32-bit Write */
static void esp32_spi_write_32 (uint16_t address, uint32_t val)
{
   uint8_t data[7];

   data[0] = ESC_CMD_SERIAL_WRITE;
   data[1] = ((address >> 8) & 0xFF);
   data[2] = (address & 0xFF);
   data[3] = (val & 0xFF);
   data[4] = ((val >> 8) & 0xFF);
   data[5] = ((val >> 16) & 0xFF);
   data[6] = ((val >> 24) & 0xFF);

   esp32_spi_transfern(data, 7);
}

/* Helper: Single 32-bit Read */
static uint32_t esp32_spi_read_32 (uint16_t address)
{
   uint8_t data[7];

   data[0] = ESC_CMD_SERIAL_READ;
   data[1] = ((address >> 8) & 0xFF);
   data[2] = (address & 0xFF);
   // Bytes 3-6 are dummy for clocking out data
   memset(&data[3], 0, 4);

   esp32_spi_transfern(data, 7);

   return ((uint32_t)data[6] << 24) |
          ((uint32_t)data[5] << 16) |
          ((uint32_t)data[4] << 8) |
          (uint32_t)data[3];
}

/* ESC read CSR function */
static void ESC_read_csr (uint16_t address, void *buf, uint16_t len)
{
   uint32_t value;

   value = (ESC_CSR_CMD_READ | ESC_CSR_CMD_SIZE(len) | address);
   esp32_spi_write_32(ESC_CSR_CMD_REG, value);

   do
   {
      value = esp32_spi_read_32(ESC_CSR_CMD_REG);
   } while(value & ESC_CSR_CMD_BUSY);

   value = esp32_spi_read_32(ESC_CSR_DATA_REG);
   memcpy(buf, (uint8_t *)&value, len);
}

/* ESC write CSR function */
static void ESC_write_csr (uint16_t address, void *buf, uint16_t len)
{
   uint32_t value = 0;

   memcpy((uint8_t*)&value, buf, len);
   esp32_spi_write_32(ESC_CSR_DATA_REG, value);
   
   value = (ESC_CSR_CMD_WRITE | ESC_CSR_CMD_SIZE(len) | address);
   esp32_spi_write_32(ESC_CSR_CMD_REG, value);

   do
   {
      value = esp32_spi_read_32(ESC_CSR_CMD_REG);
   } while(value & ESC_CSR_CMD_BUSY);
}

/* ESC read process data ram function */
static void ESC_read_pram (uint16_t address, void *buf, uint16_t len)
{
   uint32_t value;
   uint8_t * temp_buf = (uint8_t*)buf;
   uint16_t quotient, remainder, byte_offset = 0;
   uint8_t fifo_cnt, fifo_size, fifo_range, first_byte_position, temp_len;
   uint8_t *buffer = NULL;
   int i, size;

   esp32_spi_write_32(ESC_PRAM_RD_CMD_REG, ESC_PRAM_CMD_ABORT);
   do
   {
      value = esp32_spi_read_32(ESC_PRAM_RD_CMD_REG);
   }while(value & ESC_PRAM_CMD_BUSY);

   esp32_spi_write_32(ESC_PRAM_RD_ADDR_LEN_REG, (ESC_PRAM_SIZE(len) | ESC_PRAM_ADDR(address)));

   esp32_spi_write_32(ESC_PRAM_RD_CMD_REG, ESC_PRAM_CMD_BUSY);

   first_byte_position = (address & 0x03);

   while (len > 0)
   {
      if (byte_offset > 0)
      {
         quotient = len/4;
         remainder = len - quotient*4;
      }
      else
      {
         quotient = (len + first_byte_position)/4;
         remainder = (len + first_byte_position) - quotient*4;
      }
      if (remainder != 0)
      {
         quotient++;
      }
      fifo_range = MIN(quotient,16);
      do
      {
         value = esp32_spi_read_32(ESC_PRAM_RD_CMD_REG);
      }while(!(value & ESC_PRAM_CMD_AVAIL) || (ESC_PRAM_CMD_CNT(value) < fifo_range));

      fifo_size = ESC_PRAM_CMD_CNT(value);
      size = 3+4*fifo_size;

      buffer = (uint8_t *)realloc(buffer, size);
      fifo_cnt = fifo_size;

      memset(buffer,0,size);
      buffer[0] = ESC_CMD_SERIAL_READ;
      buffer[1] = ((ESC_PRAM_RD_FIFO_REG >>8) & 0xFF);
      buffer[2] = ( ESC_PRAM_RD_FIFO_REG & 0xFF);

      esp32_spi_transfern(buffer, size);

      i = 3;
      while (fifo_cnt > 0 && len > 0)
      {
         value = buffer[i] | (buffer[i+1] << 8) | (buffer[i+2] << 16) | (buffer[i+3] << 24);

         if (byte_offset > 0)
         {
            temp_len = (len > 4) ? 4: len;
            memcpy(temp_buf + byte_offset ,&value, temp_len);
         }
         else
         {
            temp_len = (len > (4 - first_byte_position)) ? (4 - first_byte_position) : len;
            memcpy(temp_buf ,((uint8_t *)&value + first_byte_position), temp_len);
         }

         i += 4;
         fifo_cnt--;
         len -= temp_len;
         byte_offset += temp_len;
      }
   }
   free(buffer);
}

/* ESC write process data ram function */
static void ESC_write_pram (uint16_t address, void *buf, uint16_t len)
{
   uint32_t value;
   uint8_t * temp_buf = (uint8_t*)buf;
   uint16_t quotient, remainder, byte_offset = 0;
   uint8_t fifo_cnt, fifo_size, fifo_range, first_byte_position, temp_len;
   uint8_t *buffer = NULL;
   int i, size;

   esp32_spi_write_32(ESC_PRAM_WR_CMD_REG, ESC_PRAM_CMD_ABORT);
   do
   {
      value = esp32_spi_read_32(ESC_PRAM_WR_CMD_REG);
   }while(value & ESC_PRAM_CMD_BUSY);

   esp32_spi_write_32(ESC_PRAM_WR_ADDR_LEN_REG, (ESC_PRAM_SIZE(len) | ESC_PRAM_ADDR(address)));

   esp32_spi_write_32(ESC_PRAM_WR_CMD_REG, ESC_PRAM_CMD_BUSY);

   first_byte_position = (address & 0x03);

   while (len > 0)
   {
      if (byte_offset > 0)
      {
         quotient = len/4;
         remainder = len - quotient*4;
      }
      else
      {
         quotient = (len + first_byte_position)/4;
         remainder = (len + first_byte_position) - quotient*4;
      }
      if (remainder != 0)
      {
         quotient++;
      }
      fifo_range = MIN(quotient,16);
      do
      {
         value = esp32_spi_read_32(ESC_PRAM_WR_CMD_REG);
      }while(!(value & ESC_PRAM_CMD_AVAIL) || (ESC_PRAM_CMD_CNT(value) < fifo_range));

      fifo_size = ESC_PRAM_CMD_CNT(value);
      size = 3+4*fifo_size;

      buffer = (uint8_t *)realloc(buffer, size);
      fifo_cnt = fifo_size;

      memset(buffer,0,size);
      buffer[0] = ESC_CMD_SERIAL_WRITE;
      buffer[1] = ((ESC_PRAM_WR_FIFO_REG >> 8) & 0xFF);
      buffer[2] = (ESC_PRAM_WR_FIFO_REG & 0xFF);

      i = 3;
      while (fifo_cnt > 0 && len > 0)
      {
         value = 0;
         if (byte_offset > 0)
         {
            temp_len = (len > 4) ? 4: len;
            memcpy(&value, (temp_buf + byte_offset), temp_len);
         }
         else
         {
            temp_len = (len > (4 - first_byte_position)) ? (4 - first_byte_position) : len;
            memcpy(((uint8_t *)&value + first_byte_position), temp_buf, temp_len);
         }

         buffer[i] = (value & 0xFF);
         buffer[i+1] = ((value >> 8) & 0xFF);
         buffer[i+2] = ((value >> 16) & 0xFF);
         buffer[i+3] = ((value >> 24) & 0xFF);

         i += 4;
         fifo_cnt--;
         len -= temp_len;
         byte_offset += temp_len;
      }

      esp32_spi_transfern((uint8_t *)buffer, size);
   }
   free(buffer);
}

/** ESC read function used by the Slave stack. */
void ESC_read (uint16_t address, void *buf, uint16_t len)
{
   if (address >= 0x1000)
   {
      ESC_read_pram(address, buf, len);
   }
   else
   {
      uint16_t size;
      uint8_t *temp_buf = (uint8_t *)buf;

      while(len > 0)
      {
         size = (len > 4) ? 4 : len;
         
         if(address & BIT(0)) size = 1;
         else if (address & BIT(1)) size = (size & BIT(0)) ? 1 : 2;
         else if (size == 3) size = 1;

         ESC_read_csr(address, temp_buf, size);

         len -= size;
         temp_buf += size;
         address += size;
      }
   }
   ESC_read_csr(ESCREG_ALEVENT,(void *)&ESCvar.ALevent,sizeof(ESCvar.ALevent));
   ESCvar.ALevent = etohs (ESCvar.ALevent);
}

/** ESC write function used by the Slave stack. */
void ESC_write (uint16_t address, void *buf, uint16_t len)
{
   if (address >= 0x1000)
   {
      ESC_write_pram(address, buf, len);
   }
   else
   {
      uint16_t size;
      uint8_t *temp_buf = (uint8_t *)buf;

      while(len > 0)
      {
         size = (len > 4) ? 4 : len;

         if(address & BIT(0)) size = 1;
         else if (address & BIT(1)) size = (size & BIT(0)) ? 1 : 2;
         else if (size == 3) size = 1;

         ESC_write_csr(address, temp_buf, size);

         len -= size;
         temp_buf += size;
         address += size;
      }
   }
   ESC_read_csr(ESCREG_ALEVENT,(void *)&ESCvar.ALevent,sizeof(ESCvar.ALevent));
   ESCvar.ALevent = etohs (ESCvar.ALevent);
}

void ESC_reset (void)
{
    // Not implemented in RPi example, leaving empty
}

/** Main Init Function to be called by SOES */
void ESC_init (const esc_cfg_t * config)
{
   uint32_t value;
   uint32_t counter = 0;
   uint32_t timeout = 1000; // 100ms total roughly

   // 1. Initialize ESP32 SPI
   spi_bus_config_t buscfg = {};
   buscfg.miso_io_num = LAN9252_PIN_MISO;
   buscfg.mosi_io_num = LAN9252_PIN_MOSI;
   buscfg.sclk_io_num = LAN9252_PIN_CLK;
   buscfg.quadwp_io_num = -1;
   buscfg.quadhd_io_num = -1;
   buscfg.max_transfer_sz = 0; // default 4k
   
   ESP_ERROR_CHECK(spi_bus_initialize(LAN9252_HOST, &buscfg, SPI_DMA_CH_AUTO));

   spi_device_interface_config_t devcfg = {};
   devcfg.clock_speed_hz = LAN9252_SPI_CLOCK; 
   devcfg.mode = 0; // Mode 0
   devcfg.spics_io_num = LAN9252_PIN_CS;
   devcfg.queue_size = 7;
   
   ESP_ERROR_CHECK(spi_bus_add_device(LAN9252_HOST, &devcfg, &spi_handle));
   ESP_LOGI(TAG, "SPI Initialized");

   // 2. Initialize Interrupt Pin
   gpio_config_t io_conf = {};
   io_conf.intr_type = GPIO_INTR_DISABLE;
   io_conf.mode = GPIO_MODE_INPUT;
   io_conf.pin_bit_mask = (1ULL << LAN9252_PIN_IRQ);
   io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
   io_conf.pull_up_en = GPIO_PULLUP_ENABLE; 
   gpio_config(&io_conf);


   // 3. Reset the LAN9252 Core
   esp32_spi_write_32(ESC_CMD_RESET_CTL, ESC_RESET_CTRL_RST);

   // Wait for reset
   do
   {
      esp_rom_delay_us(100);
      counter++;
      value = esp32_spi_read_32(ESC_CMD_RESET_CTL);
   } while ((value & ESC_RESET_CTRL_RST) && (counter < timeout));

   if (counter >= timeout) {
       ESP_LOGE(TAG, "Timeout waiting for Reset");
       return;
   }

   // Perform byte test
   counter = 0;
   do
   {
      esp_rom_delay_us(100);
      counter++;
      value = esp32_spi_read_32(ESC_CMD_BYTE_TEST);
   } while ((value != ESC_BYTE_TEST_OK) && (counter < timeout));

   if (counter >= timeout) {
       ESP_LOGE(TAG, "Timeout waiting for Byte Test (read: %08X)", (unsigned int)value);
       return;
   }

   // Check hardware ready
   counter = 0;
   do
   {
      esp_rom_delay_us(100);
      counter++;
      value = esp32_spi_read_32(ESC_CMD_HW_CFG);
   } while (!(value & ESC_HW_CFG_READY) && (counter < timeout));

   if (counter >= timeout) {
       ESP_LOGE(TAG, "Timeout waiting for HW Ready");
       return;
   }

   // Success
   value = esp32_spi_read_32(ESC_CMD_ID_REV);
   ESP_LOGI(TAG, "Detected Chip: %04X Rev: %u", (unsigned int)((value >> 16) & 0xFFFF), (unsigned int)(value & 0xFFFF));

   // Set AL event mask
   value = (ESCREG_ALEVENT_CONTROL | ESCREG_ALEVENT_SMCHANGE | ESCREG_ALEVENT_SM0 | ESCREG_ALEVENT_SM1);
   ESC_ALeventmaskwrite(value);
}

void ESC_interrupt_enable (uint32_t mask)
{
   uint32_t user_int_mask = ESCREG_ALEVENT_DC_SYNC0 | ESCREG_ALEVENT_SM2 | ESCREG_ALEVENT_SM3;
   if (mask & user_int_mask)
   {
      ESC_ALeventmaskwrite(ESC_ALeventmaskread() | (mask & user_int_mask));
   }

   // Set LAN9252 interrupt pin driver as push-pull active high
   esp32_spi_write_32(ESC_CMD_IRQ_CFG, 0x00000111);

   // Enable LAN9252 interrupt
   esp32_spi_write_32(ESC_CMD_INT_EN, 0x00000001);
}

void ESC_interrupt_disable (uint32_t mask)
{
   uint32_t user_int_mask = ESCREG_ALEVENT_DC_SYNC0 | ESCREG_ALEVENT_SM2 | ESCREG_ALEVENT_SM3;

   if (mask & user_int_mask)
   {
      ESC_ALeventmaskwrite(ESC_ALeventmaskread() & ~(mask & user_int_mask));
   }

   esp32_spi_write_32(ESC_CMD_INT_EN, 0x00000000);
}