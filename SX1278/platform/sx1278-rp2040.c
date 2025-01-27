/**
 *  @brief:  Implementation of a SX1278 platform dependent radio functions
 *  @author: luk6xff
 *  @email:  luszko@op.pl
 *  @date:   2019-11-15
 */

#include <stdint.h>

#include "boards/pico.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/spi.h"

#include "../sx1278.h"

/**
 * SPI Interface
 */
static spi_inst_t *SPIHandle = spi0;

/**
 * SX1278 Reset pin
 */
int SX1278ResetPin = 20;

/**
 * SX1278 DIO pins
 */
int SX1278Dio0 = 15;

/**
 * Tx and Rx timers
 */


//-----------------------------------------------------------------------------
void SX1278IoInit(void)
{
    // SPI peripheral init
    spi_init(SPIHandle, 1 * 1000 * 1000); // 1MHz
    spi_set_format(SPIHandle, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // Init SPI GPIO
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);

    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, true);

    gpio_init(SX1278ResetPin);
    gpio_set_dir(SX1278ResetPin, GPIO_OUT);
    gpio_put(SX1278ResetPin, true);

    gpio_init(SX1278Dio0);
    gpio_set_dir(SX1278Dio0, false);
}

void SX1278SpiSelect(bool state)
{
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, state);
}

void SX1278WriteBuffer(uint8_t addr, uint8_t *buffer, uint8_t size)
{
    int nbytes = 0;

    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, false);

    // Write register addr
    uint8_t addr_masked = addr | 0x80;
    nbytes = spi_write_blocking(SPIHandle, &addr_masked, sizeof(addr_masked));
    if (nbytes != sizeof(addr_masked))
    {
        // DEBUG("%s", "error writing\n");
        nbytes = -1;
        goto out;
    }

    // Write data
    nbytes = spi_write_blocking(SPIHandle, buffer, size);
    if (nbytes != size)
    {
        // DEBUG("%s", "error writing\n");
        nbytes = -1;
        goto out;
    }

out:
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, true);
}


void SX1278ReadBuffer(uint8_t addr, uint8_t *buffer, uint8_t size)
{
    bool ok = true;
    int nbytes = 0;

    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, false);

    // Write register addr
    nbytes = spi_write_blocking(SPIHandle, &addr, sizeof(addr));
    if (nbytes != sizeof(addr))
    {
        // DEBUG("%s", "error writing addr\n");
        goto out;
    }

    // Write data
    nbytes = spi_read_blocking(SPIHandle, 0, buffer, size);
    if (nbytes != size)
    {
        // DEBUG("%s", "error reading\n");
        goto out;
    }

out:
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, true);
}

//-----------------------------------------------------------------------------
void SX1278IoDeInit(void)
{
}


static DioIrqHandler *gpioIrqHandlers = NULL;
void SX1278GpioIrqCallback(uint gpio, uint32_t event_mask)
{
    if (gpioIrqHandlers == NULL)
    {
        return;
    }

    if (gpio == SX1278Dio0)
    {
        gpioIrqHandlers[0]();
    }
}

//-----------------------------------------------------------------------------
void SX1278IoIrqInit(DioIrqHandler *irqHandlers)
{
    gpioIrqHandlers = irqHandlers;
    gpio_set_irq_enabled_with_callback(SX1278Dio0, GPIO_IRQ_EDGE_RISE, true, SX1278GpioIrqCallback);
}


//-----------------------------------------------------------------------------
void SX1278Reset(void)
{
    gpio_put(SX1278ResetPin, false);
    SX1278DelayMs(1);
    gpio_put(SX1278ResetPin, true);
    SX1278DelayMs(20);
}


//-----------------------------------------------------------------------------
void SX1278SetTimeout(TimeoutTimer_t timer, timeoutFuncPtr func, int timeout_ms)
{
    switch(timer)
    {
	    case RXTimeoutTimer:
        {
            if (func)
            {
                // rxTimeoutTimer.attach_us(mbed::callback(func), timeout_ms*1000);
            }
            else
            {
                // rxTimeoutTimer.detach();
            }
            break;
        }
        case TXTimeoutTimer:
        {
            if (func)
            {
                // txTimeoutTimer.attach_us(mbed::callback(func), timeout_ms*1000);
            }
            else
            {
                // txTimeoutTimer.detach();
            }
            break;
        }
        case RXTimeoutSyncWordTimer:
        {
            if (func)
            {
                // rxTimeoutSyncWord.attach_us(mbed::callback(func), timeout_ms*1000);
            }
            else
            {
                // rxTimeoutSyncWord.detach();
            }
            break;
        }
    }
}


void SX1278DelayMs(int ms)
{
    busy_wait_ms(ms);
}


