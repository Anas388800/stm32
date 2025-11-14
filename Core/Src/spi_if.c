#include "spi_if.h"
#include "os_ressource.h"
#include "board.h"
#include "uart_debug.h"
#include "cmsis_os.h"
#include <string.h>

#define SPI_BUFFER_SIZE 64

/* Buffer DMA */
static uint8_t spiRxBuffer[SPI_BUFFER_SIZE];

/* Buffer pour reconstituer le message */
static char messageBuffer[SPI_BUFFER_SIZE];

/* Variables de parsing */
static uint8_t inMessage = 0;
static uint16_t idx = 0;

/* -------------------------------------------------------------------- */
/*                      Initialisation SPI (DMA)                        */
/* -------------------------------------------------------------------- */
void spi_if_init(void)
{
    /* Lancer la réception DMA */
    HAL_SPI_Receive_DMA(&hspi1, spiRxBuffer, SPI_BUFFER_SIZE);

    UART_Debug(" -> SPI interface init (DMA started)\r\n");
}

/* -------------------------------------------------------------------- */
/*            Callback appelé depuis HAL_SPI_RxCpltCallback             */
/* -------------------------------------------------------------------- */
void spi_if_dma_rx_complete(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1)
        return;

    /* Analyse du buffer DMA pour découper <MESSAGE> */
    for (int i = 0; i < SPI_BUFFER_SIZE; i++)
    {
        char c = spiRxBuffer[i];

        if (c == '<')
        {
            inMessage = 1;
            idx = 0;
            continue;
        }
        else if (c == '>')
        {
            if (inMessage)
            {
                messageBuffer[idx] = '\0';
                inMessage = 0;

                /* Envoi à la task par queue */
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xQueueSendFromISR(SPIQueue, messageBuffer, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
            continue;
        }

        if (inMessage && idx < SPI_BUFFER_SIZE - 1)
        {
            messageBuffer[idx++] = c;
        }
    }

    /* Relancer la réception DMA */
    HAL_SPI_Receive_DMA(&hspi1, spiRxBuffer, SPI_BUFFER_SIZE);
}

/* -------------------------------------------------------------------- */
/*                      Tâche de traitement SPI                          */
/* -------------------------------------------------------------------- */
void spi_task(void *argument)
{
    char receivedMessage[SPI_BUFFER_SIZE];

    UART_Debug("SPI Task started.\r\n");

    for (;;)
    {
        if (xQueueReceive(SPIQueue, receivedMessage, pdMS_TO_TICKS(100)) == pdPASS)
        {
            UART_Debug("SPI RX: %s\r\n", receivedMessage);

            /* Tu peux ensuite gérer LED/BUZZER ici selon le message */
            /* exemple :
                if (strcmp(receivedMessage, "RD0") == 0) { ... }
            */
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
