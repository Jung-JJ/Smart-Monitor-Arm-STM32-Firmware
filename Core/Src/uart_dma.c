/*
 * uart_dma.c
 *
 *  Created on: 2026. 8. 12.
 *      Author: wowns
 */


#include "uart_dma.h"

#include <string.h>

#include "usart.h"
#include "communicationTask.h"

extern osMessageQueueId_t uartRxQueueHandle;
extern osMessageQueueId_t uartTxQueueHandle;

static uint8_t uart2_dma_rx_buffer[UART_DMA_RX_BUFFER_SIZE];

static UartDmaTxFrame_t uart2_tx_active_frame;
static volatile uint8_t uart2_tx_busy = 0U;

HAL_StatusTypeDef UartDma_StartReceive(void)
{
    HAL_StatusTypeDef status;

    status = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart2,
        uart2_dma_rx_buffer,
        UART_DMA_RX_BUFFER_SIZE
    );

    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }

    return status;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                                uint16_t Size)
{
    UartDmaRxChunk_t rxChunk;

    if (huart->Instance != USART2)
    {
        return;
    }

    if ((Size > 0U) &&
        (Size <= UART_DMA_RX_BUFFER_SIZE))
    {
        rxChunk.length = Size;

        memcpy(rxChunk.data,
               uart2_dma_rx_buffer,
               Size);

        (void)osMessageQueuePut(
            uartRxQueueHandle,
            &rxChunk,
            0U,
            0U
        );
    }

    if (UartDma_StartReceive() != HAL_OK)
    {
        Error_Handler();
    }
}

HAL_StatusTypeDef UartDma_QueueTransmit(
    const uint8_t *data,
    uint16_t length)
{
    UartDmaTxFrame_t txFrame;

    if ((data == NULL) ||
        (length == 0U) ||
        (length > UART_DMA_TX_BUFFER_SIZE))
    {
        return HAL_ERROR;
    }

    txFrame.length = length;

    memcpy(
        txFrame.data,
        data,
        length
    );

    if (osMessageQueuePut(
            uartTxQueueHandle,
            &txFrame,
            0U,
            0U) != osOK)
    {
        return HAL_BUSY;
    }

    return HAL_OK;
}

void UartDma_ProcessTransmit(void)
{
    if (uart2_tx_busy != 0U)
    {
        return;
    }

    if (osMessageQueueGet(
            uartTxQueueHandle,
            &uart2_tx_active_frame,
            NULL,
            0U) != osOK)
    {
        return;
    }

    uart2_tx_busy = 1U;

    if (HAL_UART_Transmit_DMA(
            &huart2,
            uart2_tx_active_frame.data,
            uart2_tx_active_frame.length) != HAL_OK)
    {
        uart2_tx_busy = 0U;
    }
}

void HAL_UART_TxCpltCallback(
    UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uart2_tx_busy = 0U;
    }
}
