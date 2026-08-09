#include "communicationTask.h"
#include "cmsis_os.h"
#include "protocol.h"
#include "usart.h"
#include "motorCommand.h"
#include "encoderTask.h"
#include "motorTask.h"

#define HEARTBEAT_TIMEOUT_MS       3000U
#define PROTOCOL_RX_TIMEOUT_MS       20U
#define UART_RX_WAIT_TIMEOUT_MS       5U
#define CURRENT_ANGLE_TX_PERIOD_MS  200U


extern osMessageQueueId_t motorCommandQueueHandle;
extern osMessageQueueId_t motorStatusQueueHandle;

static uint8_t txFrame[PROTOCOL_MAX_FRAME_LENGTH];
static uint16_t txFrameLength = 0U;
static uint8_t rxByte;
static ProtocolRxParser_t rxParser;

static ProtocolMessage_t rxMessage;
static ProtocolStatus_t decodeStatus;
static ProtocolStatus_t buildStatus;
static uint32_t lastHeartbeatTick = 0U;

static uint32_t lastRxByteTick = 0U;

static uint8_t readySent = 0U;

volatile uint8_t lastAliveCounter = 0U;
volatile uint32_t heartbeatReceiveCount = 0U;
volatile uint8_t communicationLost = 1U;

volatile int16_t debugCurrentAngleX10 = 0;

static uint8_t Communication_CommandTypeToMsgId(uint8_t commandType)
{
    switch (commandType)
    {
        case MOTOR_COMMAND_SET_TARGET:
            return PROTOCOL_MSG_SET_TARGET;

        case MOTOR_COMMAND_SET_HOME:
            return PROTOCOL_MSG_SET_HOME;

        case MOTOR_COMMAND_MOVE_HOME:
            return PROTOCOL_MSG_MOVE_HOME;

        case MOTOR_COMMAND_JOG:
            return PROTOCOL_MSG_JOG;

        default:
            return 0U;
    }
}

static void Communication_SendError(MotorError_t error)
{
    uint8_t errorData[1];

    errorData[0] = (uint8_t)error;

    buildStatus =
        Protocol_BuildFrame(
            PROTOCOL_MSG_ERROR,
            errorData,
            sizeof(errorData),
            txFrame,
            sizeof(txFrame),
            &txFrameLength
        );

    if (buildStatus == PROTOCOL_OK)
    {
        (void)HAL_UART_Transmit(
            &huart2,
            txFrame,
            txFrameLength,
            100U
        );
    }
}


static void Communication_SendReady(void)
{
    buildStatus =
        Protocol_BuildFrame(
            PROTOCOL_MSG_READY,
            NULL,
            0U,
            txFrame,
            sizeof(txFrame),
            &txFrameLength
        );

    if (buildStatus == PROTOCOL_OK)
    {
        (void)HAL_UART_Transmit(
            &huart2,
            txFrame,
            txFrameLength,
            100U
        );
    }
}

static int16_t Protocol_ReadInt16BigEndian(const uint8_t *data)
{
    uint16_t value;

    value = ((uint16_t)data[0] << 8) |
            ((uint16_t)data[1]);

    return (int16_t)value;
}


static void Communication_SendAck(uint8_t receivedMsgId)
{
    uint8_t ackData[1];

    ackData[0] = receivedMsgId;

    buildStatus =
        Protocol_BuildFrame(
            PROTOCOL_MSG_ACK,
            ackData,
            1U,
            txFrame,
            sizeof(txFrame),
            &txFrameLength
        );

    if (buildStatus == PROTOCOL_OK)
    {
        (void)HAL_UART_Transmit(
            &huart2,
            txFrame,
            txFrameLength,
            100U
        );
    }
}

static void Communication_WriteInt16BigEndian(uint8_t *data, int16_t value)
{
    uint16_t rawValue;

    if (data == NULL)
    {
        return;
    }

    rawValue = (uint16_t)value;

    data[0] = (uint8_t)(rawValue >> 8U);
    data[1] = (uint8_t)(rawValue & 0xFFU);
}

static int16_t Communication_DegreeToX10(float angleDeg)
{
    float scaledAngle;

    scaledAngle = angleDeg * 10.0f;

    if (scaledAngle >= 0.0f)
    {
        scaledAngle += 0.5f;
    }
    else
    {
        scaledAngle -= 0.5f;
    }

    return (int16_t)scaledAngle;
}

static uint8_t Communication_SendCurrentAngle(void)
{
    uint8_t angleData[2];
    float angleDeg;
    int16_t angleX10;

    if (encoderStatus != AS5600_OK)
    {
        return 0U;
    }

    angleDeg = currentEncoderAngleDeg;

    angleX10 = Communication_DegreeToX10(angleDeg);

    debugCurrentAngleX10 = angleX10;

    Communication_WriteInt16BigEndian(
        angleData,
        angleX10
    );

    buildStatus =
        Protocol_BuildFrame(
            PROTOCOL_MSG_CURRENT_ANGLE,
            angleData,
            sizeof(angleData),
            txFrame,
            sizeof(txFrame),
            &txFrameLength
        );

    if (buildStatus != PROTOCOL_OK)
    {
        return 0U;
    }

    if (HAL_UART_Transmit(&huart2,
                          txFrame,
                          txFrameLength,
                          100U) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

static uint8_t Communication_SendCurrentCommandAngles(void)
{
    uint8_t angleData[6];

    float theta1Deg;
    float theta2Deg;
    float theta3Deg;

    int16_t theta1X10;
    int16_t theta2X10;
    int16_t theta3X10;

    Motor_GetCommandedAngles(&theta1Deg, &theta2Deg, &theta3Deg);

    theta1X10 = Communication_DegreeToX10(theta1Deg);
    theta2X10 = Communication_DegreeToX10(theta2Deg);
    theta3X10 = Communication_DegreeToX10(theta3Deg);

    Communication_WriteInt16BigEndian(&angleData[0], theta1X10);
    Communication_WriteInt16BigEndian(&angleData[2], theta2X10);
    Communication_WriteInt16BigEndian(&angleData[4], theta3X10);

    buildStatus =
        Protocol_BuildFrame(
            PROTOCOL_MSG_CURRENT_COMMAND_ANGLES,
            angleData,
            sizeof(angleData),
            txFrame,
            sizeof(txFrame),
            &txFrameLength
        );

    if (buildStatus != PROTOCOL_OK)
    {
        return 0U;
    }

    if (HAL_UART_Transmit(&huart2,
                          txFrame,
                          txFrameLength,
                          100U) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}


void StartCommunicationTask(void *argument)
{
    uint32_t currentTick;
    MotorCommand_t motorCommand; //우리가 정의한 큐 구조
    MotorStatusMessage_t motorStatusMessage;
    osStatus_t queueStatus; //큐 상태
    uint32_t lastCurrentAngleTxTick;

    (void)argument;

    Protocol_RxParserInit(&rxParser);

    communicationLost = 1U;
    readySent = 0U;

    lastHeartbeatTick = osKernelGetTickCount();
    lastRxByteTick = osKernelGetTickCount();
    lastCurrentAngleTxTick = osKernelGetTickCount();

    for (;;)
    {
        if (HAL_UART_Receive(&huart2,&rxByte,1U,UART_RX_WAIT_TIMEOUT_MS) == HAL_OK)
        {
            lastRxByteTick = osKernelGetTickCount();
            Protocol_RxProcessByte(&rxParser, rxByte);

            if (rxParser.frame_complete == 1U)
            {
                decodeStatus = Protocol_RxDecodeMessage(&rxParser,&rxMessage);
                Protocol_RxParserReset(&rxParser);

                if (decodeStatus == PROTOCOL_OK)
                {
                    switch (rxMessage.msg_id){
						case PROTOCOL_MSG_HEARTBEAT:
						{
							if (rxMessage.data_length == 1U)
							{
								lastAliveCounter = rxMessage.data[0];

								heartbeatReceiveCount++;

								lastHeartbeatTick =
									osKernelGetTickCount();

								communicationLost = 0U;

								Communication_SendAck(
									rxMessage.msg_id
								);

								if ((motorState == MOTOR_STATE_IDLE) &&
									(readySent == 0U))
								{
									Communication_SendReady();

									readySent = 1U;
								}
							}

							break;
						}

                        case PROTOCOL_MSG_SET_TARGET:
                        	{
                        		if (communicationLost != 0U)
                        		    {
                        		        Communication_SendError(
                        		            MOTOR_ERROR_COMM_LOST
                        		        );

                        		        break;
                        		    }

								if (rxMessage.data_length == 6U)
								{
									motorCommand.type = MOTOR_COMMAND_SET_TARGET;

									motorCommand.theta1_x10 =
										Protocol_ReadInt16BigEndian(&rxMessage.data[0]);

									motorCommand.theta2_x10 =
										Protocol_ReadInt16BigEndian(&rxMessage.data[2]);

									motorCommand.theta3_x10 =
										Protocol_ReadInt16BigEndian(&rxMessage.data[4]);

									queueStatus = osMessageQueuePut(motorCommandQueueHandle, &motorCommand, 0U, 0U);

									if (queueStatus == osOK)
									{

										Communication_SendAck(
											rxMessage.msg_id
										);
									}
								}

								break;
                        	}

                        case PROTOCOL_MSG_JOG:

                        	if (communicationLost != 0U){
								Communication_SendError(MOTOR_ERROR_COMM_LOST);
								break;
							}

                        	if(rxMessage.data_length == 3U){
                        		motorCommand.type = MOTOR_COMMAND_JOG;
                        		motorCommand.axis = (MotorAxis_t)rxMessage.data[0];

                        		motorCommand.delta_x10 = Protocol_ReadInt16BigEndian(&rxMessage.data[1]);
                        		queueStatus = osMessageQueuePut(motorCommandQueueHandle,
                        				&motorCommand, 0U, 0U);

                        		if(queueStatus == osOK){
                        			Communication_SendAck(rxMessage.msg_id);
                        		}

                        	}
                        	break;
                        case PROTOCOL_MSG_SET_HOME:
						{
							if (communicationLost != 0U)
							{
								Communication_SendError(MOTOR_ERROR_COMM_LOST);
								break;
							}

							if(rxMessage.data_length == 0U){
								motorCommand.type = MOTOR_COMMAND_SET_HOME;
								queueStatus = osMessageQueuePut(motorCommandQueueHandle, &motorCommand, 0U, 0U);

								if(queueStatus ==osOK){
									Communication_SendAck(rxMessage.msg_id);
								}
							}
							break;
						}

                        case PROTOCOL_MSG_MOVE_HOME:
                        {
                        	if (communicationLost != 0U){
								Communication_SendError(MOTOR_ERROR_COMM_LOST);
								break;
							}

                        	if(rxMessage.data_length == 0U){
								motorCommand.type = MOTOR_COMMAND_MOVE_HOME;
								queueStatus = osMessageQueuePut(motorCommandQueueHandle, &motorCommand, 0U, 0U);

								if(queueStatus ==osOK){
									Communication_SendAck(rxMessage.msg_id);
								}
							}
							break;
                        }

                        default:
                            break;
                    }
                }
            }
        }

        currentTick = osKernelGetTickCount();

        if ((rxParser.state != PROTOCOL_RX_WAIT_START) &&
            ((currentTick - lastRxByteTick) >= PROTOCOL_RX_TIMEOUT_MS)) //아직 프레임을 받는데 타임아웃보다 오래걸리면 패킷 버리기.
        {
            Protocol_RxParserReset(&rxParser);
        }

        currentTick = osKernelGetTickCount();

        if ((currentTick - lastHeartbeatTick) >= HEARTBEAT_TIMEOUT_MS)
        {
            communicationLost = 1U;
            readySent = 0U;
        }

        if ((currentTick - lastCurrentAngleTxTick) >= CURRENT_ANGLE_TX_PERIOD_MS)
        {
            lastCurrentAngleTxTick = currentTick;

            (void)Communication_SendCurrentAngle();
        }

        if (osMessageQueueGet(motorStatusQueueHandle, &motorStatusMessage, NULL, 0U) == osOK)
        {
            switch (motorStatusMessage.status)
            {
				case MOTOR_STATUS_COMMAND_DONE:
				{
					uint8_t doneData[1];
					uint8_t completedMsgId;

					completedMsgId =
						Communication_CommandTypeToMsgId(
							motorStatusMessage.command_type
						);

					if (completedMsgId == 0U)
					{
						break;
					}

					doneData[0] = completedMsgId;

					buildStatus =
						Protocol_BuildFrame(
							PROTOCOL_MSG_COMMAND_DONE,
							doneData,
							sizeof(doneData),
							txFrame,
							sizeof(txFrame),
							&txFrameLength
						);

					if (buildStatus == PROTOCOL_OK)
					{
						(void)HAL_UART_Transmit(
							&huart2,
							txFrame,
							txFrameLength,
							100U
						);
					}

					(void)Communication_SendCurrentCommandAngles();

					break;
				}



                case MOTOR_STATUS_ERROR:
                {
                	Communication_SendError(motorStatusMessage.error);
                    break;

                default:
                    break;
                }
            }
        }
    }
}
