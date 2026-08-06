/*
 * protocol.c
 *
 *  Created on: 2026. 8. 1.
 *      Author: wowns
 */


#include "protocol.h"

#include <stddef.h>
#include <string.h>

uint8_t Protocol_CalculateChecksum(uint8_t msg_id,
                                   uint8_t data_length,
                                   const uint8_t *data)
{
	uint8_t checksum;
	uint8_t	index;

	checksum = msg_id ^ data_length;
	if((data != NULL)&&(data_length>0U)){
		for(index = 0U; index < data_length; index++){
			checksum ^= data[index];
		}
	}
	return checksum;
}

ProtocolStatus_t Protocol_BuildFrame(uint8_t msg_id,
                                     const uint8_t *data,
                                     uint8_t data_length,
                                     uint8_t *frame,
                                     uint16_t frame_capacity,
                                     uint16_t *frame_length)
{
    uint16_t required_length;
    uint8_t checksum;

    if ((frame == NULL) || (frame_length == NULL))
    {
        return PROTOCOL_ERROR_NULL_POINTER;
    }

    if (data_length > PROTOCOL_MAX_DATA_LENGTH)
    {
        return PROTOCOL_ERROR_DATA_LENGTH;
    }

    if ((data_length > 0U) && (data == NULL))
    {
        return PROTOCOL_ERROR_NULL_POINTER;
    }

    required_length =
        (uint16_t)data_length + PROTOCOL_OVERHEAD_LENGTH;

    if (frame_capacity < required_length)
    {
        return PROTOCOL_ERROR_FRAME_LENGTH;
    }

    frame[0] = PROTOCOL_START_BYTE;
    frame[1] = msg_id;
    frame[2] = data_length;

    if (data_length > 0U)
    {
        (void)memcpy(&frame[3], data, data_length);
    }

    checksum =
        Protocol_CalculateChecksum(msg_id,
                                   data_length,
                                   data);

    frame[3U + data_length] = checksum;
    frame[4U + data_length] = PROTOCOL_END_BYTE;

    *frame_length = required_length;

    return PROTOCOL_OK;
}

ProtocolStatus_t Protocol_DecodeFrame(const uint8_t *frame,
                                      uint16_t frame_length,
                                      ProtocolMessage_t *message)
{
    uint8_t data_length;
    uint8_t received_checksum;
    uint8_t calculated_checksum;
    uint16_t expected_frame_length;

    if ((frame == NULL) || (message == NULL))
    {
        return PROTOCOL_ERROR_NULL_POINTER;
    }


    if (frame_length < PROTOCOL_MIN_FRAME_LENGTH)
    {
        return PROTOCOL_ERROR_FRAME_LENGTH;
    }

    if (frame[0] != PROTOCOL_START_BYTE)
    {
        return PROTOCOL_ERROR_START_BYTE;
    }

    data_length = frame[2];

    if (data_length > PROTOCOL_MAX_DATA_LENGTH)
    {
        return PROTOCOL_ERROR_DATA_LENGTH;
    }

    expected_frame_length =
        (uint16_t)data_length + PROTOCOL_OVERHEAD_LENGTH;

    if (frame_length != expected_frame_length)
    {
        return PROTOCOL_ERROR_FRAME_LENGTH;
    }

    if (frame[expected_frame_length - 1U] != PROTOCOL_END_BYTE)
    {
        return PROTOCOL_ERROR_END_BYTE;
    }

    received_checksum = frame[3U + data_length];

    calculated_checksum =
        Protocol_CalculateChecksum(
            frame[1],
            data_length,
            (data_length > 0U) ? &frame[3] : NULL
        );

    if (received_checksum != calculated_checksum)
    {
        return PROTOCOL_ERROR_CHECKSUM;
    }

    message->msg_id = frame[1];
    message->data_length = data_length;

    if (data_length > 0U)
    {
        (void)memcpy(message->data,
                     &frame[3],
                     data_length);
    }

    return PROTOCOL_OK;
}

void Protocol_RxParserInit(ProtocolRxParser_t *parser)
{
    Protocol_RxParserReset(parser);
}

void Protocol_RxParserReset(ProtocolRxParser_t *parser)
{
    if (parser == NULL)
    {
        return;
    }

    parser->state = PROTOCOL_RX_WAIT_START;
    parser->msg_id = 0U;
    parser->data_length = 0U;
    parser->data_index = 0U;
    parser->checksum = 0U;
    parser->frame_complete = 0U;
    memset(parser->data,0,sizeof(parser->data));

}



void Protocol_RxProcessByte(ProtocolRxParser_t *parser, uint8_t rx_byte)
{
    if (parser == NULL)
    {
        return;
    }

    switch (parser->state)
    {
        case PROTOCOL_RX_WAIT_START:
            if (rx_byte == PROTOCOL_START_BYTE)
            {
            	parser->frame_complete = 0U;
                parser->state = PROTOCOL_RX_READ_MSG_ID;
            }
            break;

        case PROTOCOL_RX_READ_MSG_ID:
        	parser->msg_id = rx_byte;
        	parser->state = PROTOCOL_RX_READ_LENGTH;
        	break;

        case PROTOCOL_RX_READ_LENGTH:
        	parser->data_length = rx_byte;
        	if(parser->data_length>PROTOCOL_MAX_DATA_LENGTH){
        		Protocol_RxParserReset(parser); //잘못된 데이터 즉 폐기.
        	}

        	else if(parser->data_length==0U){
        		parser->state = PROTOCOL_RX_READ_CHECKSUM; //데이터 거칠 필요 x
        	}

        	else{
        		parser->data_index = 0U; //새로운 패킷은 0부터 시작
        		parser->state = PROTOCOL_RX_READ_DATA;
        	}
        	break;

        case PROTOCOL_RX_READ_DATA:
        	parser->data[parser->data_index] = rx_byte;
        	parser->data_index++;
        	if(parser->data_index >= parser->data_length){
        		parser->state = PROTOCOL_RX_READ_CHECKSUM;
        	}
        	break;

        case PROTOCOL_RX_READ_CHECKSUM:
        	parser->checksum = rx_byte;
        	parser->state = PROTOCOL_RX_READ_END;
        	break;

        case PROTOCOL_RX_READ_END:
        	if(rx_byte == PROTOCOL_END_BYTE){
        		parser->frame_complete = 1U;
        		parser->state = PROTOCOL_RX_WAIT_START;
        	}
        	else{
        		Protocol_RxParserReset(parser);
        	}
        	parser->state = PROTOCOL_RX_WAIT_START;
        	break;

        default:
        	Protocol_RxParserReset(parser);

            break;
    }


}

ProtocolStatus_t Protocol_RxDecodeMessage(const ProtocolRxParser_t *parser,
   										  ProtocolMessage_t *message){
   	uint8_t frame[PROTOCOL_MAX_FRAME_LENGTH];
   	uint8_t frame_length;

   	if((parser==NULL)||(message==NULL)){
   		return PROTOCOL_ERROR_NULL_POINTER;
   	}

   	if(parser->frame_complete == 0U){
   		return PROTOCOL_ERROR_FRAME_LENGTH;
   	}

   	frame_length = (uint16_t)parser->data_length + PROTOCOL_OVERHEAD_LENGTH;

   	frame[0] = PROTOCOL_START_BYTE;
   	frame[1] = parser->msg_id;
   	frame[2] = parser->data_length;

   	if(parser->data_length > 0U){
   		memcpy(&frame[3],parser->data,parser->data_index);
   	}

   	frame[3U+parser->data_length] = parser->checksum;
   	frame[4U+parser->data_length] = PROTOCOL_END_BYTE;

   	return Protocol_DecodeFrame(frame, frame_length, message);

   }

