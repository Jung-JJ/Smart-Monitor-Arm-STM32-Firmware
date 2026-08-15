/*
 * communicationTask.h
 *
 *  Created on: 2026. 8. 1.
 *      Author: wowns
 */

#ifndef INC_COMMUNICATIONTASK_H_
#define INC_COMMUNICATIONTASK_H_
#include <stdint.h>
void StartCommunicationTask(void *argument);

extern volatile uint8_t communicationLost;

#endif /* INC_COMMUNICATIONTASK_H_ */
