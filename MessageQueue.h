/*
 * MessageQueue.h
 *
 *  Created on: May 27, 2023
 *      Author: henla464
 */

#ifndef INC_MESSAGEQUEUE_H_
#define INC_MESSAGEQUEUE_H_
#include <stdio.h>
#include <stdbool.h>
#include "service_lora_p2p.h"

#define MESSAGEQUEUE_SIZE 10
#define QUEUEISFULL 1
#define ENQUEUESUCCESS 0
#define SAMEMESSAGE 2


typedef struct LoraMessage
{
    uint8_t BufferSize; 
    uint8_t Buffer[30];
    int16_t Rssi;
    int8_t Snr;
    int8_t Status; //0:RxDone; 1:RxTimeout, 2:RxError
} LoraMeessage_t;

struct MessageQueue {
	LoraMeessage_t MessageQueue_items[MESSAGEQUEUE_SIZE];
	int8_t MessageQueue_front;
	int8_t MessageQueue_rear;
};


extern struct MessageQueue incomingMessageQueue;

uint8_t MessageQueue_getNoOfItems();
bool MessageQueue_isFull(struct MessageQueue * queue);
bool MessageQueue_isEmpty(struct MessageQueue * queue);
bool MessageQueue_isSameMessage(struct LoraMessage * msg1, volatile struct LoraMessage * msg2);
uint8_t MessageQueue_enQueue(struct MessageQueue * queue, LoraMeessage_t * msg);
bool MessageQueue_deQueue(struct MessageQueue * queue, LoraMeessage_t * msg);
bool MessageQueue_peek(struct MessageQueue * queue, LoraMeessage_t * msg);
bool MessageQueue_pop(struct MessageQueue * queue);

#endif /* INC_MessageQueue_H_ */
