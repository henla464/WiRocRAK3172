/*
 * MessageQueue.c
 *
 *  Created on: May 27, 2023
 *      Author: henla464
 */

#include "MessageQueue.h"
#include "app.h"


struct MessageQueue incomingMessageQueue = { .MessageQueue_front = -1, .MessageQueue_rear = -1 };
struct LoraMessage lastMessage;
// Check if the queue is full
uint8_t MessageQueue_getNoOfItems(struct MessageQueue * queue)
{
	if (queue->MessageQueue_front == -1)
	{
		return 0;
	}
	if (queue->MessageQueue_rear < queue->MessageQueue_front)
	{
		return MESSAGEQUEUE_SIZE - (queue->MessageQueue_front - queue->MessageQueue_rear) + 1;
	}
	else
	{
		return queue->MessageQueue_rear - queue->MessageQueue_front + 1;
	}
}

// Check if the queue is full
bool MessageQueue_isFull(struct MessageQueue * queue)
{
	if ((queue->MessageQueue_front == queue->MessageQueue_rear + 1)
		  ||
		  (queue->MessageQueue_front == 0 && queue->MessageQueue_rear == MESSAGEQUEUE_SIZE - 1))
	{
		return true;
	}
	return false;
}

// Check if the queue is empty
bool MessageQueue_isEmpty(struct MessageQueue * queue)
{
	if (queue->MessageQueue_front == -1)
	{
		return true;
	}
	return false;
}

bool MessageQueue_isSameMessage(struct LoraMessage * msg1, volatile struct LoraMessage * msg2)
{
	if (msg1->BufferSize != msg2->BufferSize)
	{
		return false;
	}
	for (uint8_t i; i < msg1->BufferSize; i++)
	{
		if (msg1->Buffer[i] != msg2->Buffer[i]) {
			return false;
		}
	}
	return true;
}

uint8_t MessageQueue_enQueue(struct MessageQueue * queue, LoraMeessage_t * msg)
{
	if (MessageQueue_isFull(queue))
	{
		return QUEUEISFULL;
	}
	else
	{
		if (MessageQueue_isSameMessage(msg, &lastMessage))
		{
			// Same message as last received
			return SAMEMESSAGE;
		}
		if (queue->MessageQueue_front == -1)
		{
			queue->MessageQueue_front = 0;
		}
		queue->MessageQueue_rear = (queue->MessageQueue_rear + 1) % MESSAGEQUEUE_SIZE;
		queue->MessageQueue_items[queue->MessageQueue_rear] = *msg;
		lastMessage = *msg;
		digitalWrite(LORA_IRQ, HIGH);
		return ENQUEUESUCCESS;
	}
}

bool MessageQueue_deQueue(struct MessageQueue * queue, LoraMeessage_t * msg)
{
	if (MessageQueue_isEmpty(queue))
	{
		return false;
	}
	else
	{
		*msg = queue->MessageQueue_items[queue->MessageQueue_front];
		if (queue->MessageQueue_front == queue->MessageQueue_rear) {
			queue->MessageQueue_front = -1;
			queue->MessageQueue_rear = -1;
			digitalWrite(LORA_IRQ, LOW);
		}
		else
		{
			queue->MessageQueue_front = (queue->MessageQueue_front + 1) % MESSAGEQUEUE_SIZE;
		}
		return true;
	}
}

bool MessageQueue_peek(struct MessageQueue * queue, LoraMeessage_t * msg)
{
	if (MessageQueue_isEmpty(queue))
	{
		return false;
	}
	else
	{
		*msg = queue->MessageQueue_items[queue->MessageQueue_front];
		return true;
	}
}

bool MessageQueue_pop(struct MessageQueue * queue)
{
	if (MessageQueue_isEmpty(queue))
	{
		return false;
	}
	else
	{
		if (queue->MessageQueue_front == queue->MessageQueue_rear) {
			queue->MessageQueue_front = -1;
			queue->MessageQueue_rear = -1;
			digitalWrite(LORA_IRQ, LOW);
		}
		else
		{
			queue->MessageQueue_front = (queue->MessageQueue_front + 1) % MESSAGEQUEUE_SIZE;
		}
		return true;
	}
}

