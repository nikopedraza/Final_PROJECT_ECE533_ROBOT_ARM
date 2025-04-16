#include <stdio.h>
#include <stdint.h>

#define SIZE 100

typedef struct {
	uint8_t data[SIZE];
	uint8_t head;
	uint8_t tail;
	uint8_t count;
}Queue;

void initQueue(Queue *q) {
	q->head = 0;
	q->tail = 0;
	q->count = 0;
}

int enqueue(Queue *q, uint8_t value) {
	//queue full
	if (q->count == SIZE) {
		return -1;
	}

	q->data[q->tail] = value;
	q->tail = (q->tail + 1) % SIZE; // wrap around increment rear
	q->count++;

	return 1;
}

int dequeue(Queue *q) {
	// queue empty
	if (q->count == 0) {
		return -1;
	}

	uint8_t ret_val = q->data[q->head];
	q->head = (q->head + 1) % SIZE;
	q->count--;
	return ret_val;
}
