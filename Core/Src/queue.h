/*
 * queue.h
 *
 *  Created on: Apr 12, 2025
 *      Author: ianv
 */

#ifndef QUEUE_H
#define QUEUE_H

#include <stdint.h>
#include <stdbool.h>

#define QUEUE_SIZE 100

typedef struct {
    uint8_t data[QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} Queue;

// Initializes the queue
void initQueue(Queue *q);

// Enqueues a value into the queue
// Returns 1 on success, -1 if full
int enqueue(Queue *q, uint8_t value);

// Dequeues a value from the queue
// Returns the value, or -1 if empty
int dequeue(Queue *q);
