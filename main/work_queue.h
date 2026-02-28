#ifndef WORK_QUEUE_H
#define WORK_QUEUE_H

#include <pthread.h>
#include "freertos/FreeRTOS.h"

#define QUEUE_SIZE 128

typedef struct {
    void * buffer[QUEUE_SIZE];
    int head;
    int tail;
    int count;
    pthread_mutex_t lock;
} work_queue;

void queue_init(work_queue *q);
int queue_enqueue(work_queue *q, void * item);
void * queue_dequeue(work_queue *q);
void queue_clear(work_queue *q);

void ASIC_jobs_queue_clear(work_queue *q);

#endif /* WORK_QUEUE_H */
