#include "work_queue.h"
#include "esp_log.h"
#include "mining.h"
#include "stratum_api.h"
#include <stdlib.h>

static const char * TAG = "work_queue";

void queue_init(work_queue *q) {
    q->head = 0;
    q->tail = 0;
    q->count = 0;
    pthread_mutex_init(&q->lock, NULL);
}

int queue_enqueue(work_queue *q, void * item) {
    pthread_mutex_lock(&q->lock);
    if (q->count >= QUEUE_SIZE) {
        pthread_mutex_unlock(&q->lock);
        return -1; 
    }
    q->buffer[q->tail] = item;
    q->tail = (q->tail + 1) % QUEUE_SIZE;
    q->count++;
    pthread_mutex_unlock(&q->lock);
    return 0; 
}

void * queue_dequeue(work_queue *q) {
    pthread_mutex_lock(&q->lock);
    if (q->count == 0) {
        pthread_mutex_unlock(&q->lock);
        return NULL;
    }
    void * item = q->buffer[q->head];
    q->buffer[q->head] = NULL;
    q->head = (q->head + 1) % QUEUE_SIZE;
    q->count--;
    pthread_mutex_unlock(&q->lock);
    return item;
}

void queue_clear(work_queue *q) {
    pthread_mutex_lock(&q->lock);

    while (q->count > 0) {
        mining_notify * item = (mining_notify *) q->buffer[q->head];
        if (item != NULL) {
            STRATUM_V1_free_mining_notify(item);
        }
        q->buffer[q->head] = NULL;
        q->head = (q->head + 1) % QUEUE_SIZE;
        q->count--;
    }

    q->head = 0;
    q->tail = 0;
    q->count = 0;
    pthread_mutex_unlock(&q->lock);
}

void ASIC_jobs_queue_clear(work_queue *q) {
    while (q->count > 0) {
        bm_job * item = (bm_job *) q->buffer[q->head];
        if (item != NULL) {
            free_bm_job(item);
        }
        q->buffer[q->head] = NULL;
        q->head = (q->head + 1) % QUEUE_SIZE;
        q->count--;
    }

    q->head = 0;
    q->tail = 0;
    q->count = 0;
}
