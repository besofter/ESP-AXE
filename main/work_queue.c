#include "work_queue.h"
#include "esp_log.h"
#include "stratum_api.h"

void queue_init(work_queue *queue)
{
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    pthread_mutex_init(&queue->lock, NULL);
    pthread_cond_init(&queue->not_empty, NULL);
    pthread_cond_init(&queue->not_full, NULL);
}

void queue_enqueue(work_queue *queue, void *new_work)
{
    pthread_mutex_lock(&queue->lock);

    // 如果队列满了，休眠等待，直到被 dequeue 唤醒
    while (queue->count == QUEUE_SIZE)
    {
        pthread_cond_wait(&queue->not_full, &queue->lock);
    }

    queue->buffer[queue->tail] = new_work;
    queue->tail = (queue->tail + 1) % QUEUE_SIZE;
    queue->count++;

    // 唤醒因为没有任务而在等待的 dequeue 线程
    pthread_cond_signal(&queue->not_empty);
    pthread_mutex_unlock(&queue->lock);
}

void *queue_dequeue(work_queue *queue)
{
    pthread_mutex_lock(&queue->lock);

    while (queue->count == 0)
    {
        pthread_cond_wait(&queue->not_empty, &queue->lock);
    }

    void *next_work = queue->buffer[queue->head];
    queue->head = (queue->head + 1) % QUEUE_SIZE;
    queue->count--;

    // 唤醒因为满载而在等待的 enqueue 线程
    pthread_cond_signal(&queue->not_full);
    pthread_mutex_unlock(&queue->lock);

    return next_work;
}

void queue_clear(work_queue *queue)
{
    pthread_mutex_lock(&queue->lock);

    while (queue->count > 0)
    {
        mining_notify *next_work = (mining_notify *)queue->buffer[queue->head];
        if (next_work != NULL) {
            // 使用新版专门释放二进制指针的函数
            STRATUM_V1_free_mining_notify(next_work);
        }
        queue->head = (queue->head + 1) % QUEUE_SIZE;
        queue->count--;
    }

    pthread_cond_signal(&queue->not_full);
    pthread_mutex_unlock(&queue->lock);
}

void ASIC_jobs_queue_clear(work_queue *queue)
{
    pthread_mutex_lock(&queue->lock);

    while (queue->count > 0)
    {
        bm_job *next_work = (bm_job *)queue->buffer[queue->head];
        if (next_work != NULL) {
            // 调用优化后的安全释放接口
            free_bm_job(next_work);
        }
        queue->head = (queue->head + 1) % QUEUE_SIZE;
        queue->count--;
    }

    pthread_cond_signal(&queue->not_full);
    pthread_mutex_unlock(&queue->lock);
}
