#include <sys/time.h>
#include <limits.h>

#include "work_queue.h"
#include "global_state.h"
#include "esp_log.h"
#include "esp_system.h"
#include "mining.h"
#include "string.h"
#include "utils.h"

#include "asic.h"

static const char *TAG = "create_jobs_task";

#define QUEUE_LOW_WATER_MARK 10 

static bool should_generate_more_work(GlobalState *GLOBAL_STATE)
{
    return GLOBAL_STATE->ASIC_jobs_queue.count < QUEUE_LOW_WATER_MARK;
}

void create_jobs_task(void *pvParameters)
{
    GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;

    uint32_t difficulty = GLOBAL_STATE->stratum_difficulty;
    while (1)
    {
        mining_notify *mining_notification = (mining_notify *)queue_dequeue(&GLOBAL_STATE->stratum_queue);
        if (mining_notification == NULL) {
            ESP_LOGE(TAG, "Failed to dequeue mining notification");
            vTaskDelay(100 / portTICK_PERIOD_MS); 
            continue;
        }

        ESP_LOGI(TAG, "New Work Dequeued %s", mining_notification->job_id);

        if (GLOBAL_STATE->abandon_work == 1) {
            GLOBAL_STATE->abandon_work = 0;
            xSemaphoreGive(GLOBAL_STATE->ASIC_TASK_MODULE.semaphore); 
        }
        
        if (GLOBAL_STATE->new_set_mining_difficulty_msg)
        {
            ESP_LOGI(TAG, "New pool difficulty %lu", GLOBAL_STATE->stratum_difficulty);
            difficulty = GLOBAL_STATE->stratum_difficulty;
            GLOBAL_STATE->new_set_mining_difficulty_msg = false;
        }

        if (GLOBAL_STATE->new_stratum_version_rolling_msg) {
            ESP_LOGI(TAG, "Set chip version rolls %i", (int)(GLOBAL_STATE->version_mask >> 13));
            ASIC_set_version_mask(GLOBAL_STATE, GLOBAL_STATE->version_mask);
            GLOBAL_STATE->new_stratum_version_rolling_msg = false;
        }

        // ====== [极致零拷贝]: 利用已经落地的纯二进制指针直接拼装模板 ======
        size_t cb1_bin_len = mining_notification->coinbase_1_len;
        size_t en1_bin_len = GLOBAL_STATE->extranonce_bin_len;
        size_t en2_bin_len = GLOBAL_STATE->extranonce_2_len;
        size_t cb2_bin_len = mining_notification->coinbase_2_len;
        size_t cb_tot_len = cb1_bin_len + en1_bin_len + en2_bin_len + cb2_bin_len;

        uint8_t *coinbase_bin = malloc(cb_tot_len);
        if (coinbase_bin != NULL) {
            // 全程 0 字符串操作，纯内存位移贴片！
            memcpy(coinbase_bin, mining_notification->coinbase_1_bin, cb1_bin_len);
            memcpy(coinbase_bin + cb1_bin_len, GLOBAL_STATE->extranonce_bin, en1_bin_len);
            // 中间自动留出 en2_bin_len 长度的空洞
            memcpy(coinbase_bin + cb1_bin_len + en1_bin_len + en2_bin_len, mining_notification->coinbase_2_bin, cb2_bin_len);
        } else {
            ESP_LOGE(TAG, "OOM generating coinbase bin template!");
            STRATUM_V1_free_mining_notify(mining_notification);
            continue;
        }
        // ==========================================================

        uint64_t extranonce_2 = 0;
        
        while (GLOBAL_STATE->stratum_queue.count < 1 && GLOBAL_STATE->abandon_work == 0)
        {
            if (should_generate_more_work(GLOBAL_STATE))
            {
                // [神级盲写]: ESP32 是小端序架构。直接把计数器的内存贴进模板空洞！
                memcpy(coinbase_bin + cb1_bin_len + en1_bin_len, &extranonce_2, en2_bin_len);

                // 专为提交日志与 JSON 上报保留一个短效十六进制字符串
                char *extranonce_2_str = malloc(en2_bin_len * 2 + 1);
                bin2hex((uint8_t*)&extranonce_2, en2_bin_len, extranonce_2_str, en2_bin_len * 2 + 1);

                // 纯二进制哈希，内部 0 拷贝
                uint8_t merkle_root_bin[32];
                calculate_merkle_root_hash_bin(coinbase_bin, cb_tot_len, 
                                               (const uint8_t(*)[32])mining_notification->merkle_branches, 
                                               mining_notification->n_merkle_branches, 
                                               merkle_root_bin);

                // 生成无字符串开销的下发结构
                bm_job next_job = construct_bm_job_bin(mining_notification, merkle_root_bin, GLOBAL_STATE->version_mask, difficulty);

                bm_job *queued_next_job = malloc(sizeof(bm_job));
                memcpy(queued_next_job, &next_job, sizeof(bm_job));
                queued_next_job->extranonce2 = extranonce_2_str; 
                queued_next_job->jobid = strdup(mining_notification->job_id);
                queued_next_job->version_mask = GLOBAL_STATE->version_mask;

                queue_enqueue(&GLOBAL_STATE->ASIC_jobs_queue, queued_next_job);

                extranonce_2++;
            }
            else
            {
                // 现在算得极快，队列瞬间塞满，等待时间降至 10ms，提高供块密度
                vTaskDelay(10 / portTICK_PERIOD_MS); 
            }
        }

        // 任务终结，销毁纯二进制模板
        free(coinbase_bin);
        STRATUM_V1_free_mining_notify(mining_notification);
    }
}
