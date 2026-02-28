#include <lwip/tcpip.h>

#include "system.h"
#include "work_queue.h"
#include "serial.h"
#include "bm1397.h"
#include <string.h>
#include "esp_log.h"
#include "nvs_config.h"
#include "utils.h"
#include "stratum_task.h"
#include "asic.h"

static const char *TAG = "asic_result";

void ASIC_result_task(void *pvParameters)
{
    GlobalState *GLOBAL_STATE = (GlobalState *)pvParameters;

    while (1)
    {
        task_result *asic_result = ASIC_process_work(GLOBAL_STATE);

        if (asic_result == NULL)
        {
            continue;
        }

        uint8_t job_id = asic_result->job_id;

        // [修复 1]: 获取互斥锁，确保在读取 active_job 期间它不会被清理线程 free 掉
        pthread_mutex_lock(&GLOBAL_STATE->valid_jobs_lock);
        
        if (GLOBAL_STATE->valid_jobs[job_id] == 0 || GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[job_id] == NULL)
        {
            pthread_mutex_unlock(&GLOBAL_STATE->valid_jobs_lock);
            ESP_LOGW(TAG, "Invalid job nonce found, 0x%02X", job_id);
            continue;
        }
        
        bm_job *active_job = GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[job_id];
        
        // 在锁保护下进行 Hash 校验
        double nonce_diff = test_nonce_value(active_job, asic_result->nonce, asic_result->rolled_version);
        
        double pool_diff = active_job->pool_diff;
        uint32_t active_ntime = active_job->ntime;
        uint32_t active_version = active_job->version;

        // [修复 1]: 必须深拷贝字符串数据到栈空间，因为后续的 TCP 发送非常耗时，不能占用锁
        char jobid_copy[32] = {0};
        if(active_job->jobid) {
            strncpy(jobid_copy, active_job->jobid, sizeof(jobid_copy)-1);
        }
        
        char extranonce2_copy[64] = {0};
        if(active_job->extranonce2) {
            strncpy(extranonce2_copy, active_job->extranonce2, sizeof(extranonce2_copy)-1);
        }

        // 数据拷贝完成，立即释放锁，不阻塞后续的新块接收
        pthread_mutex_unlock(&GLOBAL_STATE->valid_jobs_lock);

        //log the ASIC response
        ESP_LOGI(TAG, "ID: %s, ver: %08" PRIX32 " Nonce %08" PRIX32 " diff %.1f of %.1f.", jobid_copy, asic_result->rolled_version, asic_result->nonce, nonce_diff, pool_diff);

        if (nonce_diff >= pool_diff)
        {
            if (GLOBAL_STATE->sock >= 0 && !GLOBAL_STATE->abandon_work) {
                char * user = GLOBAL_STATE->SYSTEM_MODULE.is_using_fallback ? GLOBAL_STATE->SYSTEM_MODULE.fallback_pool_user : GLOBAL_STATE->SYSTEM_MODULE.pool_user;
                
                // 使用拷贝出来的数据发送给网络，杜绝 Use-After-Free
                int ret = STRATUM_V1_submit_share(
                    GLOBAL_STATE->sock,
                    GLOBAL_STATE->send_uid++,
                    user,
                    jobid_copy,
                    extranonce2_copy,
                    active_ntime,
                    asic_result->nonce,
                    asic_result->rolled_version ^ active_version);

                if (ret < 0) {
                    ESP_LOGI(TAG, "Unable to write share to socket. Closing connection. Ret: %d (errno %d: %s)", ret, errno, strerror(errno));
                    stratum_close_connection(GLOBAL_STATE);
                }
            } else {
                ESP_LOGW(TAG, "Socket unavailable or abandoning work, dropping share to prevent crash...");
            }
        }

        SYSTEM_notify_found_nonce(GLOBAL_STATE, nonce_diff, job_id);
    }
}
