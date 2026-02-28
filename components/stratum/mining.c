#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include "mining.h"
#include "utils.h"
#include "mbedtls/sha256.h"

void free_bm_job(bm_job *job)
{
    free(job->jobid);
    free(job->extranonce2);
    free(job);
}

// [神级优化 1]: 数学抵消，用 8 次寄存器赋值完美等效替代原来的 swap_endian_words + reverse_bytes
static inline void fast_word_reverse_32(const uint8_t *src, uint8_t *dest) {
    const uint32_t *s = (const uint32_t *)src;
    uint32_t *d = (uint32_t *)dest;
    d[0] = s[7]; d[1] = s[6]; d[2] = s[5]; d[3] = s[4];
    d[4] = s[3]; d[5] = s[2]; d[6] = s[1]; d[7] = s[0];
}

// [神级优化 2]: 使用 GCC 内置硬件指令进行瞬间字内反转
static inline void swap_words_32bytes(const uint8_t *src, uint8_t *dest) {
    const uint32_t *s = (const uint32_t *)src;
    uint32_t *d = (uint32_t *)dest;
    for (int i = 0; i < 8; i++) d[i] = __builtin_bswap32(s[i]);
}

// 常规的数组前后倒序（用于部分必须按原样倒序的环节）
static inline void reverse_bytes_32(const uint8_t *src, uint8_t *dest) {
    for(int i=0; i<32; i++) dest[i] = src[31 - i];
}

// [神级优化 3]: In-Place 原地更新哈希，彻底消灭中间缓冲和拷贝
void calculate_merkle_root_hash_bin(const uint8_t *coinbase_bin, size_t coinbase_len, const uint8_t merkle_branches[][32], const int num_merkle_branches, uint8_t *merkle_root_bin)
{
    uint8_t both_merkles[64];
    
    // 第一重哈希：直接写入 both_merkles 前 32 字节
    double_sha256_bin(coinbase_bin, coinbase_len, both_merkles);

    for (int i = 0; i < num_merkle_branches; i++)
    {
        // 拼接分支到后 32 字节
        memcpy(both_merkles + 32, merkle_branches[i], 32);
        // [极速突破]：直接覆盖自身！把输入源当输出源
        double_sha256_bin(both_merkles, 64, both_merkles);
    }

    // 将最终结果推到输出指针
    memcpy(merkle_root_bin, both_merkles, 32);
}

// 剔除字符串，纯二进制组装 Job，无缝对接原有逻辑
bm_job construct_bm_job_bin(mining_notify *params, const uint8_t *merkle_root_bin, const uint32_t version_mask, const uint32_t difficulty)
{
    bm_job new_job;

    new_job.version = params->version;
    new_job.target = params->target;
    new_job.ntime = params->ntime;
    new_job.starting_nonce = 0;
    new_job.pool_diff = difficulty;

    // 1. 处理 Merkle Root
    memcpy(new_job.merkle_root, merkle_root_bin, 32);
    // 瞬间完成端序重排
    fast_word_reverse_32(new_job.merkle_root, new_job.merkle_root_be);

    // 2. 处理 Prev Block Hash 
    // [完美契合]: 这里直接使用我们在 stratum_api.c 中已经解析好的纯二进制 prev_block_hash_bin
    swap_words_32bytes(params->prev_block_hash_bin, new_job.prev_block_hash);
    reverse_bytes_32(params->prev_block_hash_bin, new_job.prev_block_hash_be);

    // ============================================
    // Midstate Hash
    uint8_t midstate_data[64];
    memcpy(midstate_data, &new_job.version, 4);
    memcpy(midstate_data + 4, new_job.prev_block_hash, 32); 
    memcpy(midstate_data + 36, new_job.merkle_root, 28); 
    
    midstate_sha256_bin(midstate_data, 64, new_job.midstate);
    reverse_bytes(new_job.midstate, 32);

    if (version_mask != 0) {
        uint32_t rolled_version = increment_bitmask(new_job.version, version_mask);
        memcpy(midstate_data, &rolled_version, 4);
        midstate_sha256_bin(midstate_data, 64, new_job.midstate1);
        reverse_bytes(new_job.midstate1, 32);

        rolled_version = increment_bitmask(rolled_version, version_mask);
        memcpy(midstate_data, &rolled_version, 4);
        midstate_sha256_bin(midstate_data, 64, new_job.midstate2);
        reverse_bytes(new_job.midstate2, 32);

        rolled_version = increment_bitmask(rolled_version, version_mask);
        memcpy(midstate_data, &rolled_version, 4);
        midstate_sha256_bin(midstate_data, 64, new_job.midstate3);
        reverse_bytes(new_job.midstate3, 32);
        new_job.num_midstates = 4;
    } else {
        new_job.num_midstates = 1;
    }

    return new_job;
}

static const double truediffone = 26959535291011309493156476344723991336010898738574164086137773096960.0;

double test_nonce_value(const bm_job *job, const uint32_t nonce, const uint32_t rolled_version)
{
    double d64, s64, ds;
    unsigned char header[80];

    memcpy(header, &rolled_version, 4);
    memcpy(header + 4, job->prev_block_hash, 32);
    memcpy(header + 36, job->merkle_root, 32);
    memcpy(header + 68, &job->ntime, 4);
    memcpy(header + 72, &job->target, 4);
    memcpy(header + 76, &nonce, 4);

    unsigned char hash_buffer[32];
    unsigned char hash_result[32];

    mbedtls_sha256(header, 80, hash_buffer, 0);
    mbedtls_sha256(hash_buffer, 32, hash_result, 0);

    d64 = truediffone;
    s64 = le256todouble(hash_result);
    ds = d64 / s64;

    return ds;
}

uint32_t increment_bitmask(const uint32_t value, const uint32_t mask)
{
    if (mask == 0) return value;
    uint32_t carry = (value & mask) + (mask & -mask);      
    uint32_t overflow = carry & ~mask;                     
    uint32_t new_value = (value & ~mask) | (carry & mask); 

    if (overflow > 0)
    {
        uint32_t carry_mask = (overflow << 1);                
        new_value = increment_bitmask(new_value, carry_mask); 
    }
    return new_value;
}
