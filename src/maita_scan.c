#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "bxi_pci_drv.h"
#include "maita_protocol.h"

typedef enum {
    SCAN_UNTESTED,
    SCAN_NO_REPLY,
    SCAN_TX_FAILED,
    SCAN_REPLIED,
    SCAN_CANCELLED,
    SCAN_WAIT_FAILED
} scan_result;

typedef struct {
    pthread_mutex_t mutex;
    pthread_cond_t condition;
    bool waiting;
    bool received;
    unsigned int bus;
    unsigned int id;
    uint8_t reply[8];
    uint64_t deadline_ns;
    uint64_t ignored;
    uint64_t can_errors[CANFD_DEVICE_NUM];
} scan_receiver;

static volatile sig_atomic_t scan_signal;

static void scan_on_signal(int signal_number)
{
    scan_signal = signal_number;
}

static uint64_t scan_now_ns(void)
{
    struct timespec now;

    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint64_t)now.tv_sec * 1000000000u + (uint64_t)now.tv_nsec;
}

static void scan_sleep_ms(unsigned int milliseconds)
{
    uint64_t deadline = scan_now_ns() + (uint64_t)milliseconds * 1000000u;

    while (!scan_signal) {
        uint64_t now = scan_now_ns();
        uint64_t remaining;
        struct timespec interval;

        if (now >= deadline) break;
        remaining = deadline - now;
        if (remaining > 20000000u) remaining = 20000000u;
        interval.tv_sec = 0;
        interval.tv_nsec = (long)remaining;
        nanosleep(&interval, NULL);
    }
}

static int scan_receive(void *argument, canfd_packet *packet)
{
    scan_receiver *receiver = argument;

    if (packet == NULL || receiver == NULL) return 0;
    pthread_mutex_lock(&receiver->mutex);
    if (packet->bus < CANFD_DEVICE_NUM && (packet->frame.can_id & CAN_ERR_FLAG) != 0u) {
        receiver->can_errors[packet->bus]++;
    } else if (receiver->waiting && !receiver->received &&
               scan_now_ns() <= receiver->deadline_ns &&
               packet->bus == receiver->bus &&
               packet->frame.can_id == MAITA_REPLY_BASE + receiver->id &&
               packet->frame.len == 8u && packet->frame.flags == 0u &&
               packet->frame.data[0] == MAITA_CMD_STATUS1) {
        memcpy(receiver->reply, packet->frame.data, sizeof(receiver->reply));
        receiver->received = true;
        pthread_cond_signal(&receiver->condition);
    } else {
        receiver->ignored++;
    }
    pthread_mutex_unlock(&receiver->mutex);
    return 0;
}

static scan_result scan_query(scan_receiver *receiver, unsigned int bus, unsigned int id,
                              unsigned int timeout_ms, uint8_t reply[8])
{
    canfd_packet packet = {0};
    scan_result result;
    int wait_result = 0;

    if (scan_signal) return SCAN_CANCELLED;
    packet.bus = bus;
    packet.frame.can_id = MAITA_SEND_BASE + id;
    packet.frame.len = 8u;
    packet.frame.data[0] = MAITA_CMD_STATUS1;
    pthread_mutex_lock(&receiver->mutex);
    receiver->bus = bus;
    receiver->id = id;
    receiver->received = false;
    receiver->waiting = true;
    receiver->deadline_ns = scan_now_ns() + (uint64_t)timeout_ms * 1000000u;
    pthread_mutex_unlock(&receiver->mutex);

    if (canfd_send_packet(&packet, 1u) < 0) {
        pthread_mutex_lock(&receiver->mutex);
        receiver->waiting = false;
        pthread_mutex_unlock(&receiver->mutex);
        return SCAN_TX_FAILED;
    }
    pthread_mutex_lock(&receiver->mutex);
    while (!receiver->received && !scan_signal) {
        uint64_t now = scan_now_ns();
        uint64_t wake;
        struct timespec deadline;

        if (now >= receiver->deadline_ns) break;
        wake = now + 20000000u;
        if (wake > receiver->deadline_ns) wake = receiver->deadline_ns;
        deadline.tv_sec = (time_t)(wake / 1000000000u);
        deadline.tv_nsec = (long)(wake % 1000000000u);
        wait_result = pthread_cond_timedwait(&receiver->condition, &receiver->mutex, &deadline);
        if (wait_result != 0 && wait_result != ETIMEDOUT) break;
    }
    receiver->waiting = false;
    if (scan_signal) {
        result = SCAN_CANCELLED;
    } else if (wait_result != 0 && wait_result != ETIMEDOUT) {
        result = SCAN_WAIT_FAILED;
    } else if (receiver->received) {
        memcpy(reply, receiver->reply, sizeof(receiver->reply));
        result = SCAN_REPLIED;
    } else {
        result = SCAN_NO_REPLY;
    }
    pthread_mutex_unlock(&receiver->mutex);
    return result;
}

static void scan_print_reply(const uint8_t reply[8])
{
    unsigned int byte;
    unsigned int voltage = (unsigned int)reply[4] | ((unsigned int)reply[5] << 8u);
    unsigned int error = (unsigned int)reply[6] | ((unsigned int)reply[7] << 8u);

    printf(" temp=%dC voltage=%.1fV brake=%s error=0x%04x data=",
           (int)(int8_t)reply[1], (double)voltage * 0.1,
           reply[3] ? "release" : "lock", error);
    for (byte = 0u; byte < 8u; byte++) printf("%s%02x", byte ? " " : "", reply[byte]);
    printf("\n");
}

static int scan_parse_uint(const char *text, unsigned int minimum, unsigned int maximum,
                           unsigned int *value)
{
    char *end;
    unsigned long parsed;

    if (text == NULL || text[0] < '0' || text[0] > '9') return -1;
    errno = 0;
    parsed = strtoul(text, &end, 10);
    if (errno != 0 || *end != '\0' || parsed < minimum || parsed > maximum) return -1;
    *value = (unsigned int)parsed;
    return 0;
}

static void scan_help(const char *program)
{
    printf("用法: %s [--timeout-ms 100] [--attempts 2] [--power-wait-ms 2000] [--keep-power]\n", program);
    printf("扫描 PCI/CAN 全部 %u 个端口(bus 0..%u)、脉塔 ID %u..%u，不依赖电机配置文件。\n",
           (unsigned int)CANFD_DEVICE_NUM, (unsigned int)CANFD_DEVICE_NUM - 1u,
           MAITA_CAN_ID_MIN, MAITA_CAN_ID_MAX);
    printf("仅发送 Classic CAN 标准帧 0x140+id: [9a 00 00 00 00 00 00 00]，匹配 0x240+id 的状态回复。\n");
    printf("  --timeout-ms      每次查询的回复超时，1..5000 ms\n");
    printf("  --attempts        无有效回复时最多尝试次数，1..10\n");
    printf("  --power-wait-ms   扫描前的上电等待，0..60000 ms\n");
    printf("  --keep-power      正常扫描结束后保留本程序开启的电源\n");
    printf("默认仅在电源原先关闭时上电，退出时恢复关闭；原先已上电则保持不变。\n");
    printf("不发送使能、MIT、运动、置零或参数写入。上电/下电影响公共电源，请先支撑负载。\n");
    printf("不要与 motor_console 或其他 PCI/CAN 控制程序同时运行；仅用于脉塔兼容总线。\n");
    printf("退出码: 0=扫描完成且有回复, 2=完成但无回复, 1=驱动/发送/清理失败, 128+信号=中断。\n");
}

int main(int argc, char **argv)
{
    static const struct option options[] = {
        {"timeout-ms", required_argument, NULL, 't'},
        {"attempts", required_argument, NULL, 'a'},
        {"power-wait-ms", required_argument, NULL, 'w'},
        {"keep-power", no_argument, NULL, 'k'},
        {"help", no_argument, NULL, 'h'},
        {NULL, 0, NULL, 0}
    };
    static scan_receiver receiver;
    scan_result results[CANFD_DEVICE_NUM][MAITA_CAN_ID_MAX + 1u] = {{SCAN_UNTESTED}};
    unsigned int timeout_ms = 100u;
    unsigned int attempts = 2u;
    unsigned int power_wait_ms = 2000u;
    unsigned int tx_failed = 0u;
    unsigned int replies = 0u;
    bool keep_power = false;
    bool powered_here = false;
    bool completed = false;
    pthread_condattr_t attributes;
    struct sigaction action = {0};
    int initial_power;
    int option;
    int result = 1;
    int driver_exit;
    unsigned int bus;
    unsigned int id;

    while ((option = getopt_long(argc, argv, "ht:a:w:k", options, NULL)) != -1) {
        if (option == 'h') {
            scan_help(argv[0]);
            return 0;
        } else if (option == 'k') {
            keep_power = true;
        } else if ((option == 't' && scan_parse_uint(optarg, 1u, 5000u, &timeout_ms) == 0) ||
                   (option == 'a' && scan_parse_uint(optarg, 1u, 10u, &attempts) == 0) ||
                   (option == 'w' && scan_parse_uint(optarg, 0u, 60000u, &power_wait_ms) == 0)) {
            continue;
        } else {
            scan_help(argv[0]);
            return 1;
        }
    }
    if (optind != argc) {
        scan_help(argv[0]);
        return 1;
    }
    setvbuf(stdout, NULL, _IOLBF, 0);
    action.sa_handler = scan_on_signal;
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT, &action, NULL) != 0 || sigaction(SIGTERM, &action, NULL) != 0) {
        perror("sigaction");
        return 1;
    }
    if (pthread_mutex_init(&receiver.mutex, NULL) != 0) return 1;
    if (pthread_condattr_init(&attributes) != 0) {
        pthread_mutex_destroy(&receiver.mutex);
        return 1;
    }
    if (pthread_condattr_setclock(&attributes, CLOCK_MONOTONIC) != 0 ||
        pthread_cond_init(&receiver.condition, &attributes) != 0) {
        pthread_condattr_destroy(&attributes);
        pthread_mutex_destroy(&receiver.mutex);
        return 1;
    }
    pthread_condattr_destroy(&attributes);
    printf("maita_scan: 只读扫描，不发送使能或运动；请勿同时运行其他 PCI/CAN 控制程序。\n");
    if (bxi_pci_init(scan_receive, &receiver, -1) < 0) {
        fprintf(stderr, "maita_scan: PCI/CAN 初始化失败\n");
        goto destroy_receiver;
    }
    initial_power = motor_pwr_get();
    if (initial_power != 0 && initial_power != 1) {
        fprintf(stderr, "maita_scan: 无法确认电源状态(%d)，未操作电源\n", initial_power);
        goto release_driver;
    }
    if (scan_signal) goto release_driver;
    if (initial_power == 0) {
        powered_here = true;
        if (motor_pwr_set(1u) < 0) {
            fprintf(stderr, "maita_scan: 上电失败\n");
            goto release_driver;
        }
    }
    printf("maita_scan: power=%s wait_ms=%u\n", powered_here ? "turned-on" : "already-on", power_wait_ms);
    scan_sleep_ms(power_wait_ms);
    printf("maita_scan: start buses=0..%u ids=%u..%u timeout_ms=%u attempts=%u\n",
           (unsigned int)CANFD_DEVICE_NUM - 1u, MAITA_CAN_ID_MIN, MAITA_CAN_ID_MAX, timeout_ms, attempts);
    for (bus = 0u; bus < CANFD_DEVICE_NUM && !scan_signal; bus++) {
        for (id = MAITA_CAN_ID_MIN; id <= MAITA_CAN_ID_MAX && !scan_signal; id++) {
            uint8_t reply[8];
            unsigned int attempt;
            bool sent = false;
            scan_result response = SCAN_UNTESTED;

            for (attempt = 1u; attempt <= attempts && !scan_signal; attempt++) {
                response = scan_query(&receiver, bus, id, timeout_ms, reply);
                if (response == SCAN_TX_FAILED) tx_failed++;
                else if (response != SCAN_CANCELLED) sent = true;
                if (response == SCAN_REPLIED || response == SCAN_CANCELLED || response == SCAN_WAIT_FAILED) break;
                scan_sleep_ms(2u);
            }
            if (scan_signal) {
                results[bus][id] = SCAN_CANCELLED;
                break;
            }
            if (response == SCAN_WAIT_FAILED) {
                fprintf(stderr, "maita_scan: 等待回复失败，扫描中止\n");
                goto summary;
            }
            if (attempt > attempts) attempt = attempts;
            results[bus][id] = response == SCAN_REPLIED ? SCAN_REPLIED :
                (sent ? SCAN_NO_REPLY : SCAN_TX_FAILED);
            printf("[bus%u id%02u] tx=0x%03x rx=0x%03x attempts=%u result=%s",
                   bus, id, MAITA_SEND_BASE + id, MAITA_REPLY_BASE + id, attempt,
                   response == SCAN_REPLIED ? "reply" : (sent ? "no_reply" : "tx_failed"));
            if (response == SCAN_REPLIED) {
                replies++;
                scan_print_reply(reply);
            } else {
                printf("\n");
            }
            scan_sleep_ms(2u);
        }
    }
    completed = scan_signal == 0;
    result = tx_failed ? 1 : (replies ? 0 : 2);
summary:
    pthread_mutex_lock(&receiver.mutex);
    receiver.waiting = false;
    for (bus = 0u; bus < CANFD_DEVICE_NUM; bus++) {
        unsigned int checked = 0u;
        unsigned int online = 0u;

        printf("[bus%u] responding_ids:", bus);
        for (id = MAITA_CAN_ID_MIN; id <= MAITA_CAN_ID_MAX; id++) {
            if (results[bus][id] == SCAN_REPLIED) {
                printf(" %u", id);
                online++;
            }
            if (results[bus][id] == SCAN_REPLIED || results[bus][id] == SCAN_NO_REPLY ||
                results[bus][id] == SCAN_TX_FAILED) checked++;
        }
        if (online == 0u) printf(" none");
        printf("; responding=%u checked=%u/%u CAN_errors=%llu\n", online, checked,
               MAITA_CAN_ID_MAX - MAITA_CAN_ID_MIN + 1u,
               (unsigned long long)receiver.can_errors[bus]);
    }
    printf("maita_scan: %s responding=%u tx_failed=%u ignored_rx=%llu\n",
           completed ? "complete" : "incomplete", replies, tx_failed,
           (unsigned long long)receiver.ignored);
    pthread_mutex_unlock(&receiver.mutex);
release_driver:
    if (powered_here && !(keep_power && completed && result != 1 && !scan_signal)) {
        if (motor_pwr_set(0u) < 0) {
            fprintf(stderr, "maita_scan: 恢复下电失败，请检查公共电源\n");
            result = 1;
        } else {
            printf("maita_scan: 已恢复下电（电源由本程序开启）\n");
        }
    } else if (powered_here) {
        printf("maita_scan: --keep-power 生效，公共电源保持开启\n");
    } else if (initial_power == 1) {
        printf("maita_scan: 电源原先已开启，保持不变\n");
    }
    driver_exit = bxi_pci_exit();
    if (driver_exit < 0) {
        fprintf(stderr, "maita_scan: PCI/CAN 释放失败\n");
        return 1;
    }
destroy_receiver:
    pthread_cond_destroy(&receiver.condition);
    pthread_mutex_destroy(&receiver.mutex);
    if (scan_signal) {
        printf("maita_scan: interrupted signal=%d\n", (int)scan_signal);
        return 128 + scan_signal;
    }
    return result;
}
