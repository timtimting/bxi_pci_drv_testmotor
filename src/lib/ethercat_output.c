#include <stdarg.h>

#define ETHERCAT_OUTPUT_QUEUE_SIZE 512u
#define ETHERCAT_OUTPUT_CHUNK_SIZE 1024u

typedef struct {
    console_output_stamp stamp;
    char text[ETHERCAT_OUTPUT_CHUNK_SIZE];
} ethercat_output_message;

static pthread_mutex_t ethercat_output_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t ethercat_output_condition = PTHREAD_COND_INITIALIZER;
static pthread_t ethercat_output_thread;
static ethercat_output_message ethercat_output_queue[ETHERCAT_OUTPUT_QUEUE_SIZE];
static unsigned int ethercat_output_head;
static unsigned int ethercat_output_count;
static unsigned int ethercat_output_dropped;
static bool ethercat_output_started;
static bool ethercat_output_stopping;

static int console_ethercat_printf(const char *format, ...)
    __attribute__((format(printf, 1, 2)));

static int console_ethercat_printf(const char *format, ...)
{
    va_list arguments;
    char message[ETHERCAT_OUTPUT_CHUNK_SIZE];
    console_output_stamp stamp = console_output_capture();
    int length;

    va_start(arguments, format);
    pthread_mutex_lock(&ethercat_output_mutex);
    if (!ethercat_output_started) {
        pthread_mutex_unlock(&ethercat_output_mutex);
        length = console_output_vfprintf(stdout, format, arguments);
        va_end(arguments);
        return length;
    }
    length = vsnprintf(message, sizeof(message), format, arguments);
    va_end(arguments);
    if (length >= 0 && ethercat_output_count < ETHERCAT_OUTPUT_QUEUE_SIZE) {
        unsigned int slot = (ethercat_output_head + ethercat_output_count) %
                            ETHERCAT_OUTPUT_QUEUE_SIZE;
        ethercat_output_queue[slot].stamp = stamp;
        memcpy(ethercat_output_queue[slot].text, message,
               (size_t)length < sizeof(message) ? (size_t)length + 1u : sizeof(message));
        ethercat_output_count++;
        if ((size_t)length >= sizeof(message)) ethercat_output_dropped++;
    } else {
        ethercat_output_dropped++;
    }
    pthread_cond_signal(&ethercat_output_condition);
    pthread_mutex_unlock(&ethercat_output_mutex);
    return length;
}

static void *console_ethercat_output_worker(void *argument)
{
    (void)argument;
    console_output_defer_prompt = true;
    for (;;) {
        ethercat_output_message message;
        unsigned int dropped;
        bool drained;

        pthread_mutex_lock(&ethercat_output_mutex);
        while (ethercat_output_count == 0u && !ethercat_output_stopping) {
            pthread_cond_wait(&ethercat_output_condition, &ethercat_output_mutex);
        }
        if (ethercat_output_count == 0u && ethercat_output_stopping) {
            pthread_mutex_unlock(&ethercat_output_mutex);
            break;
        }
        message = ethercat_output_queue[ethercat_output_head];
        ethercat_output_head = (ethercat_output_head + 1u) % ETHERCAT_OUTPUT_QUEUE_SIZE;
        ethercat_output_count--;
        dropped = ethercat_output_dropped;
        ethercat_output_dropped = 0u;
        pthread_mutex_unlock(&ethercat_output_mutex);
        if (dropped) {
            char warning[96];
            int length = snprintf(warning, sizeof(warning),
                                  "\nEtherCAT: %u output chunks dropped/truncated\n", dropped);
            console_output_emit(stdout, warning, (size_t)length, message.stamp);
        }
        console_output_emit(stdout, message.text, strlen(message.text), message.stamp);
        fflush(stdout);
        pthread_mutex_lock(&ethercat_output_mutex);
        drained = ethercat_output_count == 0u;
        pthread_mutex_unlock(&ethercat_output_mutex);
        if (drained) console_output_prompt_refresh();
    }
    return NULL;
}

static int console_ethercat_output_start(void)
{
    int result = 0;

    pthread_mutex_lock(&ethercat_output_mutex);
    if (!ethercat_output_started) {
        ethercat_output_stopping = false;
        if (pthread_create(&ethercat_output_thread, NULL,
                            console_ethercat_output_worker, NULL) != 0) {
            result = -1;
        } else {
            ethercat_output_started = true;
        }
    }
    pthread_mutex_unlock(&ethercat_output_mutex);
    return result;
}

static void console_ethercat_output_stop(void)
{
    bool started;

    pthread_mutex_lock(&ethercat_output_mutex);
    started = ethercat_output_started;
    ethercat_output_stopping = true;
    pthread_cond_signal(&ethercat_output_condition);
    pthread_mutex_unlock(&ethercat_output_mutex);
    if (started) pthread_join(ethercat_output_thread, NULL);
    pthread_mutex_lock(&ethercat_output_mutex);
    ethercat_output_started = false;
    pthread_mutex_unlock(&ethercat_output_mutex);
}
