#include <stdarg.h>
#include <time.h>

typedef struct {
    uint64_t sequence;
    uint64_t started_ns;
} console_output_context;

typedef struct {
    console_output_context context;
    uint64_t emitted_ns;
} console_output_stamp;

typedef struct {
    bool partial;
    uint64_t sequence;
} console_output_line;

static pthread_mutex_t console_output_context_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t console_output_stream_mutex = PTHREAD_MUTEX_INITIALIZER;
static console_output_context console_output_latest;
static console_output_line console_output_lines[2];
static __thread console_output_context console_output_current;
static __thread bool console_output_raw;

static uint64_t console_output_monotonic_ns(void)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return 0u;
    return (uint64_t)now.tv_sec * 1000000000u + (uint64_t)now.tv_nsec;
}

static void console_output_begin_command(void)
{
    uint64_t started_ns = console_output_monotonic_ns();

    pthread_mutex_lock(&console_output_context_mutex);
    console_output_latest.sequence++;
    console_output_latest.started_ns = started_ns;
    console_output_current = console_output_latest;
    pthread_mutex_unlock(&console_output_context_mutex);
}

static console_output_context console_output_get_context(void)
{
    console_output_context context = console_output_current;

    if (context.sequence == 0u) {
        pthread_mutex_lock(&console_output_context_mutex);
        context = console_output_latest;
        pthread_mutex_unlock(&console_output_context_mutex);
    }
    return context;
}

static console_output_stamp console_output_capture(void)
{
    console_output_stamp stamp = {0};

    if (!console_output_raw) stamp.context = console_output_get_context();
    stamp.emitted_ns = console_output_monotonic_ns();
    return stamp;
}

static int console_output_emit(FILE *stream, const char *message, size_t length,
                               console_output_stamp stamp)
{
    console_output_line *line;
    uint64_t elapsed_ms;
    size_t offset = 0u;
    int result = 0;

    if (stream != stdout && stream != stderr)
        return fwrite(message, 1u, length, stream) == length ? 0 : -1;
    elapsed_ms = stamp.emitted_ns >= stamp.context.started_ns ?
        (stamp.emitted_ns - stamp.context.started_ns) / 1000000u : 0u;
    pthread_mutex_lock(&console_output_stream_mutex);
    flockfile(stream);
    line = &console_output_lines[stream == stderr ? 1 : 0];
    if (length != 0u && stamp.context.sequence != 0u && line->partial &&
        line->sequence != stamp.context.sequence) {
        if (fputc('\n', stream) == EOF) result = -1;
        line->partial = false;
    }
    while (offset < length && result == 0) {
        size_t end = offset;

        while (end < length && message[end] != '\n' && message[end] != '\r') end++;
        if (!line->partial && end != offset && stamp.context.sequence != 0u) {
            if (fprintf(stream, "[cmd=%llu +%llu.%03llus] ",
                        (unsigned long long)stamp.context.sequence,
                        (unsigned long long)(elapsed_ms / 1000u),
                        (unsigned long long)(elapsed_ms % 1000u)) < 0) result = -1;
        }
        line->partial = end == length;
        line->sequence = stamp.context.sequence;
        if (end < length) end++;
        if (fwrite(message + offset, 1u, end - offset, stream) != end - offset) result = -1;
        offset = end;
    }
    funlockfile(stream);
    pthread_mutex_unlock(&console_output_stream_mutex);
    return result;
}

static int console_output_vfprintf(FILE *stream, const char *format, va_list arguments)
{
    console_output_stamp stamp = console_output_capture();
    char buffer[1024];
    char *message = buffer;
    va_list copy;
    int length;

    if (stream != stdout && stream != stderr) return vfprintf(stream, format, arguments);
    va_copy(copy, arguments);
    length = vsnprintf(buffer, sizeof(buffer), format, copy);
    va_end(copy);
    if (length < 0) return length;
    if ((size_t)length >= sizeof(buffer)) {
        message = malloc((size_t)length + 1u);
        if (message == NULL) return -1;
        vsnprintf(message, (size_t)length + 1u, format, arguments);
    }
    if (console_output_emit(stream, message, (size_t)length, stamp) != 0) length = -1;
    if (message != buffer) free(message);
    return length;
}

static int console_output_printf(const char *format, ...)
    __attribute__((format(printf, 1, 2)));
static int console_output_fprintf(FILE *stream, const char *format, ...)
    __attribute__((format(printf, 2, 3)));

static int console_output_printf(const char *format, ...)
{
    va_list arguments;
    int result;

    va_start(arguments, format);
    result = console_output_vfprintf(stdout, format, arguments);
    va_end(arguments);
    return result;
}

static int console_output_fprintf(FILE *stream, const char *format, ...)
{
    va_list arguments;
    int result;

    va_start(arguments, format);
    result = console_output_vfprintf(stream, format, arguments);
    va_end(arguments);
    return result;
}

static int console_output_putchar(int character)
{
    unsigned char byte = (unsigned char)character;

    return console_output_emit(stdout, (const char *)&byte, 1u,
                               console_output_capture()) == 0 ? byte : EOF;
}

static int console_output_fputs(const char *message, FILE *stream)
{
    return console_output_emit(stream, message, strlen(message),
                               console_output_capture()) == 0 ? 0 : EOF;
}

static int console_output_puts(const char *message)
{
    return console_output_printf("%s\n", message);
}

static void console_output_perror(const char *label)
{
    int saved_errno = errno;

    console_output_fprintf(stderr, "%s%s%s\n", label ? label : "",
                            label != NULL && label[0] != '\0' ? ": " : "",
                            strerror(saved_errno));
    errno = saved_errno;
}

#define printf(...) console_output_printf(__VA_ARGS__)
#define fprintf(...) console_output_fprintf(__VA_ARGS__)
#define putchar(...) console_output_putchar(__VA_ARGS__)
#define fputs(...) console_output_fputs(__VA_ARGS__)
#define puts(...) console_output_puts(__VA_ARGS__)
#define perror(...) console_output_perror(__VA_ARGS__)
