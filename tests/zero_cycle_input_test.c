#define MOTOR_CONSOLE_TEST
#include "../src/main.c"
#include <assert.h>
#include <pty.h>

int main(void)
{
    int master, slave, original_stdin = dup(STDIN_FILENO);
    assert(original_stdin >= 0 && openpty(&master, &slave, NULL, NULL, NULL) == 0);
    assert(dup2(slave, STDIN_FILENO) >= 0);
    struct termios before, during, after;
    assert(tcgetattr(STDIN_FILENO, &before) == 0);
    assert(zero_cycle_input_begin() == 0);
    assert(tcgetattr(STDIN_FILENO, &during) == 0);
    assert(!(during.c_lflag & ICANON) && !(during.c_lflag & ECHO));
    assert(!zero_cycle_cancel_requested());
    assert(write(master, "qQx", 3) == 3);
    usleep(1000);
    assert(!zero_cycle_cancel_requested());
    assert(write(master, "\x18", 1) == 1); /* Ctrl+X, no newline. */
    usleep(1000);
    assert(zero_cycle_cancel_requested() && !stop_requested);
    zero_cycle_input_end();
    assert(tcgetattr(STDIN_FILENO, &after) == 0 && after.c_lflag == before.c_lflag);
    assert(zero_cycle_input_begin() == 0 && !zero_cycle_cancel_requested());
    assert(write(master, "\x18\n", 2) == 2);
    usleep(1000);
    assert(zero_cycle_cancel_requested() && !stop_requested);
    zero_cycle_input_end();
    assert(dup2(original_stdin, STDIN_FILENO) >= 0);
    close(original_stdin); close(master); close(slave);
    puts("PASS: Ctrl+X cancels without Enter; q/Q do not cancel; terminal restored and console stop flag preserved");
    return 0;
}
