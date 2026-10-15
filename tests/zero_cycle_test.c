#define MOTOR_CONSOLE_TEST
#include "../src/main.c"
#include <assert.h>
int main(void)
{
    unsigned int seed = 1234, replay = seed;
    int sectors[36] = {0};
    for (unsigned int i = 0; i < 36; ++i) {
        double target = zero_cycle_target(i, &seed, 9);
        assert(target > -9 && target < 9);
        assert(target == zero_cycle_target(i, &replay, 9));
        int sector = (int)((target + 9) * 2);
        assert(sector >= 0 && sector < 36);
        assert(!sectors[sector]++);
    }
    char path[] = "/tmp/zero-cycle-config-XXXXXX";
    int fd = mkstemp(path); assert(fd >= 0);
    FILE *f = fdopen(fd, "w"); assert(f);
    fprintf(f, "gear_ratio: 19.54570637\ncount: 0\nduration_s: 60\n"); fclose(f);
    zero_cycle_config c;
    assert(zero_cycle_load(path, &c) == 0 && c.count == 0 && c.duration_s == 60);
    bxi_motor_limits limits = bxi_motor_default_limits;
    limits.v_min = -45; limits.v_max = 45;
    double decoded_zero_speed = (limits.v_max - limits.v_min) / 4095. / 2 * c.ratio / (2 * M_PI);
    assert(decoded_zero_speed > c.tolerance); /* Old criterion never settled. */
    assert(decoded_zero_speed < zero_cycle_speed_threshold(&limits, &c));
    assert(decoded_zero_speed * 2 > zero_cycle_speed_threshold(&limits, &c));
    f = fopen(path, "w"); fprintf(f, "gear_ratio: 18\n"); fclose(f);
    assert(zero_cycle_load(path, &c) != 0); /* Half output turn is ambiguous. */
    f = fopen(path, "w"); fprintf(f, "gear_ratio: 20\nmax_motor_turns: 9.1\n"); fclose(f);
    assert(zero_cycle_load(path, &c) != 0);
    f = fopen(path, "w"); fprintf(f, "gear_ratio: 20\ncount: 0\nduration_s: 0\n"); fclose(f);
    assert(zero_cycle_load(path, &c) != 0);
    f = fopen(path, "w"); fprintf(f, "gear_ratio: nan\n"); fclose(f);
    assert(zero_cycle_load(path, &c) != 0);
    unlink(path);
    assert(zero_cycle_load("config/zero_cycle_test.yaml", &c) == 0);
    assert(c.count == 5 && c.speed == 10 && c.kp == 50 && c.kd == 5 && c.tolerance == .1);
    custom_command_words = console_command_words;
    custom_command_word_count = sizeof(console_command_words) / sizeof(console_command_words[0]);
    const char *line = "zero_cycle_test 01 config/zero_c";
    size_t count = 0;
    const char *const *words = completion_words_for_line(line, strlen(line), &count);
    assert(words && count == 1 && !strcmp(words[0], "config/zero_cycle_test.yaml"));
    line = "zero_cycle_test 01 confi";
    words = completion_words_for_line(line, strlen(line), &count);
    assert(words && count == 1 && !strcmp(words[0], "config/"));
    char input[256] = "zero_cycle_test 01 confi";
    size_t length = strlen(input), cursor = length;
    complete_line(input, &length, &cursor, sizeof(input), "");
    assert(!strcmp(input, "zero_cycle_test 01 config/")); /* No space after directory. */
    puts("PASS: random sector coverage, reproducibility and invalid config rejection");
    return 0;
}
