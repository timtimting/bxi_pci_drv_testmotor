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
    f = fopen(path, "w"); fprintf(f, "gear_ratio: 18\n"); fclose(f);
    assert(zero_cycle_load(path, &c) != 0); /* Half output turn is ambiguous. */
    f = fopen(path, "w"); fprintf(f, "gear_ratio: 20\nmax_motor_turns: 9.1\n"); fclose(f);
    assert(zero_cycle_load(path, &c) != 0);
    f = fopen(path, "w"); fprintf(f, "gear_ratio: 20\ncount: 0\nduration_s: 0\n"); fclose(f);
    assert(zero_cycle_load(path, &c) != 0);
    f = fopen(path, "w"); fprintf(f, "gear_ratio: nan\n"); fclose(f);
    assert(zero_cycle_load(path, &c) != 0);
    unlink(path);
    assert(zero_cycle_load("config/zero_cycle_test.yaml", &c) != 0); /* No guessed ratio. */
    puts("PASS: random sector coverage, reproducibility and invalid config rejection");
    return 0;
}
