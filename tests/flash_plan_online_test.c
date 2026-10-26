#define MOTOR_CONSOLE_TEST
#include "../src/main.c"
#include <assert.h>

static flash_state fixture;

int main(void)
{
    flash_plan_config plan;
    size_t target_indices[FLASH_PLAN_MAX_TARGETS];
    size_t slots[FLASH_PLAN_MAX_TARGETS];
    size_t selected_count;
    size_t index;

    assert(load_motor_map_config("config/motor_console.yaml", &fixture.config) == 0);
    assert(console_load_default_topology(&fixture.config) == 0);
    assert(load_flash_plan_config("config/flash_plan_default.yaml", &plan) == 0);
    assert(fixture.config.entry_count == 33u);
    assert(plan.target_count == 33u);

    for (index = 0u; index < 31u; index++) {
        fixture.motors[index].online = true;
    }
    selected_count = console_flash_online_targets(&fixture, &plan, target_indices, slots);
    assert(selected_count == 31u);
    for (index = 0u; index < selected_count; index++) {
        assert(plan.targets[target_indices[index]].index == index);
        assert(fixture.config.entries[slots[target_indices[index]]].index == index);
    }

    fixture.motors[31].online = true;
    selected_count = console_flash_online_targets(&fixture, &plan, target_indices, slots);
    assert(selected_count == 32u);
    assert(plan.targets[target_indices[31]].index == 31u);
    fixture.motors[31].online = false;

    plan.targets[33].index = 33u;
    plan.targets[33].bus = 6u;
    plan.targets[33].id = 8u;
    plan.target_count++;
    selected_count = console_flash_online_targets(&fixture, &plan, target_indices, slots);
    assert(selected_count == 31u);
    plan.target_count--;

    fixture.motors[21].online = false;
    selected_count = console_flash_online_targets(&fixture, &plan, target_indices, slots);
    assert(selected_count == 30u);
    for (index = 0u; index < selected_count; index++) {
        assert(plan.targets[target_indices[index]].index != 21u);
    }

    for (index = 0u; index < fixture.config.entry_count; index++) {
        fixture.motors[index].online = false;
    }
    assert(console_flash_online_targets(&fixture, &plan, target_indices, slots) == 0u);

    puts("PASS: flash plan selects only online configured motors");
    return 0;
}
