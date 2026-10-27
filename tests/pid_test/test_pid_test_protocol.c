#include "pid_test_protocol.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static int parse(const char *s, PID_TestProtocol_t *out)
{ return PID_TestProtocol_Parse(s, out); }

int main(void)
{
    PID_TestProtocol_t p = {0};
    assert(parse("PID START", &p) == 1 && p.command == PID_TEST_CMD_START);
    assert(parse("PID STOP", &p) == 1 && p.command == PID_TEST_CMD_STOP);
    assert(parse("PID PING", &p) == 1 && p.command == PID_TEST_CMD_PING);
    assert(parse("F", &p) == 0);
    assert(parse("PID START extra", &p) == -1);
    assert(parse("PID SET P:3 I:0 D:0 SPEED:25", &p) == 1);
    assert(p.command == PID_TEST_CMD_SET && p.p == 3.0f && p.speed_pwm_percent == 25.0f);
    assert(parse("PID SET P:3.2 I:0.05 D:0.01 SPEED:25.5", &p) == 1);
    assert(fabsf(p.p - 3.2f) < 0.0001f && fabsf(p.speed_pwm_percent - 25.5f) < 0.0001f);
    assert(parse("PID SET P:3,2 I:0,05 D:0,01 SPEED:25,5", &p) == 1);
    assert(fabsf(p.p - 3.2f) < 0.0001f && fabsf(p.i - 0.05f) < 0.0001f);
    assert(parse("PID SET P:1 I:2 D:3", &p) == -1);
    assert(parse("PID SET P:1 I:NaN D:0 SPEED:10", &p) == -1);
    assert(parse("PID SET P:Inf I:0 D:0 SPEED:10", &p) == -1);
    assert(parse("PID SET P:1e9999 I:0 D:0 SPEED:10", &p) == -1);
    assert(parse("PID SET P:1,2.3 I:0 D:0 SPEED:10", &p) == -1);
    assert(parse("PID SET P:-0.1 I:0 D:0 SPEED:10", &p) == -2);
    assert(parse("PID SET P:1 I:0 D:0 SPEED:31", &p) == -2);
    assert(parse("PID SET P:1 I:0 D:0 SPEED:10 extra", &p) == -1);
    puts("PASS: PID command grammar, decimal forms, finite values, atomic ranges and extras");
    return 0;
}
