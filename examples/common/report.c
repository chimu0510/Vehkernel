#include "report.h"

#include <stdarg.h>
#include <stdio.h>

#include "board.h"

static unsigned s_pass;
static unsigned s_fail;
static unsigned s_skip;

void report_begin(const char *example)
{
    s_pass = s_fail = s_skip = 0;
    printf("\n==== %s ====\n", example);
}

static void print_line(const char *tag, const char *name, const char *fmt, va_list ap)
{
    printf("[%s] %s: ", tag, name);
    vprintf(fmt, ap);
    printf("\n");
}

bool report_check(bool ok, const char *name, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    print_line(ok ? "PASS" : "FAIL", name, fmt, ap);
    va_end(ap);
    if (ok) {
        s_pass++;
    } else {
        s_fail++;
    }
    return ok;
}

void report_skip(const char *name, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    print_line("SKIP", name, fmt, ap);
    va_end(ap);
    s_skip++;
}

bool report_end(void)
{
    const bool ok = (s_fail == 0);
    printf("RESULT: %s (%u/%u passed, %u skipped)\n",
           ok ? "PASS" : "FAIL", s_pass, s_pass + s_fail, s_skip);
    led_set(!ok, ok, false);
    return ok;
}

void report_halt(void)
{
    printf("done. press RESET (or run: tools/c50c.py reset) to run again\n");
    for (;;) {
    }
}
