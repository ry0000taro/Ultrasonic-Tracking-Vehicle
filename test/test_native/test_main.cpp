#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

void test_ci_scaffold_is_working(void)
{
    TEST_ASSERT_EQUAL(2, 1 + 1);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ci_scaffold_is_working);
    return UNITY_END();
}
