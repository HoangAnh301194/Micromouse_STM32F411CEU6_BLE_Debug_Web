#include "mode_test_resources.h"

uint8_t ModeTest_OneBoxArmed(const ModeMenu *menu)
{
    return (menu != 0 &&
            menu->page == MODE_MENU_PAGE_ENCODER &&
            menu->active_encoder_option == MODE_MENU_ENCODER_1BOX) ? 1U : 0U;
}

uint8_t ModeTest_Resources(const ModeMenu *menu)
{
    if (!menu) return 0U;
    /* Main mode 1 explicitly grants IR/MPU/encoder sampling and motor
     * capability; the calibration FSM is the sole actuator authorizer. */
    if (menu->page == MODE_MENU_PAGE_MAIN && menu->selected == 0U && menu->focus == 0U)
        return TEST_RES_IR | TEST_RES_MPU | TEST_RES_ENCODER | TEST_RES_MOTOR;
    if (menu->page == MODE_MENU_PAGE_TEST) {
        switch (menu->active_test) {
        case MODE_MENU_TEST_IR:  return TEST_RES_IR;
        case MODE_MENU_TEST_MPU: return TEST_RES_MPU;
        case MODE_MENU_TEST_ALL:
            return TEST_RES_IR | TEST_RES_MPU |
                   TEST_RES_ENCODER | TEST_RES_MOTOR;
        default: return 0U;
        }
    }
    if (menu->page == MODE_MENU_PAGE_MPU) {
        if (ModeMenu_PivotTarget(menu)) return TEST_RES_MPU | TEST_RES_ENCODER | TEST_RES_MOTOR;
        if (menu->active_test == MODE_MENU_TEST_MPU) return TEST_RES_MPU;
    }
    if (menu->page == MODE_MENU_PAGE_ENCODER) {
        if (menu->active_encoder_option == MODE_MENU_ENCODER_LOG_RECEIVE)
            return TEST_RES_ENCODER;
        if (ModeTest_OneBoxArmed(menu))
            return TEST_RES_ENCODER | TEST_RES_MOTOR;
        /* Unimplemented forward/pivot choices do not acquire motor access. */
    }
    return 0U;
}
