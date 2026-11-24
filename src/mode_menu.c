#include "mode_menu.h"
/* Mode 9 replaces the old Mode 7 System Test in the same menu slot. */
static const char *const labels[7]={"1.Calibration","2.Run Slow","3.Run Faster","4.A* Run","5.Speedrun","6.Reset Map","9.System Test"};
static const char *const names[7]={"Calibration","Run Slow","Run Faster","A* Run","Speedrun","Reset Map","System Test"};
static const char *const codes[7]={"CALIBRATION","RUN_SLOW","RUN_FASTER","ASTAR_RUN","SPEEDRUN","RESET_MAP","SYSTEM_TEST"};
static const char *const groups[8]={"IR SENSOR TEST","ENCODER TEST","MPU TEST","MOTOR TEST","PID TEST","BLUETOOTH TEST","SYSTEM INTEGRATION","BACK"};
static const char *const ir[]={"IR Live Data","IR Filter","IR Burst Capture","IR Calibration","Back"};
static const char *const enc[]={"Log Receive","1 Box Forward","5 Box Forward","Pivot 90 Left","Pivot 90 Right","Pivot 180 Left","Pivot 180 Right","Encoder Speed","Back"};
static const char *const mpu[]={"Gyro Data","Yaw Monitor","MPU Calibration","I2C Status","Back"};
static const char *const motor[]={"Left Motor","Right Motor","Both Motors","PWM Sweep","Motor Driver","Back"};
static const char *const pid[]={"Speed PID","Turn PID","Wall PID","Back"};
static const char *const bt[]={"SPP Connection","BLE RX","Baudrate Test","SPP Throughput","Back"};
static const char *const integ[]={"Clock Validation","UART Benchmark","Timer/DMA","I2C Test","Flash Map Test","Back"};
static const char *const *const items[7]={ir,enc,mpu,motor,pid,bt,integ};
static const uint8_t counts[7]={5U,9U,5U,6U,4U,5U,6U};
uint8_t ModeMenu_ItemCount(uint8_t g){return g<7U?counts[g]:0U;}
const char *ModeMenu_TestName(uint8_t g){return g<8U?groups[g]:"INVALID";}
const char *ModeMenu_ItemName(uint8_t g,uint8_t i){return g<7U&&i<counts[g]?items[g][i]:"INVALID";}
const char *ModeMenu_StreamName(uint8_t s){switch(s){case 0:return "IR Live Data";case 1:return "Encoder Log Receive";case 2:return "MPU Gyro Data";default:return "No live stream";}}
const char *ModeMenu_Label(uint8_t i){return i<7U?labels[i]:"INVALID";}
const char *ModeMenu_Code(uint8_t i){return i<7U?codes[i]:"INVALID";}
const char *ModeMenu_Name(uint8_t i){return i<7U?names[i]:"INVALID";}
void ModeMenu_Init(ModeMenu*m,uint8_t pressed,uint32_t now){
 if(!m)return;m->focus=m->selected=m->test_focus=m->active_test=m->encoder_focus=m->active_encoder_option=m->active_group=m->active_item=MODE_MENU_NO_SELECTION;
 m->page=MODE_MENU_PAGE_MAIN;m->stream_running=0U;m->raw_pressed=pressed?1U:0U;m->stable_pressed=m->raw_pressed;m->armed=m->raw_pressed?0U:1U;m->long_sent=0U;m->raw_changed_ms=now;m->pressed_at_ms=now;
}
ModeMenuEvent ModeMenu_Next(ModeMenu*m){
 uint8_t n;if(!m)return MODE_MENU_EVENT_NONE;
 if(m->page==MODE_MENU_PAGE_ENCODER){m->active_test=m->active_encoder_option=m->active_item=MODE_MENU_NO_SELECTION;m->stream_running=0U;n=ModeMenu_ItemCount(m->active_group);if(!n)return MODE_MENU_EVENT_NONE;m->encoder_focus=m->encoder_focus==MODE_MENU_NO_SELECTION?0U:(uint8_t)((m->encoder_focus+1U)%n);return MODE_MENU_EVENT_ENCODER_FOCUS;}
 if(m->page==MODE_MENU_PAGE_TEST){m->active_test=m->active_group=m->active_item=MODE_MENU_NO_SELECTION;m->stream_running=0U;m->test_focus=m->test_focus==MODE_MENU_NO_SELECTION?0U:(uint8_t)((m->test_focus+1U)%8U);return MODE_MENU_EVENT_TEST_FOCUS;}
 m->focus=m->focus==MODE_MENU_NO_SELECTION?0U:(uint8_t)((m->focus+1U)%7U);return MODE_MENU_EVENT_FOCUS;
}
ModeMenuEvent ModeMenu_Select(ModeMenu*m){
 uint8_t n;if(!m)return MODE_MENU_EVENT_NONE;
 if(m->page==MODE_MENU_PAGE_ENCODER){n=ModeMenu_ItemCount(m->active_group);if(m->encoder_focus==MODE_MENU_NO_SELECTION||!n)return MODE_MENU_EVENT_NONE;
  m->active_test=m->active_encoder_option=m->active_item=MODE_MENU_NO_SELECTION;m->stream_running=0U;
  if(m->encoder_focus==(uint8_t)(n-1U)){m->page=MODE_MENU_PAGE_TEST;m->encoder_focus=MODE_MENU_NO_SELECTION;return MODE_MENU_EVENT_ENCODER_EXIT;}
  m->active_item=m->active_encoder_option=m->encoder_focus;
  if(m->active_group==0U&&m->active_item==0U){m->active_test=0U;m->stream_running=1U;}
  else if(m->active_group==1U&&m->active_item==0U)m->active_test=1U;
  else if(m->active_group==2U&&(m->active_item==0U||m->active_item==1U)){m->active_test=2U;m->stream_running=1U;}
  return MODE_MENU_EVENT_ENCODER_SELECT;
 }
 if(m->page==MODE_MENU_PAGE_TEST){if(m->test_focus==MODE_MENU_NO_SELECTION)return MODE_MENU_EVENT_NONE;
  if(m->test_focus==7U){m->page=MODE_MENU_PAGE_MAIN;m->active_test=m->active_group=m->active_item=MODE_MENU_NO_SELECTION;m->stream_running=0U;m->test_focus=MODE_MENU_NO_SELECTION;return MODE_MENU_EVENT_TEST_EXIT;}
  m->active_group=m->test_focus;m->page=MODE_MENU_PAGE_ENCODER;m->encoder_focus=m->active_encoder_option=m->active_item=m->active_test=MODE_MENU_NO_SELECTION;m->stream_running=0U;return MODE_MENU_EVENT_ENCODER_ENTER;
 }
 if(m->focus==MODE_MENU_NO_SELECTION)return MODE_MENU_EVENT_NONE;m->selected=m->focus;
 if(m->selected==6U){m->page=MODE_MENU_PAGE_TEST;m->test_focus=m->active_test=m->active_group=m->active_item=m->encoder_focus=m->active_encoder_option=MODE_MENU_NO_SELECTION;m->stream_running=0U;return MODE_MENU_EVENT_TEST_ENTER;}
 return MODE_MENU_EVENT_SELECT;
}
uint8_t ModeMenu_PauseTest(ModeMenu*m){if(!m||m->page!=MODE_MENU_PAGE_ENCODER||m->active_test==MODE_MENU_NO_SELECTION||!m->stream_running)return 0U;m->stream_running=0U;return 1U;}
uint8_t ModeMenu_ResumeTest(ModeMenu*m){if(!m||m->page!=MODE_MENU_PAGE_ENCODER||m->active_test==MODE_MENU_NO_SELECTION||m->stream_running)return 0U;m->stream_running=1U;return 1U;}
uint8_t ModeMenu_CanResetSensor(const ModeMenu*m,uint8_t s){
 if(!m||m->page!=MODE_MENU_PAGE_ENCODER||m->active_test==MODE_MENU_NO_SELECTION)return 0U;
 if(s==0U)return m->active_group==0U&&m->active_item==0U;
 if(s==1U)return m->active_group==1U&&m->active_item==0U;
 if(s==2U)return m->active_group==2U&&(m->active_item==0U||m->active_item==1U);
 return 0U;
}
ModeMenuEvent ModeMenu_Poll(ModeMenu*m,uint8_t pressed,uint32_t now){
 uint8_t raw;if(!m)return MODE_MENU_EVENT_NONE;raw=pressed?1U:0U;
 if(raw!=m->raw_pressed){m->raw_pressed=raw;m->raw_changed_ms=now;}
 if(raw!=m->stable_pressed&&(uint32_t)(now-m->raw_changed_ms)>=MODE_MENU_DEBOUNCE_MS){m->stable_pressed=raw;
  if(raw){m->pressed_at_ms=now;m->long_sent=0U;}else{if(!m->armed){m->armed=1U;return MODE_MENU_EVENT_NONE;}
   if(!m->long_sent){if((uint32_t)(now-m->pressed_at_ms)>=MODE_MENU_LONG_PRESS_MS){m->long_sent=1U;return ModeMenu_Select(m);}return ModeMenu_Next(m);}}
 }
 if(m->armed&&m->stable_pressed&&!m->long_sent&&(uint32_t)(now-m->pressed_at_ms)>=MODE_MENU_LONG_PRESS_MS){m->long_sent=1U;return ModeMenu_Select(m);}
 return MODE_MENU_EVENT_NONE;
}
