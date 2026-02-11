#pragma once

enum class time_mode_t{
    NONE = 0,
    RTC,
    NTP
};
enum class play_mode_t{
    STOP = 0 ,
    ARMED,
    RECORD,
    ERROR
};
enum class smpl_t{
    KNONE =0,
    K44,    //44100
    K48,    //48000
    K88,    //88200
    K96,    //96000
    K176,   //176400
    K192    //192000
};

//oled variables
//-- top status row
extern bool        oled_isRx;
extern bool        oled_24bit;
extern smpl_t      oled_samples;
extern uint64_t    oled_free;
extern bool        oled_asplit;
extern time_mode_t oled_time;
extern uint8_t     oled_buff;
//--- main mode and time double row
extern uint32_t    oled_volL;
extern uint32_t    oled_volR;
extern play_mode_t oled_mode;
extern uint16_t    oled_hour;
extern uint8_t     oled_min;
extern uint8_t     oled_sec;
extern uint8_t     oled_frame;     // 1/75th of a sec (works even 44k1 and 48k samples -- even if it is not necessary)
//--- bottom filename row
extern char       oled_fnam[28];

