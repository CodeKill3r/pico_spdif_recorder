/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#pragma once

#include <string>

#include "ff.h"
#include "spdif_rec_wav.h"


#pragma pack(push,4)

class wav_file
{
public:
    // === Public class constants ===

    // === Public class functions ===
    // functions called from core0

    // === Constructor and Destructor ===
    // called from core0
    wav_file(const uint32_t suffix, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample);
    virtual ~wav_file();

    // === Public member functions ===
    // functions called from core1
    uint32_t write(const uint32_t* buff, const uint32_t sub_frame_count);
    void set_truncate(const float sec);
    void record_queue_ratio(float queue_ratio);
    void report_start() const;
    void report_final() const;
    bool is_data_written() const;
    struct tm wav_rtc;

protected:
    // === Private class constants ===
    static constexpr int NUM_CHANNELS = spdif_rec_wav::NUM_CHANNELS;
    static constexpr int NUM_SUB_FRAME_BUF = spdif_rec_wav::NUM_SUB_FRAME_BUF;
    static constexpr const char* WAV_PREFIX = "record_";
    static constexpr const char* WAV_EXT = ".wav\0";
#ifdef W64
    static constexpr const char* WAV64_EXT = ".w64\0";

    typedef struct {
        char ckID [16];
        int64_t ckSize;
        char formType [16];
    } Wave64FileHeader;

    typedef struct {
        char ckID [16];
        int64_t ckSize;
    } Wave64ChunkHeader;

    typedef struct {
        uint16_t FormatTag, NumChannels;        //4
        uint32_t SampleRate, BytesPerSecond;    //12
        uint16_t BlockAlign, BitsPerSample;     //16
        //////only would need for advanced (2+ ch) audio files -- not used here
        //
        // uint16_t cbSize, ValidBitsPerSample;
        // int32_t ChannelMask;
        // uint16_t SubFormat;
        // char GUID [14];
    } WaveHeader;

    static constexpr unsigned char riff_guid [16] = { 'r','i','f','f', 0x2e,0x91,0xcf,0x11,0xa5,0xd6,0x28,0xdb,0x04,0xc1,0x00,0x00 };
    static constexpr unsigned char wave_guid [16] = { 'w','a','v','e', 0xf3,0xac,0xd3,0x11,0x8c,0xd1,0x00,0xc0,0x4f,0x8e,0xdb,0x8a };
    static constexpr unsigned char  fmt_guid [16] = { 'f','m','t',' ', 0xf3,0xac,0xd3,0x11,0x8c,0xd1,0x00,0xc0,0x4f,0x8e,0xdb,0x8a };
    static constexpr unsigned char data_guid [16] = { 'd','a','t','a', 0xf3,0xac,0xd3,0x11,0x8c,0xd1,0x00,0xc0,0x4f,0x8e,0xdb,0x8a };
#else   //RF64
    typedef struct {
        char ckID [4];
        uint32_t ckSize;
        char formType [4];
    } RiffChunkHeader;

    typedef struct {
        char ckID [4];
        uint32_t ckSize;
    } ChunkHeader;

    typedef struct {
        char ckID [4];
        uint64_t chunkSize64;
    } CS64Chunk;

    typedef struct {
        uint64_t riffSize64, dataSize64, sampleCount64;
        uint32_t tableLength;
    } DS64Chunk;

    typedef struct {
        uint16_t FormatTag, NumChannels;
        uint32_t SampleRate, BytesPerSecond;
        uint16_t BlockAlign, BitsPerSample;
        //////only would need for advanced (2+ ch) audio files -- not used here
        //
        // uint16_t cbSize, ValidBitsPerSample;
        // int32_t ChannelMask;
        // uint16_t SubFormat;
        // char GUID [14];
    } WaveHeader;


#endif
    static constexpr int WAV_HEADER_SIZE = 128;     //44; -- have some more space
    static constexpr int FNAMLEN = 36;  //maximum leght of filename
    static constexpr uint32_t MAX_TOTAL_BYTES = 0xfff00000;  // max total bytes of wav data to avoid 32bit overflow
    static constexpr int64_t SEEK_STEP_BYTES = 10 * 1024 * 1024;  // 10MB

    // === Private class functions ===

    // === Private class variables ===
    static uint32_t _wav_buf[SPDIF_BLOCK_SIZE*3/4 * NUM_SUB_FRAME_BUF / 2];

    // === Private member functions ===
    // functions called from core0
    FRESULT _stepwise_seek(const FSIZE_t pos);
    // functions called from core1
    uint32_t _write_core(const uint32_t* buff, const uint32_t sub_frame_count);

    // === Private member variables ===
    FIL                     _fil;
    std::string             _filename;
    const uint32_t          _sample_freq;
    const bits_per_sample_t _bits_per_sample;
    uint64_t                _total_bytes;
    uint64_t                _total_time_us;
    float                   _best_bandwidth;
    float                   _worst_bandwidth;
    float                   _worst_queue_ratio;
    bool                    _data_written;
    float                   _truncate_sec;
    uint8_t                 _header_size;
};
