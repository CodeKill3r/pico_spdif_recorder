/*------------------------------------------------------/
/ Copyright (c) 2024, Elehobica
/ Released under the BSD-2-Clause
/ refer to https://opensource.org/licenses/BSD-2-Clause
/------------------------------------------------------*/

#include "wav_file.h"

#include <string>
#include <cstring>
#include <cmath>

#include "wav_file_status.h"

#include "oled.h"

/*-----------------/
/  Local function
/-----------------*/
static inline uint64_t _micros()
{
    return to_us_since_boot(get_absolute_time());
}

static inline void _blocking_wait_core0_grant()
{
    wav_file_status::blocking_wait_core0_grant();
}

static inline void _drain_core0_grant()
{
    wav_file_status::drain_core0_grant();
}

/*----------------------------------------/
/  FATFS wrappers for waiting core0 grant
/----------------------------------------*/
static inline FRESULT _wrap_f_open(FIL* fp, const TCHAR* path, BYTE mode)
{
    _blocking_wait_core0_grant();
    return f_open(fp, path, mode);
}

static inline FRESULT _wrap_f_close(FIL* fp)
{
    _blocking_wait_core0_grant();
    return f_close(fp);
}

static inline FRESULT _wrap_f_read(FIL* fp, void* buff, UINT btr, UINT* br)
{
    _blocking_wait_core0_grant();
    return f_read(fp, buff, btr, br);
}

static inline FRESULT _wrap_f_write(FIL* fp, const void* buff, UINT btw, UINT* bw)
{
    _blocking_wait_core0_grant();
    return f_write(fp, buff, btw, bw);
}

static inline FRESULT _wrap_f_write_priority(FIL* fp, const void* buff, UINT btw, UINT* bw)
{
    // no blocking wait for core0 grant
    return f_write(fp, buff, btw, bw);
}

static inline FRESULT _wrap_f_lseek(FIL* fp, FSIZE_t ofs)
{
    _blocking_wait_core0_grant();
    return f_lseek(fp, ofs);
}

static inline FRESULT _wrap_f_truncate(FIL* fp)
{
    _blocking_wait_core0_grant();
    return f_truncate(fp);
}

static inline FRESULT _wrap_f_sync(FIL* fp)
{
    _blocking_wait_core0_grant();
    return f_sync(fp);
}

static inline FRESULT _wrap_f_sync_priority(FIL* fp)
{
    // no blocking wait for core0 grant
    return f_sync(fp);
}

static inline FRESULT _wrap_f_unlink(const TCHAR* path)
{
    _blocking_wait_core0_grant();
    return f_unlink(path);
}

static inline FSIZE_t _wrap_f_tell(FIL* fp)
{
    _blocking_wait_core0_grant();
    return f_tell(fp);
}

/*-----------------/
/  Class variables
/-----------------*/
uint32_t wav_file::_wav_buf[SPDIF_BLOCK_SIZE*3/4 * NUM_SUB_FRAME_BUF / 2];

/*------------------------/
/  Public class functions
/------------------------*/

/*-----------------/
/  Constructor
/-----------------*/
wav_file::wav_file(const uint32_t suffix, const uint32_t sample_freq, const bits_per_sample_t bits_per_sample) :
    _fil(),
    _filename(),
    _sample_freq(sample_freq),
    _bits_per_sample(bits_per_sample),
    _total_bytes(0),
    _total_time_us(0),
    _best_bandwidth(-INFINITY),
    _worst_bandwidth(INFINITY),
    _worst_queue_ratio(0.0f),
    _data_written(false),
    _truncate_sec(0.0f)
{
    char wav_filename[FNAMLEN];
    char wav_ext[5];
#ifdef W64
    if ((spdif_rec_wav::get_fsys()==FS_EXFAT) && (!spdif_rec_wav::noW64()))
        memcpy(wav_ext,WAV64_EXT,5);
    else
#endif   //RF64 uses regular .wav extension
        memcpy(wav_ext,WAV_EXT,5);

#ifdef RTCSUFX
    if (spdif_rec_wav::isRtc()){
        aon_timer_get_time_calendar(&wav_rtc);
        sprintf(wav_filename, "%s%d-%02d-%02d_%02d-%02d-%02d%s", WAV_PREFIX, wav_rtc.tm_year+1900, wav_rtc.tm_mon+1, wav_rtc.tm_mday,  wav_rtc.tm_hour, wav_rtc.tm_min, wav_rtc.tm_sec, wav_ext);
    }else{
        sprintf(wav_filename, "%s%03d%s", WAV_PREFIX, suffix, wav_ext);
    }
#else
    sprintf(wav_filename, "%s%03d%s", WAV_PREFIX, suffix, wav_ext);
#endif
    _filename = std::string(wav_filename);
    memccpy(oled_fnam,wav_filename,0,27);

    _drain_core0_grant();
    for ( ; ; ) {
        FRESULT fr;     /* FatFs return code */
        UINT bw;
        uint8_t buf[WAV_HEADER_SIZE];
        uint16_t u16;
        uint32_t u32;
        _header_size=0;

        fr = _wrap_f_open(&_fil, _filename.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
        if (fr != FR_OK){
//            printf("open_err %d \r\n",fr);
            break;
        }

        if ((spdif_rec_wav::get_fsys()!=FS_EXFAT)  || (spdif_rec_wav::noW64())){
            _header_size=44;

            // ChunkID
            memcpy(&buf[0], "RIFF", 4);
            // ChunkSize (temporary 0)
            memset(&buf[4], 0, 4);
            // Format
            memcpy(&buf[8], "WAVE", 4);

            // Subchunk1ID
            memcpy(&buf[12], "fmt ", 4);
            // Subchunk1Size
            u32 = 16;  // 16 for PCM
            memcpy(&buf[16], (const void *) &u32, 4);
            // AudioFormat
            u16 = 1;  // PCM = 1
            memcpy(&buf[20], (const void *) &u16, 2);
            // NumChannels
            u16 = NUM_CHANNELS;  // e.g. Stereo = 2
            memcpy(&buf[22], (const void *) &u16, 2);
            // SampleRate
            u32 = _sample_freq;  // e.g. 44100
            memcpy(&buf[24], (const void *) &u32, 4);
            // ByteRate
            u32 = _sample_freq * NUM_CHANNELS * (static_cast<uint16_t>(_bits_per_sample)/8);
            memcpy(&buf[28], (const void *) &u32, 4);
            // BlockAlign
            u16 = NUM_CHANNELS * (static_cast<uint16_t>(_bits_per_sample)/8);
            memcpy(&buf[32], (const void *) &u16, 2);
            // BitsPerSample
            u16 = static_cast<uint16_t>(_bits_per_sample);
            memcpy(&buf[34], (const void *) &u16, 2);

            // Subchunk2ID
            memcpy(&buf[36], "data", 4);
            // Subchunk2Size (temporary 0)
            memset(&buf[40], 0, 4);

            fr = _wrap_f_write(&_fil, buf, _header_size, &bw);
            if (fr != FR_OK || bw != _header_size) {
    //            printf("wr_err \r\n");
                break;
            }
            fr = _wrap_f_sync(&_fil);
            if (fr != FR_OK){
    //            printf("sync_err \r\n");
                break;
            }
        }else{  //extFat -- use 64bit wave file
#ifdef W64  //wave64 format
            //build header
            _header_size=104;
            Wave64FileHeader filehdr;
            Wave64ChunkHeader datahdr, fmthdr;
            WaveHeader wavhdr;

            memcpy (filehdr.ckID, riff_guid, sizeof (riff_guid));
            memcpy (filehdr.formType, wave_guid, sizeof (wave_guid));
            filehdr.ckSize=-1;          // update at the end of record

            wavhdr.FormatTag=1;                 //PCM
            wavhdr.NumChannels=NUM_CHANNELS;    //=2 stereo
            wavhdr.SampleRate=_sample_freq;
            wavhdr.BytesPerSecond=_sample_freq * NUM_CHANNELS * (static_cast<uint16_t>(_bits_per_sample)/8);
            wavhdr.BlockAlign=NUM_CHANNELS * (static_cast<uint16_t>(_bits_per_sample)/8);
            wavhdr.BitsPerSample=static_cast<uint16_t>(_bits_per_sample);

            memcpy (fmthdr.ckID, fmt_guid, sizeof (fmt_guid));
            fmthdr.ckSize = sizeof (fmthdr) + sizeof(wavhdr);

            memcpy (datahdr.ckID, data_guid, sizeof (data_guid));
            datahdr.ckSize = -1;        // update at the end of record

            //prepare write buffer
            memcpy(&buf[0],&filehdr,sizeof(filehdr));       //40bytes

            memcpy(&buf[40],&fmthdr,sizeof(fmthdr));        //64
            memcpy(&buf[64],&wavhdr,sizeof(wavhdr));        //80

            memcpy(&buf[80],&datahdr,sizeof(datahdr));      //104



            fr = _wrap_f_write(&_fil, buf, _header_size, &bw);
            if (fr != FR_OK || bw != _header_size) {
                break;
            }
            fr = _wrap_f_sync(&_fil);
            if (fr != FR_OK){
                break;
            }
#else       //RF64 format
            //build header
            _header_size=80;

            ChunkHeader ds64hdr, datahdr, fmthdr;
            RiffChunkHeader riffhdr;
            DS64Chunk ds64_chunk;
            WaveHeader wavhdr;

            memcpy (riffhdr.ckID, "RF64" , sizeof (riffhdr.ckID));
            memcpy (riffhdr.formType, "WAVE", sizeof (riffhdr.formType));
            riffhdr.ckSize = (uint32_t) -1;

            wavhdr.FormatTag=1;                 //PCM
            wavhdr.NumChannels=NUM_CHANNELS;    //=2 stereo
            wavhdr.SampleRate=_sample_freq;
            wavhdr.BytesPerSecond=_sample_freq * NUM_CHANNELS * (static_cast<uint16_t>(_bits_per_sample)/8);
            wavhdr.BlockAlign=NUM_CHANNELS * (static_cast<uint16_t>(_bits_per_sample)/8);
            wavhdr.BitsPerSample=static_cast<uint16_t>(_bits_per_sample);

            memcpy (ds64hdr.ckID, "ds64", sizeof (ds64hdr.ckID));
            ds64hdr.ckSize = sizeof(ds64_chunk);
            ds64_chunk.riffSize64 = 0;      // total_riff_bytes;
            ds64_chunk.dataSize64 = 0;      // total_data_bytes;
            ds64_chunk.sampleCount64 = 0;   // total_samples;
            ds64_chunk.tableLength =0;

            memcpy (fmthdr.ckID, "fmt ", sizeof (fmthdr.ckID));
            fmthdr.ckSize = sizeof(wavhdr);

            memcpy (datahdr.ckID, "data", sizeof (datahdr.ckID));
            datahdr.ckSize = (uint32_t) -1;


            memcpy(&buf[0],&riffhdr,sizeof(riffhdr));           //12
            memcpy(&buf[12],&ds64hdr,sizeof(ds64hdr));          //8
            memcpy(&buf[20],&ds64_chunk, sizeof(ds64_chunk));   //28
            memcpy(&buf[48],&fmthdr,sizeof(fmthdr));            //8
            memcpy(&buf[56],&wavhdr,sizeof(wavhdr));            //16
            memcpy(&buf[72],&datahdr,sizeof(datahdr));          //8
            sizeof(uint32_t);


            fr = _wrap_f_write(&_fil, buf, _header_size, &bw);
            if (fr != FR_OK || bw != _header_size) {
    //            printf("wr_err \r\n");
                break;
            }
            fr = _wrap_f_sync(&_fil);
            if (fr != FR_OK){
    //            printf("sync_err \r\n");
                break;
            }
#endif
        }
        return;
    }

    spdif_rec_wav::report_error(spdif_rec_wav::error_type_t::WAV_OPEN_FAIL);
}

/*-----------------/
/  Destructor
/-----------------*/
wav_file::~wav_file()
{
    _drain_core0_grant();
    for ( ; ; ) {
        FRESULT fr;     /* FatFs return code */

        if (_total_bytes == 0) {
            // remove file if no samples
            fr = _wrap_f_close(&_fil);
            if (fr != FR_OK) break;

            fr = _wrap_f_unlink(_filename.c_str());
            if (fr != FR_OK) break;
        } else if ((spdif_rec_wav::get_fsys()!=FS_EXFAT) || (spdif_rec_wav::noW64())  ){
            UINT bw;
            uint32_t u32;
            DWORD cur_pos = _wrap_f_tell(&_fil);

            if (_truncate_sec > 0.0f) {
                uint32_t truncate_bytes = static_cast<uint32_t>(_truncate_sec * (static_cast<uint32_t>(_bits_per_sample)/8) * NUM_CHANNELS * _sample_freq);
                cur_pos -= truncate_bytes;
                _total_bytes -= truncate_bytes;
                fr = _stepwise_seek(cur_pos);
                if (fr != FR_OK) break;
                fr = _wrap_f_truncate(&_fil);
                if (fr != FR_OK) break;
            }

            fr = _wrap_f_sync(&_fil);
            if (fr != FR_OK) break;

            // ChunkSize
            fr = _stepwise_seek(4);
            if (fr != FR_OK) break;
            u32 = _total_bytes + (_header_size - 8);
            fr = _wrap_f_write(&_fil, static_cast<const void *>(&u32), sizeof(uint32_t), &bw);
            if (fr != FR_OK || bw != sizeof(uint32_t)) break;
            fr = _wrap_f_sync(&_fil);
            if (fr != FR_OK) break;

            // Subchunk2Size
            fr = _stepwise_seek(40);
            if (fr != FR_OK) break;
            u32 = _total_bytes;
            fr = _wrap_f_write(&_fil, static_cast<const void *>(&u32), sizeof(uint32_t), &bw);
            if (fr != FR_OK || bw != sizeof(uint32_t)) break;
            fr = _wrap_f_sync(&_fil);
            if (fr != FR_OK) break;

            /////why?
            // fr = _stepwise_seek(cur_pos);
            // if (fr != FR_OK) break;
            fr = _wrap_f_close(&_fil);
            if (fr != FR_OK) break;
        } else {
#ifdef W64
            UINT bw;
            uint64_t u64;
            FSIZE_t cur_pos = _wrap_f_tell(&_fil);

            if (_truncate_sec > 0.0f) {
                uint64_t truncate_bytes = static_cast<uint64_t>(_truncate_sec * (static_cast<uint64_t>(_bits_per_sample)/8) * NUM_CHANNELS * _sample_freq);
                cur_pos -= truncate_bytes;
                _total_bytes -= truncate_bytes;
                fr = _stepwise_seek(cur_pos);
                if (fr != FR_OK) break;
                fr = _wrap_f_truncate(&_fil);
                if (fr != FR_OK) break;
            }

            fr = _wrap_f_sync(&_fil);
            if (fr != FR_OK) {
//                printf("close sync1\r\n");
                break;
            }

            // ChunkSize
            fr = _stepwise_seek(16);
            if (fr != FR_OK) {
//                printf("close seek1\r\n");
                break;
            }
            //              filehdr                        fmthdr                   wavhdr                  datahdr
            u64 = sizeof (Wave64FileHeader) + sizeof (Wave64ChunkHeader) + sizeof(WaveHeader) + sizeof (Wave64ChunkHeader) + ((_total_bytes + 7) & ~(int64_t)7);
            fr = _wrap_f_write(&_fil, static_cast<const void *>(&u64), sizeof(uint64_t), &bw);
            if (fr != FR_OK || bw != sizeof(uint64_t)) {
//                printf("close write1\r\n");
                break;
            }
            fr = _wrap_f_sync(&_fil);
            if (fr != FR_OK) {
//                printf("close sync2\r\n");
                break;
            }

            // Subchunk2Size
            fr = _stepwise_seek(96);
            if (fr != FR_OK) break;
            u64 = _total_bytes + sizeof (Wave64ChunkHeader);
            fr = _wrap_f_write(&_fil, static_cast<const void *>(&u64), sizeof(uint64_t), &bw);
            if (fr != FR_OK || bw != sizeof(uint64_t)) break;
            fr = _wrap_f_sync(&_fil);
            if (fr != FR_OK) break;

            // fr = _stepwise_seek(cur_pos);
            // if (fr != FR_OK) break;
            fr = _wrap_f_close(&_fil);
            if (fr != FR_OK) break;
#else //RF64
            UINT bw;
            uint64_t u64;
            FSIZE_t cur_pos = _wrap_f_tell(&_fil);

            if (_truncate_sec > 0.0f) {
                uint64_t truncate_bytes = static_cast<uint64_t>(_truncate_sec * (static_cast<uint64_t>(_bits_per_sample)/8) * NUM_CHANNELS * _sample_freq);
                cur_pos -= truncate_bytes;
                _total_bytes -= truncate_bytes;
                fr = _stepwise_seek(cur_pos);
                if (fr != FR_OK) break;
                fr = _wrap_f_truncate(&_fil);
                if (fr != FR_OK) break;
            }

            fr = _wrap_f_sync(&_fil);
            if (fr != FR_OK) {
//                printf("close sync1\r\n");
                break;
            }

            // ChunkSize
            fr = _stepwise_seek(20);
            if (fr != FR_OK) {
//                printf("close seek1\r\n");
                break;
            }

            // ds64_chunk.riffSize64 = 0;      // total_riff_bytes;
            // ds64_chunk.dataSize64 = 0;      // total_data_bytes;
            // ds64_chunk.sampleCount64 = 0;   // total_samples;

            // total_riff_bytes = sizeof (RiffChunkHeader) + sizeof(WaveHeader) + sizeof (ChunkHeader) + ((_total_bytes + 1) & ~(int64_t)1);
            // total_riff_bytes += sizeof (ChunkHeader) + sizeof (DS64Chunk); // <-last one is broken -- allocated(DS64Chunk)=28  not 32=sizeof (DS64Chunk)

            u64 = sizeof (RiffChunkHeader) + sizeof(WaveHeader) + sizeof (ChunkHeader) + ((_total_bytes + 1) & ~(int64_t)1) + sizeof (ChunkHeader) + sizeof(DS64Chunk);
            fr = _wrap_f_write(&_fil, static_cast<const void *>(&u64), sizeof(uint64_t), &bw);
            if (fr != FR_OK || bw != sizeof(uint64_t)) {
//                printf("close write1\r\n");
                break;
            }

            //data size
            u64 = _total_bytes;
            fr = _wrap_f_write(&_fil, static_cast<const void *>(&u64), sizeof(uint64_t), &bw);
            if (fr != FR_OK || bw != sizeof(uint64_t)) {
//                printf("close write1\r\n");
                break;
            }

            //total sample count
            u64 = _total_bytes/ (static_cast<uint32_t>(_bits_per_sample)/8) / NUM_CHANNELS;
            fr = _wrap_f_write(&_fil, static_cast<const void *>(&u64), sizeof(uint64_t), &bw);
            if (fr != FR_OK || bw != sizeof(uint64_t)) {
//                printf("close write1\r\n");
                break;
            }

            fr = _wrap_f_sync(&_fil);
            if (fr != FR_OK) {
//                printf("close sync2\r\n");
                break;
            }

            // fr = _stepwise_seek(cur_pos);
            // if (fr != FR_OK) break;
            fr = _wrap_f_close(&_fil);
            if (fr != FR_OK) break;

#endif
        }

        ///update free space
        FATFS* fs;
        DWORD fre_clust, fre_sect;
        f_getfree("0:",&fre_clust,&fs);
        oled_free =(uint64_t) fre_clust * (fs->csize) * 512;

        return;
    }

    spdif_rec_wav::report_error(spdif_rec_wav::error_type_t::WAV_CLOSE_FAIL);
}

/*--------------------------/
/  Public Member functions
/--------------------------*/
uint32_t wav_file::write(const uint32_t* buff, const uint32_t sub_frame_count)
{
    uint64_t start_time = _micros();
    uint32_t bytes = _write_core(buff, sub_frame_count);
    uint32_t t_us = static_cast<uint32_t>(_micros() - start_time);
    float bandwidth = static_cast<float>(bytes) / t_us * 1e3;
    if (bandwidth > _best_bandwidth) _best_bandwidth = bandwidth;
    if (bandwidth < _worst_bandwidth) {
        _worst_bandwidth = bandwidth;
        if (spdif_rec_wav::get_verbose()) {
            printf("worst bandwidth updated: %7.2f KB/s\r\n", _worst_bandwidth);
        }
    }
    _total_bytes += bytes;
    _total_time_us += t_us;
    _data_written = true;

    //OLED time calc
    float total_sec_f = static_cast<float>(_total_bytes) / (static_cast<uint32_t>(_bits_per_sample)/8) / NUM_CHANNELS / _sample_freq - _truncate_sec;
    uint32_t total_sec = static_cast<uint32_t>(total_sec_f);
    //uint32_t total_sec_dp = static_cast<uint32_t>((total_sec_f - total_sec) * 1e3);
    oled_frame=static_cast<uint8_t>(total_sec_f*75-total_sec*75)%75;
    oled_sec=total_sec%60;

    if ((oled_min/10)!=((total_sec/600)%6)){
        ///update free space every 10 min
        FATFS* fs;
        DWORD fre_clust, fre_sect;
        f_getfree("0:",&fre_clust,&fs);
        oled_free =(uint64_t) fre_clust * (fs->csize) * 512;
    }

    oled_min=(total_sec/60)%60;
    oled_hour=(total_sec/3600);


    // force immediate split to avoid 32bit file size overflow
    if ((_total_bytes > MAX_TOTAL_BYTES) && ( (spdif_rec_wav::get_fsys()!=FS_EXFAT)  || (spdif_rec_wav::noW64())) ) {
        spdif_rec_wav::split_recording(_bits_per_sample);
        spdif_rec_wav::log_printf("force immediate wav split due to file size\r\n");
    }

    return bytes;
}

void wav_file::set_truncate(const float sec)
{
    _truncate_sec = sec;
}

void wav_file::record_queue_ratio(float queue_ratio)
{
    if (queue_ratio > _worst_queue_ratio) _worst_queue_ratio = queue_ratio;
}

void wav_file::report_start() const
{
    spdif_rec_wav::log_printf("recording start \"%s\" @ %d bits %5.1f KHz (bitrate: %6.1f Kbps)\r\n", _filename.c_str(), _bits_per_sample, static_cast<float>(_sample_freq)*1e-3, static_cast<float>(_bits_per_sample)*_sample_freq*2*1e-3);
}

void wav_file::report_final() const
{
    float total_sec_f = static_cast<float>(_total_bytes) / (static_cast<uint32_t>(_bits_per_sample)/8) / NUM_CHANNELS / _sample_freq - _truncate_sec;
    uint32_t total_sec = static_cast<uint32_t>(total_sec_f);
    uint32_t total_sec_dp = static_cast<uint32_t>((total_sec_f - total_sec) * 1e3);
    spdif_rec_wav::log_printf("recording done \"%s\" %llu bytes (time:  %d:%02d.%03d)\r\n", _filename.c_str(), _total_bytes + WAV_HEADER_SIZE, total_sec/60, total_sec%60, total_sec_dp);
    if (spdif_rec_wav::get_verbose()) {
        float avg_bw = static_cast<float>(_total_bytes) / _total_time_us * 1e3;
        printf("SD Card writing bandwidth\r\n");
        printf(" avg:   %7.2f KB/s\r\n", avg_bw);
        printf(" best:  %7.2f KB/s\r\n", _best_bandwidth);
        printf(" worst: %7.2f KB/s\r\n", _worst_bandwidth);
        printf("WAV file required bandwidth\r\n");
        printf(" wav:   %7.2f KB/s\r\n", static_cast<float>((static_cast<uint32_t>(_bits_per_sample)*_sample_freq*2/8)) / 1e3);
        printf("spdif queue usage: %7.2f %%\r\n", _worst_queue_ratio * 1e2);
    }
}

bool wav_file::is_data_written() const
{
    return _data_written;
}

/*--------------------------/
/  Protected class functions
/--------------------------*/

/*-----------------------------/
/  Protected Member functions
/-----------------------------*/
FRESULT wav_file::_stepwise_seek(const FSIZE_t pos)
{
    FRESULT fr;     /* FatFs return code */

    int64_t target_pos = static_cast<int64_t>(pos);
    int64_t cur_pos    = static_cast<int64_t>(_wrap_f_tell(&_fil));
    int64_t diff       = target_pos - cur_pos;

    while (cur_pos != target_pos) {
        if (diff >= 0) {
            if (cur_pos + SEEK_STEP_BYTES < target_pos) {
                cur_pos += SEEK_STEP_BYTES;
            } else {
                cur_pos = target_pos;
            }
        } else {
            if (cur_pos > target_pos + SEEK_STEP_BYTES) {
                cur_pos -= SEEK_STEP_BYTES;
            } else {
                cur_pos = target_pos;
            }
        }
        fr = _wrap_f_lseek(&_fil, static_cast<FSIZE_t>(cur_pos));
        if (fr != FR_OK) break;
    }
    return fr;
}

uint32_t wav_file::_write_core(const uint32_t* buff, const uint32_t sub_frame_count)
{
    FRESULT fr;     /* FatFs return code */
    UINT bw;

    if (_bits_per_sample == bits_per_sample_t::_16BITS) {
        for (int i = 0; i < sub_frame_count; i++) {
            _wav_buf[i/2] >>= 16;
            _wav_buf[i/2] |= ((buff[i] >> 12) & 0xffff) << 16;
        }
        fr = _wrap_f_write_priority(&_fil, static_cast<const void *>(_wav_buf), sub_frame_count*2, &bw);
        if (fr != FR_OK || bw != sub_frame_count*2) {
            spdif_rec_wav::report_error(spdif_rec_wav::error_type_t::WAV_DATA_WRITE_FAIL);
        }
    } else if (_bits_per_sample == bits_per_sample_t::_24BITS) {
        for (int i = 0, j = 0; i < sub_frame_count; i += 4, j += 3) {
            _wav_buf[j+0] = (((buff[i+1] >> 4) & 0x0000ff) << 24) | (((buff[i+0] >> 4) & 0xffffff) >>  0);
            _wav_buf[j+1] = (((buff[i+2] >> 4) & 0x00ffff) << 16) | (((buff[i+1] >> 4) & 0xffff00) >>  8);
            _wav_buf[j+2] = (((buff[i+3] >> 4) & 0xffffff) <<  8) | (((buff[i+2] >> 4) & 0xff0000) >> 16);
        }
        fr = _wrap_f_write_priority(&_fil, static_cast<const void *>(_wav_buf), sub_frame_count*3, &bw);
        if (fr != FR_OK || bw != sub_frame_count*3) {
            spdif_rec_wav::report_error(spdif_rec_wav::error_type_t::WAV_DATA_WRITE_FAIL);
        }
    }

    // comment below to let FATFS to do f_sync() timing for better performance
    /*
    fr = _wrap_f_sync_priority(&_fil);
    if (fr != FR_OK) {
        spdif_rec_wav::report_error(spdif_rec_wav::error_type_t::WAV_DATA_SYNC_FAIL);
    }
    */

    return static_cast<uint32_t>(bw);
}
