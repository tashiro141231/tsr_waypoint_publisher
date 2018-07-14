#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <alsa/asoundlib.h>
/* PCMデフォルト設定 */
#define DEF_CHANNEL         2
#define DEF_FS              48000
#define DEF_BITPERSAMPLE    16
#define WAVE_FORMAT_PCM     1
#define BUF_SAMPLES         1024

/* ChunkID 定義 */
const char ID_RIFF[] = "RIFF";
const char ID_WAVE[] = "WAVE";
const char ID_FMT[]  = "fmt ";
const char ID_DATA[] = "data";

/* PCM情報格納用構造体 */
typedef struct {
    uint16_t      wFormatTag;         // format type
    uint16_t      nChannels;          // number of channels (1:mono, 2:stereo)
    uint32_t      nSamplesPerSec;     // sample rate
    uint32_t      nAvgBytesPerSec;    // for buffer estimation
    uint16_t      nBlockAlign;        // block size of data
    uint16_t      wBitsPerSample;     // number of bits per sample of mono data
    uint16_t      cbSize;             // extra information
} WAVEFORMATEX;

/* CHUNK */
typedef struct {
    char        ID[4];  // Chunk ID
    uint32_t    Size;   // Chunk size;
} CHUNK;

static int readWavHeader(FILE *fp, WAVEFORMATEX *wf);
int wavePlay(char* filename);
