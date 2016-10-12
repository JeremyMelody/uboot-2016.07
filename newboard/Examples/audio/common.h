#ifndef __COMMON_H__
#define __COMMON_H__

#define RECORD_TIME		(10 * 1000000)
#define BITRATE			22000//24000//22000//12000
#define CHANNELS		1
#define	BIT_PER_SAMPLE	16

#define	BUFFER_TIME_US	200000
#define PERIOD_TIME_US	50000

#define MIN(a,b)	((a)>(b)?(b):(a))

typedef struct WAV_HEADER
{
	char rId[4];    // "RIFF"
	int rLen;       // File size - 8
	char wId[4];    // "WAVE"
	char fId[4];    // "fmt"
	int fLen;       // Format block size = 16
	
	short wFormatTag;        // Format tag = 0x0001(no compressed)
	short wChannels;         // Channels
	int   nSamplesPersec ;   // Sample rate
	int   nAvgBytesPerSample;// = nSamplesPersec * wChannels * wBitsPerSample / 8
	short  wBlockAlign;		// = wChannels * wBitsPerSample / 8
	short wBitsPerSample;
	
	char dId[4];        // "data"
	int wSampleLength;  // PCM data length
} wav_header_t;

#endif