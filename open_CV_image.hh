#include "robotest.hh"

#ifndef __OPEN_CV_IAGME
#define __OPEN_CV_IAGME

enum IMAGE_THREAD_ID:int
	{
		ANCIENT_LAMP,
		AUDIO_CASSETTE,
		MAGIC_LAMP,
		MAMMOTH,
		MAYAN_CALENDAR,
		MJOLNIR_HAMMER,
		ONE_RING,
		PUEBLO_POT,
		ROMAN_GLASS,
		WILLOW_PLATE,
		N_IMAGE_THREADS
	};

void* imageThreadFun(void* params);
void* open_CV_image(void* params);

#endif