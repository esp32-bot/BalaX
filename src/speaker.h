#ifndef _SPEAKER_H
#define _SPEAKER_H

#include <stddef.h>
#include <stdint.h>

// Volume is a percentage: 0 = mute, 100 = loudest.
#define SPEAKER_MAX_VOLUME     100
#define SPEAKER_DEFAULT_VOLUME 20

// One step of a melody; freq 0 is a rest.
struct Speaker_Note {
    uint16_t freq; // Hz
    uint16_t ms;
};

extern uint8_t speaker_volume;

void Setup_Speaker();
void Speaker_SetVolume(uint8_t volume);
void Speaker_Tone(uint32_t freq);
void Speaker_Stop();
void Speaker_PlayMelody(const Speaker_Note *notes, size_t count);

template <size_t N>
void Speaker_PlayMelody(const Speaker_Note (&notes)[N]) {
    Speaker_PlayMelody(notes, N);
}

void StartUp_Sound( void * pvParameters );
void Shutdown_Sound();

#endif
