#include <M5StickCPlus.h>
#include "IO.h"
#include "speaker.h"
#include "sounds.h"

uint8_t speaker_volume = SPEAKER_DEFAULT_VOLUME;

// A square wave is loudest at 50% duty: half of full scale at this resolution.
static const uint8_t PWM_RESOLUTION_BITS = 10;
static const uint32_t MAX_DUTY = (1u << (PWM_RESOLUTION_BITS - 1)) - 1;
// Placeholder until the first tone sets the real frequency.
static const uint32_t IDLE_FREQ_HZ = 5000;

void Setup_Speaker(){
    pinMode(SPEAKER, OUTPUT);
    ledcSetup(SPEAKER_CH, IDLE_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(SPEAKER, SPEAKER_CH);
}

void Speaker_SetVolume(uint8_t volume){
    speaker_volume = min(volume, (uint8_t)SPEAKER_MAX_VOLUME);
}

// Play freq Hz on the buzzer at speaker_volume, or stop it when freq is 0.
// Duty grows with the square of the volume so equal steps sound roughly equal;
// 50% comes out about -8 dB, which sounds about half as loud as 100%.
void Speaker_Tone(uint32_t freq){
    if (freq == 0 || speaker_volume == 0) {
        Speaker_Stop();
        return;
    }
    ledcChangeFrequency(SPEAKER_CH, freq, PWM_RESOLUTION_BITS);
    uint32_t duty = MAX_DUTY * speaker_volume * speaker_volume / (SPEAKER_MAX_VOLUME * SPEAKER_MAX_VOLUME);
    ledcWrite(SPEAKER_CH, max(duty, 1u));
}

void Speaker_Stop(){
    ledcWrite(SPEAKER_CH, 0);
}

// Blocks the calling task until the melody finishes, then silences the buzzer.
void Speaker_PlayMelody(const Speaker_Note *notes, size_t count){
    for (size_t i = 0; i < count; i++) {
        Speaker_Tone(notes[i].freq);
        vTaskDelay(pdMS_TO_TICKS(notes[i].ms));
    }
    Speaker_Stop();
}

void StartUp_Sound( void * pvParameters ){
    Speaker_PlayMelody(STARTUP_MELODY);
    vTaskDelete(NULL);
}

void Shutdown_Sound(){
    Speaker_PlayMelody(SHUTDOWN_MELODY);
}
