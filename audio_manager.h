#ifndef AUDIO_MANAGER_H
#define AUDIO_MANAGER_H

#include <Arduino.h>

// Initialize I2S peripheral and DMA ring buffer for MAX98357A amplifier.
// No-op if audio not enabled in config. Call once from setup().
void audioSetup();

// Feed DMA buffer with audio data. Non-blocking — call every loop() iteration.
// Returns immediately if nothing is playing or audio is disabled.
void audioLoop();

// Start playing a WAV file from LittleFS. Non-blocking: loads header,
// then audioLoop() feeds data to I2S in chunks.
// filename: e.g. "/armed.wav", "/go.wav", "/finish.wav", "/record.wav"
void playSound(const char* filename);

// Stop any currently playing sound immediately.
void stopSound();

// Returns true if a sound is currently playing.
bool isPlaying();

// Set volume level (0-21). Higher = louder. Adjusts I2S output scaling.
void setVolume(uint8_t level);

// Get list of WAV files in LittleFS as JSON array string
String getAudioFileList();

#endif
