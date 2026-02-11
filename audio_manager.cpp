#include "audio_manager.h"
#include "config.h"
#include <LittleFS.h>
#include <driver/i2s.h>

// ============================================================================
// I2S CONFIGURATION for MAX98357A
// ============================================================================
#define I2S_PORT          I2S_NUM_0
#define DMA_BUF_COUNT     8
#define DMA_BUF_LEN       256
#define SAMPLE_RATE       16000   // 16kHz mono — good balance of quality and size

// ============================================================================
// PLAYBACK STATE
// ============================================================================
static File audioFile;
static bool audioPlaying = false;
static bool audioInitialized = false;
static uint32_t audioDataStart = 0;   // Byte offset where PCM data begins in WAV
static uint32_t audioDataSize = 0;    // Total PCM data bytes
static uint32_t audioBytesRead = 0;   // Bytes fed to I2S so far
static uint8_t volumeLevel = 10;      // 0-21
static uint8_t audioBuffer[512];      // Chunk buffer for DMA feeding

// ============================================================================
// WAV HEADER PARSER
// Minimal parser — extracts data chunk offset and size from standard WAV files.
// Supports 8-bit and 16-bit PCM. We convert everything to 16-bit for I2S.
// ============================================================================
static uint8_t wavBitsPerSample = 16;
static uint16_t wavChannels = 1;
static uint32_t wavSampleRate = 16000;

static bool parseWavHeader(File& f) {
  uint8_t header[44];
  if (f.read(header, 44) != 44) return false;

  // Check RIFF header
  if (header[0] != 'R' || header[1] != 'I' || header[2] != 'F' || header[3] != 'F') return false;
  if (header[8] != 'W' || header[9] != 'A' || header[10] != 'V' || header[11] != 'E') return false;

  // Parse fmt chunk
  wavChannels = header[22] | (header[23] << 8);
  wavSampleRate = header[24] | (header[25] << 8) | (header[26] << 16) | (header[27] << 24);
  wavBitsPerSample = header[34] | (header[35] << 8);

  // Find 'data' chunk — it's usually at offset 36 but can vary
  f.seek(12); // After RIFF header
  while (f.available() >= 8) {
    uint8_t chunkHeader[8];
    if (f.read(chunkHeader, 8) != 8) return false;

    uint32_t chunkSize = chunkHeader[4] | (chunkHeader[5] << 8) |
                         (chunkHeader[6] << 16) | (chunkHeader[7] << 24);

    if (chunkHeader[0] == 'd' && chunkHeader[1] == 'a' &&
        chunkHeader[2] == 't' && chunkHeader[3] == 'a') {
      audioDataStart = f.position();
      audioDataSize = chunkSize;
      return true;
    }

    // Skip non-data chunk
    f.seek(f.position() + chunkSize);
  }

  return false;
}

// ============================================================================
// SETUP
// ============================================================================
void audioSetup() {
  if (!cfg.audio_enabled) return;

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = DMA_BUF_COUNT,
    .dma_buf_len = DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = cfg.i2s_bclk_pin,
    .ws_io_num = cfg.i2s_lrc_pin,
    .data_out_num = cfg.i2s_dout_pin,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    LOG.printf("[AUDIO] I2S driver install failed: %d\n", err);
    return;
  }

  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    LOG.printf("[AUDIO] I2S pin config failed: %d\n", err);
    i2s_driver_uninstall(I2S_PORT);
    return;
  }

  i2s_zero_dma_buffer(I2S_PORT);
  audioInitialized = true;

  LOG.printf("[AUDIO] I2S initialized: BCLK=%d, LRC=%d, DOUT=%d\n",
                cfg.i2s_bclk_pin, cfg.i2s_lrc_pin, cfg.i2s_dout_pin);
}

// ============================================================================
// MAIN LOOP - Feed DMA buffer (non-blocking)
// ============================================================================
void audioLoop() {
  if (!audioInitialized || !audioPlaying) return;

  // Feed one chunk per loop iteration — keeps it non-blocking
  size_t bytesToRead = sizeof(audioBuffer);
  size_t remaining = audioDataSize - audioBytesRead;
  if (bytesToRead > remaining) bytesToRead = remaining;

  if (bytesToRead == 0) {
    // Playback complete
    stopSound();
    return;
  }

  size_t bytesRead = audioFile.read(audioBuffer, bytesToRead);
  if (bytesRead == 0) {
    stopSound();
    return;
  }

  // Convert to 16-bit samples with volume scaling
  int16_t sampleBuf[256]; // Max 512 bytes = 256 16-bit samples
  size_t sampleCount = 0;

  if (wavBitsPerSample == 8) {
    // Convert 8-bit unsigned to 16-bit signed with volume
    for (size_t i = 0; i < bytesRead; i++) {
      int16_t sample = ((int16_t)audioBuffer[i] - 128) << 8;
      sample = (sample * volumeLevel) / 21;
      sampleBuf[sampleCount++] = sample;
    }
  } else {
    // 16-bit samples — apply volume
    sampleCount = bytesRead / 2;
    for (size_t i = 0; i < sampleCount; i++) {
      int16_t sample = (int16_t)(audioBuffer[i * 2] | (audioBuffer[i * 2 + 1] << 8));
      sample = (sample * volumeLevel) / 21;
      sampleBuf[i] = sample;
    }
  }

  // If stereo, we only play left channel (MAX98357A is mono)
  // For mono files this is a no-op
  if (wavChannels == 2) {
    size_t monoCount = sampleCount / 2;
    for (size_t i = 0; i < monoCount; i++) {
      sampleBuf[i] = sampleBuf[i * 2]; // Take left channel
    }
    sampleCount = monoCount;
  }

  // Write to I2S — non-blocking with 0 timeout
  size_t bytesWritten = 0;
  i2s_write(I2S_PORT, sampleBuf, sampleCount * 2, &bytesWritten, 0);

  audioBytesRead += bytesRead;
}

// ============================================================================
// PLAY SOUND
// ============================================================================
void playSound(const char* filename) {
  if (!audioInitialized) return;

  // Stop any current playback
  if (audioPlaying) stopSound();

  // Open WAV file from LittleFS
  String path = String("/") + filename;
  if (!LittleFS.exists(path)) {
    LOG.printf("[AUDIO] File not found: %s\n", path.c_str());
    return;
  }

  audioFile = LittleFS.open(path, "r");
  if (!audioFile) {
    LOG.printf("[AUDIO] Failed to open: %s\n", path.c_str());
    return;
  }

  // Parse WAV header to find data chunk
  if (!parseWavHeader(audioFile)) {
    LOG.printf("[AUDIO] Invalid WAV: %s\n", path.c_str());
    audioFile.close();
    return;
  }

  // Reconfigure I2S sample rate if WAV differs from default
  if (wavSampleRate != SAMPLE_RATE) {
    i2s_set_sample_rates(I2S_PORT, wavSampleRate);
  }

  // Seek to data start and begin playback
  audioFile.seek(audioDataStart);
  audioBytesRead = 0;
  audioPlaying = true;

  LOG.printf("[AUDIO] Playing: %s (%dHz, %dbit, %dch, %d bytes)\n",
                filename, wavSampleRate, wavBitsPerSample, wavChannels, audioDataSize);
}

// ============================================================================
// STOP SOUND
// ============================================================================
void stopSound() {
  if (!audioInitialized) return;

  audioPlaying = false;
  if (audioFile) {
    audioFile.close();
  }
  i2s_zero_dma_buffer(I2S_PORT);

  // Restore default sample rate
  if (wavSampleRate != SAMPLE_RATE) {
    i2s_set_sample_rates(I2S_PORT, SAMPLE_RATE);
    wavSampleRate = SAMPLE_RATE;
  }
}

// ============================================================================
// STATUS & CONTROL
// ============================================================================
bool isPlaying() {
  return audioPlaying;
}

void setVolume(uint8_t level) {
  if (level > 21) level = 21;
  volumeLevel = level;
  LOG.printf("[AUDIO] Volume set to %d/21\n", level);
}

String getAudioFileList() {
  String json = "[";
  File root = LittleFS.open("/");
  File f = root.openNextFile();
  bool first = true;
  while (f) {
    String name = String(f.name());
    if (name.endsWith(".wav")) {
      if (!first) json += ",";
      json += "{\"name\":\"" + name + "\",\"size\":" + String(f.size()) + "}";
      first = false;
    }
    f = root.openNextFile();
  }
  json += "]";
  return json;
}
