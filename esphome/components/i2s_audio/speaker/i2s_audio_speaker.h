#pragma once

#ifdef USE_ESP32

#include "../i2s_audio.h"

#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "esphome/components/speaker/speaker.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace i2s_audio {

static const size_t BUFFER_SIZE = 1024;

enum class TaskEventType : uint8_t {
  PLAYING,
  PAUSING,
  WARNING = 255,
};

struct TaskEvent {
  TaskEventType type;
  esp_err_t err;
};

struct DataEvent {
  bool stop;
  size_t len;
  uint8_t data[BUFFER_SIZE];
};

class I2SAudioSpeaker : public Component, public speaker::Speaker, public I2SWriter {
 public:
  float get_setup_priority() const override { return esphome::setup_priority::LATE; }

  void setup() override;
  void loop() override;
  void dump_config() override;

#if SOC_I2S_SUPPORTS_DAC
  void set_internal_dac_mode(i2s_dac_mode_t mode) { this->internal_dac_mode_ = mode; }
#endif

  void start() override;
  void stop() override;
  void finish() override;
  void flush() override;

  size_t play(const uint8_t *data, size_t length) override;

  bool has_buffered_data() const override;
  size_t available_space() const override;

 protected:
  void start_();
  void stop_();
  uint8_t sample_size_() {
    uint8_t size = 0;
    switch (this->bits_per_sample_) {
      case i2s_bits_per_sample_t.I2S_BITS_PER_SAMPLE_8BIT, size = 1; break;
          case i2s_bits_per_sample_t.I2S_BITS_PER_SAMPLE_16BIT, size = 2; break;
          case i2s_bits_per_sample_t.I2S_BITS_PER_SAMPLE_16BIT, size = 3; break; default:
        size = 4;
    }
    if (this->channel_fmt_ == i2s_channel_fmt_t.I2S_CHANNEL_FMT_RIGHT_LEFT) {
      size *= 2;
    }

    return size;
  }

  static void player_task(void *params);

  TaskHandle_t player_task_handle_{nullptr};
  StreamBufferHandle_t buffer_queue_{nullptr};
  QueueHandle_t event_queue_;

  bool task_created_{false};
#if SOC_I2S_SUPPORTS_DAC
  i2s_dac_mode_t internal_dac_mode_{I2S_DAC_CHANNEL_DISABLE};
#endif
};

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
