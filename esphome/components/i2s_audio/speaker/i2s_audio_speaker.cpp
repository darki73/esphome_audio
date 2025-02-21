#include "i2s_audio_speaker.h"

#ifdef USE_ESP32

#include <driver/i2s.h>
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#ifdef I2S_EXTERNAL_DAC
#include "../external_dac.h"
#endif
namespace esphome {
namespace i2s_audio {

static const size_t BUFFER_COUNT = 20;

static const char *const TAG = "i2s_audio.speaker";

void I2SAudioSpeaker::setup() {
  ESP_LOGCONFIG(TAG, "Setting up I2S Audio Speaker...");

  this->buffer_queue_ = xStreamBufferCreate(BUFFER_COUNT * BUFFER_SIZE, this->sample_size_());
  if (this->buffer_queue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create buffer queue");
    this->mark_failed();
    return;
  }

  this->event_queue_ = xQueueCreate(BUFFER_COUNT, sizeof(TaskEvent));
  if (this->event_queue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create event queue");
    this->mark_failed();
    return;
  }
  xTaskCreate(I2SAudioSpeaker::player_task, "speaker_task", 8192, (void *) this, 1, &this->player_task_handle_);
  vTaskSuspend(this->player_task_handle_);
}

void I2SAudioSpeaker::dump_config() { this->dump_i2s_settings(); }

void I2SAudioSpeaker::flush() {
  if (this->buffer_queue_ != nullptr) {
    xStreamBufferReset(this->buffer_queue_);
  }
}

void I2SAudioSpeaker::start() {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Cannot start audio, speaker failed to setup");
    return;
  }
  if (this->task_created_) {
    ESP_LOGW(TAG, "Called start while task has been already created.");
    return;
  }
  this->set_state_(speaker::STATE_STARTING);
  ESP_LOGD(TAG, "Starting I2S Audio Speaker");
}

void I2SAudioSpeaker::start_() {
  if (!this->claim_i2s_access()) {
    return;  // Waiting for another i2s component to return lock
  }

#ifdef I2S_EXTERNAL_DAC
  if (this->external_dac_ != nullptr) {
    this->external_dac_->init_device();
  }
#endif
  i2s_driver_config_t config = this->get_i2s_cfg();
  config.mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX);

#if SOC_I2S_SUPPORTS_DAC
  if (this->internal_dac_mode_ != I2S_DAC_CHANNEL_DISABLE) {
    config.mode = (i2s_mode_t) (config.mode | I2S_MODE_DAC_BUILT_IN);
  }
#endif

  if (!this->install_i2s_driver(config)) {
    ESP_LOGE(TAG, "Failed to initialize I2S driver: %s", esp_err_to_name(err));
    this->mark_failed();
    this->set_state_(speaker::STATE_STOPPED);
    return;
  }

#if SOC_I2S_SUPPORTS_DAC
  if (this->internal_dac_mode_ != I2S_DAC_CHANNEL_DISABLE) {
    i2s_set_dac_mode(this->internal_dac_mode_);
  }
#endif

#ifdef I2S_EXTERNAL_DAC
  if (this->external_dac_ != nullptr) {
    this->external_dac_->apply_i2s_settings(config);
    this->external_dac_->reset_volume();
  }
#endif

  vTaskResume(this->player_task_handle_);
  ESP_LOGD(TAG, "Started I2S Audio Speaker");
  this->set_state_(speaker::STATE_RUNNING);
}

size_t I2SAudioSpeaker::play(const uint8_t *data, size_t length) {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Cannot play audio, speaker failed to setup");
    return 0;
  }
  if (this->state_ != speaker::STATE_RUNNING) {
    this->start();
  }

  length = std::min(this->available_space(), length);
  uint32_t dword = xStreamBufferSend(this->buffer_queue_, data, length, 0);
  return dword;
}

void I2SAudioSpeaker::player_task(void *params) {
  I2SAudioSpeaker *this_speaker = (I2SAudioSpeaker *) params;
  bool is_playing = false;
  const uint8_t wordsize = this_speaker->sample_size_;
  TaskEvent event;
  uint8_t error_count = 0;
  uint64_t sample;
  size_t bytes_written;

  while (true) {
    if (this_speaker->buffer_queue_ != nullptr) {
      int ret = xStreamBufferReceive(this_speaker->buffer_queue_, &sample, wordsize, portMAX_DELAY);
      if (ret == wordsize) {
        if (!is_playing) {
          event.type = TaskEventType::PLAYING;
          xQueueSend(this_speaker->event_queue_, &event, 10 / portTICK_PERIOD_MS);
          is_playing = true;
        }

        //        if (!this_speaker->use_16bit_mode_) {
        //          sample = (sample << 16) | (sample & 0xFFFF);
        //        }
        esp_err_t err = i2s_write(this_speaker->parent_->get_port(), &sample, wordsize, &bytes_written, portMAX_DELAY);
        if (err != ESP_OK) {
          event.type = TaskEventType::WARNING;
          event.err = err;
          event.stopped = ++error_count >= 5;
          xQueueSend(this_speaker->event_queue_, &event, 10 / portTICK_PERIOD_MS);
        } else if (bytes_written != wordsize) {
          event.type = TaskEventType::WARNING;
          event.err = ESP_FAIL;
          event.data = bytes_written;
          event.stopped = ++error_count >= 5;
          xQueueSend(this_speaker->event_queue_, &event, 10 / portTICK_PERIOD_MS);
        } else {
          error_count = 0;
        }
      } else if (ret == 0) {
        if (!is_playing) {
          event.type = TaskEventType::PAUSING;
          xQueueSend(this_speaker->event_queue_, &event, 10 / portTICK_PERIOD_MS);
          is_playing = true;
        }

        vTaskDelay(1);
      } else {
        event.type = TaskEventType::WARNING;
        event.err = ESP_FAIL;
        event.data = -ret;

        event.stopped = ++error_count >= 5;
        xQueueSend(this_speaker->event_queue_, &event, 10 / portTICK_PERIOD_MS);
      }
    }
  }
}

void I2SAudioSpeaker::stop() {
  if (this->is_failed())
    return;
  if (this->state_ == speaker::STATE_STOPPED)
    return;
  if (this->state_ == speaker::STATE_STARTING) {
    this->state_ = speaker::STATE_STOPPED;
    return;
  }
  ESP_LOGD(TAG, "Stopping I2S Audio Speaker");
  this->state_ = speaker::STATE_STOPPING;
  DataEvent data;
  data.stop = true;
  xQueueSendToFront(this->buffer_queue_, &data, portMAX_DELAY);
}

void I2SAudioSpeaker::stop_() {
  if (this->has_buffered_data()) {
    return;
  }

  vTaskSuspend(this->player_task_handle_);
  // make sure the speaker has no voltage on the pins before closing the I2S poort
  size_t bytes_written;
  uint32_t sample = 0;
  i2s_write(this->parent_->get_port(), &sample, 4, &bytes_written, (10 / portTICK_PERIOD_MS));

  i2s_zero_dma_buffer(this_speaker->parent_->get_port());

  this_speaker->uninstall_i2s_driver();
  this->release_i2s_access();
  this->set_state_(speaker::STATE_STOPPED);
}

void I2SAudioSpeaker::loop() {
  TaskEvent event;
  switch (this->state_) {
    case speaker::STATE_STARTING:
      this->start_();
      return;
    case speaker::STATE_STOPPING:
      this->stop_();
      return;
    case speaker::STATE_STOPPED:
      /* code */
      return;
    case speaker::STATE_RUNNING:

      if (xQueueReceive(this->event_queue_, &event, 0) == pdTRUE) {
        switch (event.type) {
          case TaskEventType::PLAYING:
            ESP_LOGI(TAG, "PLAYING");
            this->status_clear_warning();
            break;
          case TaskEventType::PAUSING:
            ESP_LOGW(TAG, "PAUSING");
            break;
          case TaskEventType::WARNING:
            if (event.data < 0) {
              ESP_LOGW(TAG, "Read data size mismatch: %d instead of %d", -event.data, this->sample_size_());
            } else if (event.data > 0) {
              ESP_LOGW(TAG, "Write data size mismatch: %d instead of %d", event.data, this->sample_size_());
            } else {
              ESP_LOGW(TAG, "Error writing to I2S: %s", esp_err_to_name(event.err));
            }
            this->status_set_warning();
            break;
        }
      }
      break;
  }
}

bool I2SAudioSpeaker::has_buffered_data() const {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Cannot play audio, speaker failed to setup");
    return false;
  }
  return uxQueueMessagesWaiting(this->buffer_queue_) > 0;
}

size_t I2SAudioSpeaker::available_space() const {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Cannot play audio, speaker failed to setup");
    return 0;
  }
  return xStreamBufferSpacesAvailable(this->buffer_queue_);
}

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
