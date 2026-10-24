#include "drivers/QuadratureEncoder.h"

#include <driver/gpio.h>
#include <esp_intr_alloc.h>
#include <soc/gpio_struct.h>

namespace roboty {
namespace {

bool usablePin(int8_t pin) {
  // GPIO6..11 connect to flash; 20/37/38 are not exposed by ESP32-WROOM-32.
  // Check the range before GPIO_IS_VALID_GPIO, which shifts a 64-bit mask.
  return pin >= 0 && pin < GPIO_NUM_MAX && GPIO_IS_VALID_GPIO(pin) &&
         !(pin >= 6 && pin <= 11) && pin != 20 && pin != 37 && pin != 38;
}

}  // namespace

QuadratureEncoder::QuadratureEncoder(
    int8_t pinA, int8_t pinB, bool inverted, bool pullup)
    : pinA_(pinA),
      pinB_(pinB),
      pullup_(pullup),
      started_(false),
      decoder_(inverted) {}

bool QuadratureEncoder::begin() {
  if (started_) {
    return true;
  }
  if (!usablePin(pinA_) || !usablePin(pinB_) || pinA_ == pinB_ ||
      (pullup_ && (pinA_ >= 34 || pinB_ >= 34))) {
    return false;
  }

  const esp_err_t serviceResult = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  if (serviceResult != ESP_OK && serviceResult != ESP_ERR_INVALID_STATE) {
    return false;
  }

  gpio_config_t pins{};
  pins.pin_bit_mask = (1ULL << pinA_) | (1ULL << pinB_);
  pins.mode = GPIO_MODE_INPUT;
  pins.pull_up_en = pullup_ ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  pins.pull_down_en = GPIO_PULLDOWN_DISABLE;
  pins.intr_type = GPIO_INTR_DISABLE;
  if (gpio_config(&pins) != ESP_OK) {
    return false;
  }

  portENTER_CRITICAL(&mux_);
  decoder_.reset(readState());
  portEXIT_CRITICAL(&mux_);

  const gpio_num_t pinA = static_cast<gpio_num_t>(pinA_);
  const gpio_num_t pinB = static_cast<gpio_num_t>(pinB_);
  if (gpio_isr_handler_add(pinA, onChange, this) != ESP_OK) {
    return false;
  }
  if (gpio_isr_handler_add(pinB, onChange, this) != ESP_OK) {
    gpio_isr_handler_remove(pinA);
    return false;
  }
  if (gpio_set_intr_type(pinA, GPIO_INTR_ANYEDGE) != ESP_OK ||
      gpio_set_intr_type(pinB, GPIO_INTR_ANYEDGE) != ESP_OK ||
      gpio_intr_enable(pinA) != ESP_OK ||
      gpio_intr_enable(pinB) != ESP_OK) {
    gpio_isr_handler_remove(pinA);
    gpio_isr_handler_remove(pinB);
    return false;
  }
  started_ = true;
  return true;
}

void QuadratureEncoder::end() {
  if (!started_) {
    return;
  }
  gpio_isr_handler_remove(static_cast<gpio_num_t>(pinA_));
  gpio_isr_handler_remove(static_cast<gpio_num_t>(pinB_));
  started_ = false;
}

EncoderSnapshot QuadratureEncoder::snapshot() const {
  portENTER_CRITICAL(&mux_);
  const EncoderSnapshot result{decoder_.ticks(), decoder_.invalidTransitions()};
  portEXIT_CRITICAL(&mux_);
  return result;
}

void QuadratureEncoder::onChange(void* context) {
  auto* encoder = static_cast<QuadratureEncoder*>(context);
  portENTER_CRITICAL_ISR(&encoder->mux_);
  encoder->decoder_.update(encoder->readState());
  portEXIT_CRITICAL_ISR(&encoder->mux_);
}

uint8_t QuadratureEncoder::readState() const {
  const uint32_t low = GPIO.in;
  const uint32_t high = GPIO.in1.val;
  const uint32_t a = pinA_ < 32 ? (low >> pinA_) & 1U
                              : (high >> (pinA_ - 32)) & 1U;
  const uint32_t b = pinB_ < 32 ? (low >> pinB_) & 1U
                              : (high >> (pinB_ - 32)) & 1U;
  return static_cast<uint8_t>((a << 1U) | b);
}

}  // namespace roboty
