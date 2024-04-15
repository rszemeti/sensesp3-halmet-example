#ifndef HALMET_SRC_HALMET_ANY_H_
#define HALMET_SRC_HALMET_ANY_H_

#include "elapsedMillis.h"
#include "sensesp/transforms/transform.h"

using namespace sensesp;

/**
 * @brief A transform that returns true if any of the input values are true.
 *
 * If any of the input values are older than the expiration duration, no
 * value is returned. Otherwise a value is emitted if any of the input values
 * is updated.
 *
 */
class Any : public Transform<bool, bool> {
 public:
  Any(int num_channels, unsigned long expiration_duration = 1000)
      : expiration_duration_{expiration_duration} {
    last_update_ = new elapsedMillis[num_channels];
    value_ = new bool[num_channels];
  }

  virtual void set_input(bool input, uint8_t input_channel = 0) override {
    last_update_[input_channel] = 0;
    value_[input_channel] = input;
    for (int i = 0; i < input_channel; i++) {
      if (value_[i]) {
        this->emit(true);
        return;
      }
    }
    this->emit(false);
  }

 private:
  unsigned long expiration_duration_;
  elapsedMillis* last_update_;
  bool* value_;
};

#endif  // HALMET_SRC_HALMET_ANY_H_
