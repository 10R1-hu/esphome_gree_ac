#pragma once

#include "esphome/components/text/text.h"
#include "esphome/core/component.h"

namespace esphome {
namespace sinclair_ac {

class SinclairACText : public text::Text, public Component {
 public:
  void control(const std::string &value) override {
    this->state = value;
    this->publish_state(value);
  }
};

}  // namespace sinclair_ac
}  // namespace esphome
