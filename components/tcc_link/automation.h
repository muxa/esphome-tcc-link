#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "tcc_link.h"

namespace esphome
{
  namespace tcc_link
  {

    class TccLinkOnDataReceivedTrigger : public Trigger<std::vector<uint8_t>>
    {
    public:
      TccLinkOnDataReceivedTrigger(TccLinkClimate *climate)
      {
        climate->add_on_data_received_callback(
            [this](const struct DataFrame *frame)
            {
              this->trigger(frame->get_data());
            });
      }
    };

  } // namespace tcc_link
} // namespace esphome