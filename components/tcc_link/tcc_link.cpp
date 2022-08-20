#include "tcc_link.h"

namespace esphome {
namespace tcc_link {

static const char *const TAG = "tcc_link.climate";

const LogString *opcode_to_string(uint8_t opcode) {
  switch (opcode) {
    case OPCODE_PING:
      return LOG_STR("OPCODE_PING");
    case OPCODE_PARAMETER:
      return LOG_STR("OPCODE_PARAMETER");
    case OPCODE_ERROR_HISTORY:
      return LOG_STR("OPCODE_ERROR_HISTORY");
    case OPCODE_SENSOR_QUERY:
      return LOG_STR("OPCODE_SENSOR_QUERY");
    case OPCODE_ACK:
      return LOG_STR("OPCODE_ACK");
    case OPCODE_SENSOR_VALUE:
      return LOG_STR("OPCODE_SENSOR_VALUE");
    case OPCODE_STATUS:
      return LOG_STR("OPCODE_STATUS");
    case OPCODE_TEMPERATURE:
      return LOG_STR("OPCODE_TEMPERATURE");
    case OPCODE_EXTENDED_STATUS:
      return LOG_STR("OPCODE_EXTENDED_STATUS");
    default:
      // return LOG_STR(str_sprintf("UNKNOWN OPCODE 1: 0x%02x", opcode));
      return LOG_STR("UNKNOWN");
  }
}

uint8_t temp_celcius_to_payload(float temp_celsius) {
  return static_cast<uint8_t>(temp_celsius + TEMPERATURE_CONVERSION_OFFSET) *
         TEMPERATURE_CONVERSION_RATIO;  // temp is +35 in (bit7-bit0)/2
}

uint8_t get_heat_cool_bits(uint8_t mode) {
  // TODO: figure the bits out:
  // https://github.com/issalig/toshiba_air_cond/blob/master/air/toshiba_serial.hpp#L285
  switch (mode) {
    case MODE_HEAT:
    case MODE_AUTO:
      return 0x55;  // 0b01010101  // = 0x01 + 0x04 * air->heat + 0x02 * air->cold;
    case MODE_COOL:
    case MODE_DRY:
    case MODE_FAN_ONLY:
      return 0x33;  // 0b00110011  // = 0x01 + 0x04 * air->heat + 0x02 * air->cold;
  }

  return 0;
}

uint8_t get_fan_bit_mask_for_mode(uint8_t mode) {
  switch (mode) {
    case MODE_HEAT:
    case MODE_AUTO:
      return 0b00100000 | 0b00001000;
    case MODE_COOL:
    case MODE_DRY:
    case MODE_FAN_ONLY:
      return 0b00010000 | 0b00001000;
  }

  return 0;
}

void write_set_parameter(struct DataFrame *command, uint8_t opcode2, uint8_t payload[], size_t payload_size) {
  command->source = TOSHIBA_REMOTE;
  command->dest = TOSHIBA_MASTER;
  command->opcode1 = OPCODE_PARAMETER;
  command->data_length = SET_PARAMETER_PAYLOAD_HEADER_SIZE + payload_size;
  command->data[0] = COMMAND_MODE_READ;
  command->data[1] = opcode2;

  for (size_t i = 0; i < payload_size; i++) {
    command->data[SET_PARAMETER_PAYLOAD_HEADER_SIZE + i] = payload[i];
  }

  command->data[SET_PARAMETER_PAYLOAD_HEADER_SIZE + payload_size] = command->calculate_crc();
}

void write_set_parameter(struct DataFrame *command, uint8_t opcode2, uint8_t single_type_payload) {
  uint8_t payload[1] = {single_type_payload};
  write_set_parameter(command, opcode2, payload, 1);
}

void write_set_parameter_flags(struct DataFrame *command, const struct TccState *state, uint8_t set_flags) {
  uint8_t payload[6] = {
      static_cast<uint8_t>(state->mode | set_flags),
      static_cast<uint8_t>(state->fan | get_fan_bit_mask_for_mode(state->mode)),
      temp_celcius_to_payload(state->target_temp),
      EMPTY_DATA,
      get_heat_cool_bits(state->mode),
      get_heat_cool_bits(state->mode),
  };
  write_set_parameter(command, OPCODE2_SET_TEMP_WITH_FAN, payload, sizeof(payload));
}

void write_set_parameter_mode(struct DataFrame *command, const struct TccState *state) {
  write_set_parameter(command, OPCODE2_SET_MODE, state->mode);
}

void write_set_parameter_power(struct DataFrame *command, const struct TccState *state) {
  write_set_parameter(command, OPCODE2_SET_POWER, state->power | 0b0010);
}

void write_set_parameter_vent(struct DataFrame *command, const struct TccState *state) {
  write_set_parameter(command, OPCODE2_SET_VENT, state->vent);
}

uint8_t to_tcc_power(const climate::ClimateMode mode) {
  switch (mode) {
    case climate::CLIMATE_MODE_OFF:
      return 0;
    default:
      return 1;
  }
}

uint8_t to_tcc_mode(const climate::ClimateMode mode) {
  switch (mode) {
    case climate::CLIMATE_MODE_OFF:
      return 0;
    case climate::CLIMATE_MODE_HEAT:
      return MODE_HEAT;
    case climate::CLIMATE_MODE_COOL:
      return MODE_COOL;
    case climate::CLIMATE_MODE_FAN_ONLY:
      return MODE_FAN_ONLY;
    case climate::CLIMATE_MODE_DRY:
      return MODE_DRY;
    case climate::CLIMATE_MODE_HEAT_COOL:
      return MODE_AUTO;
    default:
      return 0;
  }
}

uint8_t to_tcc_fan(const climate::ClimateFanMode fan) {
  switch (fan) {
    case climate::CLIMATE_FAN_AUTO:
      return FAN_PEED_AUTO;
    case climate::CLIMATE_FAN_LOW:
      return FAN_PEED_LOW;
    case climate::CLIMATE_FAN_MEDIUM:
      return FAN_PEED_MED;
    case climate::CLIMATE_FAN_HIGH:
      return FAN_PEED_HIGH;
    default:
      return climate::CLIMATE_FAN_OFF;
  }
}

climate::ClimateAction to_climate_action(const struct TccState *state) {
  if (state->power == 0)
    return climate::CLIMATE_ACTION_OFF;

  switch (state->mode) {
    case MODE_HEAT:
    case MODE_AUTO:
    case MODE_COOL:
      if (state->cooling) {
        return climate::CLIMATE_ACTION_COOLING;
      } else if (state->heating) {
        return climate::CLIMATE_ACTION_HEATING;
      }
      return climate::CLIMATE_ACTION_IDLE;
    case MODE_FAN_ONLY:
      return climate::CLIMATE_ACTION_FAN;
    case MODE_DRY:
      return climate::CLIMATE_ACTION_DRYING;
  }

  return climate::CLIMATE_ACTION_IDLE;
}

climate::ClimateMode to_climate_mode(const struct TccState *state) {
  if (state->power == 0)
    return climate::CLIMATE_MODE_OFF;

  switch (state->mode) {
    case MODE_HEAT:
      return climate::CLIMATE_MODE_HEAT;
    case MODE_COOL:
      return climate::CLIMATE_MODE_COOL;
    case MODE_FAN_ONLY:
      return climate::CLIMATE_MODE_FAN_ONLY;
    case MODE_DRY:
      return climate::CLIMATE_MODE_DRY;
    case MODE_AUTO:
      return climate::CLIMATE_MODE_HEAT_COOL;
  }

  return climate::CLIMATE_MODE_OFF;
}

climate::ClimateFanMode to_climate_fan(const struct TccState *state) {
  if (state->power == 0)
    return climate::CLIMATE_FAN_OFF;

  switch (state->fan) {
    case FAN_PEED_AUTO:
      return climate::CLIMATE_FAN_AUTO;
    case FAN_PEED_LOW:
      return climate::CLIMATE_FAN_LOW;
    case FAN_PEED_MED:
      return climate::CLIMATE_FAN_MEDIUM;
    case FAN_PEED_HIGH:
      return climate::CLIMATE_FAN_HIGH;
  }

  return climate::CLIMATE_FAN_ON;
}

TccLinkClimate::TccLinkClimate() {
  target_temperature = NAN;
  this->traits_.set_supports_action(true);
  this->traits_.set_supports_current_temperature(true);
  this->traits_.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_HEAT,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_FAN_ONLY,
      climate::CLIMATE_MODE_DRY,
      climate::CLIMATE_MODE_HEAT_COOL,
  });
  this->traits_.set_supported_fan_modes({
      climate::CLIMATE_FAN_OFF,
      climate::CLIMATE_FAN_AUTO,
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_HIGH,
  });
  this->traits_.set_supports_two_point_target_temperature(false);
  this->traits_.set_visual_min_temperature(18);
  this->traits_.set_visual_max_temperature(29);
  this->traits_.set_visual_temperature_step(0.5);
}

climate::ClimateTraits TccLinkClimate::traits() { return traits_; }

void TccLinkClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "TCC Link:");
  this->dump_traits_(TAG);
}

void TccLinkClimate::setup() {
  if (this->failed_crcs_sensor_ != nullptr) {
    this->failed_crcs_sensor_->publish_state(0);
  }
}

void log_data_frame(const std::string msg, const struct DataFrame *frame, size_t length = 0) {
  std::string res;
  char buf[5];
  size_t len = length > 0 ? length : frame->data_length;
  for (size_t i = 0; i < len; i++) {
    if (i > 0) {
      res += ':';
    }
    sprintf(buf, "%02X", frame->data[i]);
    res += buf;
  }
  ESP_LOGD(TAG, "%s: %02X:%02X:\x1B[32m%02X\033[0m:%02X:\033[2;100;37m%s\033[0m:%02X", msg.c_str(), frame->source,
           frame->dest, frame->opcode1, frame->data_length, res.c_str(), frame->crc());
}

void log_raw_data(const std::string prefix, const uint8_t raw[], size_t size) {
  std::string res;
  char buf[size];
  for (size_t i = 0; i < size; i++) {
    if (i > 0) {
      res += ':';
    }
    sprintf(buf, "%02X", raw[i]);
    res += buf;
  }
  ESP_LOGV(TAG, "%s%s", prefix.c_str(), res.c_str());
}

void TccLinkClimate::sync_from_received_state() {
  uint8_t changes = 0;

  auto new_mode = to_climate_mode(&tcc_state);
  if (new_mode != mode) {
    mode = new_mode;
    changes++;
  }

  auto new_action = to_climate_action(&tcc_state);
  if (new_action != action) {
    action = new_action;
    changes++;
  }

  auto new_fan_mode = to_climate_fan(&tcc_state);
  if (new_fan_mode != fan_mode) {
    fan_mode = new_fan_mode;
    changes++;
  }

  if (target_temperature != tcc_state.target_temp) {
    target_temperature = tcc_state.target_temp;
    changes++;
  }

  if (current_temperature != tcc_state.room_temp) {
    current_temperature = tcc_state.room_temp;
    changes++;
  }

  if (changes > 0) {
    this->publish_state();
  }

  if (this->vent_switch_) {
    this->vent_switch_->publish_state(tcc_state.vent);
  }
}

void TccLinkClimate::process_received_data(const struct DataFrame *frame) {
  switch (frame->source) {
    case 0x00:
    case TOSHIBA_MASTER:
      // status update

      last_master_alive_millis_ = millis();
      if (this->connected_binary_sensor_) {
        this->connected_binary_sensor_->publish_state(true);
      }

      switch (frame->opcode1) {
        case OPCODE_PING:
          log_data_frame("PING", frame);
          break;
          // case OPCODE_ACK:
          // ESP_LOGD(TAG, " Command acknoledged");
          break;
        case OPCODE_PARAMETER:
          // master reporting it's state
          // e.g. 01:52:11:04:80:86:A1:05:E4
          /*
          00 52 11 04 80 86 24 00 65  heat
                |-opc1      |  |- mode  bit7-bit5, power bit0, bit2 ???
                            |- 0010 0100 -> mode bit7-bit5  bit4-bit0 ???
                              ---
          */
          log_data_frame("MASTER PARAMETERS", frame);

          tcc_state.power = (frame->data[3] & STATUS_DATA_POWER_MASK);
          tcc_state.mode =
              (frame->data[STATUS_DATA_MODEPOWER_BYTE] & STATUS_DATA_MODE_MASK) >> STATUS_DATA_MODE_SHIFT_BITS;
          tcc_state.cooling = (frame->data[STATUS_DATA_MODEPOWER_BYTE] & 0b00001000) >> 3;
          tcc_state.heating = (frame->data[STATUS_DATA_MODEPOWER_BYTE] & 0b00000001);
          // tcc_state.heating = (frame->data[3] & 0b0100) >> 2;
          tcc_state.preheating = (frame->data[3] & 0b0100) >> 2;

          ESP_LOGD(TAG, "Mode: %02X, Cooling: %d, Heating: %d, Preheating: %d", tcc_state.mode, tcc_state.cooling,
                   tcc_state.heating, tcc_state.preheating);

          sync_from_received_state();

          break;
        case OPCODE_STATUS:
          // sync power, mode, fan and target temp from the unit to the climate
          // component

          log_data_frame("STATUS", frame);

          // this message means that the command sent to master was confirmed
          // (may be it can return an error, but no idea how to read that at the
          // moment)
          if (last_unconfirmed_command_.has_value()) {
            // TODO: check if this is the right command being confirmed

            last_unconfirmed_command_ = {};  // reset last command
          }

          tcc_state.power = (frame->data[STATUS_DATA_MODEPOWER_BYTE] & STATUS_DATA_POWER_MASK);
          tcc_state.mode =
              (frame->data[STATUS_DATA_MODEPOWER_BYTE] & STATUS_DATA_MODE_MASK) >> STATUS_DATA_MODE_SHIFT_BITS;
          tcc_state.fan = (frame->data[STATUS_DATA_FANVENT_BYTE] & STATUS_DATA_FAN_MASK) >> STATUS_DATA_FAN_SHIFT_BITS;
          tcc_state.vent =
              (frame->data[STATUS_DATA_FANVENT_BYTE] & STATUS_DATA_VENT_MASK) >> STATUS_DATA_VENT_SHIFT_BITS;
          tcc_state.target_temp =
              static_cast<float>(frame->data[STATUS_DATA_TARGET_TEMP_BYTE] & TEMPERATURE_DATA_MASK) /
                  TEMPERATURE_CONVERSION_RATIO -
              TEMPERATURE_CONVERSION_OFFSET;
          // don't set heating of cooling from command status update, since the
          // actuall will be delayed and will e reported via MASTER PARAMETER
          // tcc_state.cooling = (frame->data[7] & 0b1000) >> 3;
          // tcc_state.heating = (frame->data[7] & 0b0001);
          tcc_state.preheating = (frame->data[4] & 0b10) >> 1;

          ESP_LOGD(TAG, "Mode: %02X, Preheating: %d", tcc_state.mode, tcc_state.preheating);

          sync_from_received_state();

          break;
        case OPCODE_EXTENDED_STATUS:
          // sync power, mode, fan and target temp from the unit to the climate
          // component

          log_data_frame("EXTENDED STATUS", frame);

          tcc_state.power = (frame->data[STATUS_DATA_MODEPOWER_BYTE] & STATUS_DATA_POWER_MASK);
          tcc_state.mode =
              (frame->data[STATUS_DATA_MODEPOWER_BYTE] & STATUS_DATA_MODE_MASK) >> STATUS_DATA_MODE_SHIFT_BITS;
          tcc_state.fan = (frame->data[STATUS_DATA_FANVENT_BYTE] & STATUS_DATA_FAN_MASK) >> STATUS_DATA_FAN_SHIFT_BITS;
          tcc_state.vent =
              (frame->data[STATUS_DATA_FANVENT_BYTE] & STATUS_DATA_VENT_MASK) >> STATUS_DATA_VENT_SHIFT_BITS;
          tcc_state.target_temp =
              static_cast<float>(frame->data[STATUS_DATA_TARGET_TEMP_BYTE] & TEMPERATURE_DATA_MASK) /
                  TEMPERATURE_CONVERSION_RATIO -
              TEMPERATURE_CONVERSION_OFFSET;

          if (frame->data[STATUS_DATA_TARGET_TEMP_BYTE + 1] > 1) {
            tcc_state.room_temp =
                static_cast<float>(frame->data[STATUS_DATA_TARGET_TEMP_BYTE + 1]) / TEMPERATURE_CONVERSION_RATIO -
                TEMPERATURE_CONVERSION_OFFSET;
          }

          tcc_state.preheating = (frame->data[4] & 0b10) >> 1;
          ESP_LOGD(TAG, "Mode: %02X, Preheating: %d", tcc_state.mode, tcc_state.preheating);

          sync_from_received_state();

          break;
        default:
          log_data_frame("MASTER", frame);
          break;
      }

      break;
    case TOSHIBA_REMOTE:
      // command
      log_data_frame("REMOTE", frame);
      if (frame->opcode1 == OPCODE_TEMPERATURE) {
        // current temperature is reported by the remote
        if (frame->data[3] > 1) {
          tcc_state.room_temp =
              static_cast<float>(frame->data[3]) / TEMPERATURE_CONVERSION_RATIO - TEMPERATURE_CONVERSION_OFFSET;
          sync_from_received_state();
        }
      }
      break;
    default:
      log_data_frame("UNKNOWN", frame);
      break;
  }
}

bool TccLinkClimate::receive_data(const std::vector<uint8_t> data) {
  auto frame = DataFrame();

  for (size_t i = 0; i < data.size(); i++) {
    frame.raw[i] = data[i];
  }

  return receive_data_frame(&frame);
}

bool TccLinkClimate::receive_data_frame(const struct DataFrame *frame) {
  if (frame->crc() != frame->calculate_crc()) {
    ESP_LOGW(TAG, "CRC check failed");
    log_data_frame("Failed frame", frame);

    if (this->failed_crcs_sensor_ != nullptr) {
      this->failed_crcs_sensor_->publish_state(this->failed_crcs_sensor_->state + 1);
    }

    return false;
  }

  this->set_data_received_callback_.call(frame);

  process_received_data(frame);

  return true;
}

void TccLinkClimate::loop() {
  // TODO: check if last_unconfirmed_command_ was not confirmed after a timeout
  // and log warning/error

  if (!this->write_queue_.empty() && (millis() - last_received_frame_millis_) >= FRAME_SEND_MILLIS_FROM_LAST_RECEIVE &&
      (millis() - last_sent_frame_millis_) >= FRAME_SEND_MILLIS_FROM_LAST_SEND) {
    last_sent_frame_millis_ = millis();
    auto frame = this->write_queue_.front();
    last_unconfirmed_command_ = frame;
    log_data_frame("Write frame", &frame);
    this->write_array(frame.raw, frame.size());
    this->write_queue_.pop();
    if (this->write_queue_.empty()) {
      ESP_LOGD(TAG, "All frames written");
    }
  }

  uint8_t bytes_read = 0;

  while (available()) {
    int byte = read();
    if (byte >= 0) {
      bytes_read++;

      if (!can_read_packet)
        continue;  // wait until can read packet

      if (data_reader.put(byte)) {
        // packet complete

        last_received_frame_millis_ = millis();

        auto frame = data_reader.frame;

        if (!receive_data_frame(&frame)) {
        }

        data_reader.reset();

        // read next packet (if any in the next loop)
        // the smallest packet (ALIVE) is 32ms wide,
        // which means there are max ~31 packets per second.
        // and the loop runs 33-50 times per second.
        // so should be enough throughput to process packets.
        // this ensure that each packet is interpreted separately
        break;
      }
    } else {
      ESP_LOGW(TAG, "Unable to read data");
    }
  }

  if (bytes_read > 0) {
    loops_with_reads_++;
    loops_without_reads_ = 0;

    // ESP_LOGV(TAG, "Bytes of data read: %d", bytes_read);
    // if (!data_reader.complete) {
    //   log_data_frame("Pending", data_reader.frame);
    // }

    last_read_millis_ = millis();
  } else {
    loops_without_reads_++;
    loops_with_reads_ = 0;

    if (last_read_millis_ > 0) {
      auto millis_since_last_read = millis() - last_read_millis_;
      if (millis_since_last_read >= PACKET_MIN_WAIT_MILLIS) {
        // can start reading packet

        if (!data_reader.complete && data_reader.data_index_ > 0) {
          // ESP_LOGW(TAG, "Reset pending frame buffer (%d)",
          // data_reader.data_index_); log_raw_data("Pending: ",
          // data_reader.frame.raw, data_reader.data_index_);
        }
        can_read_packet = true;
        data_reader.reset();
        last_read_millis_ = 0;
      }
    }
  }

  if (last_master_alive_millis_ > 0 && (millis() - last_master_alive_millis_) > LAST_ALIVE_TIMEOUT_MILLIS) {
    // not connected
    if (this->connected_binary_sensor_) {
      this->connected_binary_sensor_->publish_state(false);
    }
  }
}

size_t TccLinkClimate::send_new_state(const struct TccState *new_state) {
  auto commands = create_commands(new_state);
  if (commands.empty()) {
    ESP_LOGD(TAG, "New state has not changed. Nothing to send");
  } else {
    ESP_LOGD(TAG, "Send %d commands", commands.size());
    for (auto cmd : commands) {
      send_command(cmd);
    }
  }

  return commands.size();
}

std::vector<DataFrame> TccLinkClimate::create_commands(const struct TccState *new_state) {
  auto commands = std::vector<DataFrame>();

  if (new_state->power != tcc_state.power) {
    if (new_state->power) {
      // turn on
      ESP_LOGD(TAG, "Turning on");
      auto command = DataFrame{};
      write_set_parameter_power(&command, new_state);
      commands.push_back(command);
    } else {
      // turn off
      ESP_LOGD(TAG, "Turning off");
      auto command = DataFrame{};
      write_set_parameter_power(&command, new_state);
      commands.push_back(command);
      // don't process other changes when turning off
      return commands;
    }
  }

  if (new_state->mode != tcc_state.mode) {
    ESP_LOGD(TAG, "Changing mode");
    auto command = DataFrame{};
    write_set_parameter_mode(&command, new_state);
    commands.push_back(command);
  }

  if (new_state->fan != tcc_state.fan) {
    ESP_LOGD(TAG, "Changing fan");
    auto command = DataFrame{};
    write_set_parameter_flags(&command, new_state, COMMAND_SET_FAN);
    commands.push_back(command);
  }

  if (new_state->target_temp != tcc_state.target_temp) {
    ESP_LOGD(TAG, "Changing target temperature");
    auto command = DataFrame{};
    write_set_parameter_flags(&command, new_state, COMMAND_SET_TEMP);
    commands.push_back(command);
  }

  if (new_state->vent != tcc_state.vent) {
    ESP_LOGD(TAG, "Changing vent");
    auto command = DataFrame{};
    write_set_parameter_vent(&command, new_state);
    commands.push_back(command);
  }

  return commands;
}

void TccLinkClimate::control(const climate::ClimateCall &call) {
  TccState new_state = TccState{tcc_state};

  if (call.get_mode().has_value()) {
    ESP_LOGD(TAG, "Control mode");
    auto mode = call.get_mode().value();
    new_state.power = to_tcc_power(mode);
    new_state.mode = to_tcc_mode(mode);
  }

  if (call.get_fan_mode().has_value()) {
    ESP_LOGD(TAG, "Control fan");
    new_state.fan = to_tcc_fan(call.get_fan_mode().value());
  }

  if (call.get_target_temperature().has_value()) {
    ESP_LOGD(TAG, "Control target temperature");
    new_state.target_temp = call.get_target_temperature().value();
  }

  send_new_state(&new_state);
}

void TccLinkClimate::send_command(const struct DataFrame command) {
  log_data_frame("Enqueue command", &command);
  this->write_queue_.push(command);
}

bool TccLinkClimate::control_vent(bool state) {
  if (!tcc_state.power) {
    ESP_LOGW(TAG, "Can't control vent when powered off");
    return false;
  }
  ESP_LOGD(TAG, "Control vent: %d", state);
  TccState new_state = TccState{tcc_state};
  new_state.vent = state;
  return send_new_state(&new_state) > 0;
}

void TccLinkVentSwitch::write_state(bool state) {
  if (this->climate_->control_vent(state)) {
    // don't publish state. wait for the unit to report it's state
  }
}

}  // namespace tcc_link
}  // namespace esphome
