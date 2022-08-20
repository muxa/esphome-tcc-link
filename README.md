# ESPHome Toshiba TCC-Link AC Component

ESPHome component to integrate with Toshiba Air Conditioners via TCC-Link protocol (AB line).

Requires reader & writer circuit for the AB line: https://github.com/issalig/toshiba_air_cond

## Install

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/muxa/esphome-tcc-link

```

## Usage

```yaml
uart:
  tx_pin: GPIO1
  rx_pin: GPIO3
  baud_rate: 2400
  parity: EVEN

climate:
  - platform: tcc_link
    name: "Toshiba AC"
    id: toshiba_ac
    connected:
      name: "Toshiba AC Connected"
    failed_crcs:
      name: "Toshiba AC Failed CRCs"
    vent:
      name: "Toshiba AC Vent Switch"
```
