# Eisenbahn Project for my Son

This project supports building a wood train for my son. It should receive
a speed requirement via BLE, and return a current speed indication

- including a subscription.

The base is an ESP32C3 XIAO module for providing BLE functionality, and
PWM support for speed control.

Maybe we add a LED for lighting later on.

This code is mainly derived form the ble-nimble-gatt-server example in
[esp-idf](https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/ble_get_started/nimble/NimBLE_GATT_Server)
