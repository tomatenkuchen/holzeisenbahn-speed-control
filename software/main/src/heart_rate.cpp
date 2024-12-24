#include "heart_rate.hpp"

extern "C" {
#include "esp_random.h"
}

namespace {
uint8_t heart_rate;
}

uint8_t get_heart_rate() { return heart_rate; }
void update_heart_rate() { heart_rate = 60 + (uint8_t)(esp_random() % 21); }
