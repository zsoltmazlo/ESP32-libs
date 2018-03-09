#pragma once

#include <stdint.h>
#include <algorithm>
#include <array>
#include <iterator>
#include <string>
#include <utility>

// TODO we should use debug::printf instead of the normal one OR override the normal printf to use Serial with a mutex

template <uint8_t PIN, uint8_t RESOLUTION = 12>
std::pair<float, bool> readDallas18B20();

template <uint8_t PIN, uint8_t READ_COUNT, uint8_t RESOLUTION = 12>
std::pair<float, bool> readSensorValue();

template <uint8_t SIZE>
std::pair<float, bool> getMostCommonElement(const std::array<std::pair<float, bool>, SIZE>& elements);

/**
 * IMPLEMENTATION
 */

#include <OneWire.h>
#include "ArduinoJson.h"

extern HardwareSerial Serial;

template <uint8_t PIN, uint8_t RESOLUTION>
inline std::pair<float, bool> readDallas18B20() {
    // on pin a 4.7K resistor is necessary
    OneWire ds(PIN);
    uint8_t i, present = 0, type_s;
    uint8_t data[12];
    uint8_t addr[8];

    if (!ds.search(addr)) {
        ds.reset_search();
        delay(250);
        printf("TEMP | No device found on onewire bus.\n");
        return std::make_pair(0.0, false);
    }

    if (OneWire::crc8(addr, 7) != addr[7]) {
        printf("TEMP | CRC of device is invalid!\n");
        return std::make_pair(0.0, false);
    }

    ds.reset();
    ds.select(addr);

    // set resolution (we need to send three bytes with the write command)
    ds.write(0x4E);  // write on scratchPad
    ds.write(0x00);  // User byte 0 - Unused
    ds.write(0x00);  // User byte 1 - Unused
    switch (RESOLUTION) {
        case 9:
            ds.write(0x1F);
            break;
        case 10:
            ds.write(0x3F);
            break;
        case 11:
            ds.write(0x5F);
            break;
        case 12:
            ds.write(0x7F);
            break;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44);  // start conversion, without parasite power on at the end

    printf("TEMP | waiting %d ms\n", 100 * 1 << (RESOLUTION - 9));
    delay(100 * 1 << (RESOLUTION - 9));

    ds.reset();
    ds.select(addr);
    ds.write(0xBE);  // Read Scratchpad

    for (i = 0; i < 9; i++) {  // we need 9 bytes
        data[i] = ds.read();
    }

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
        raw = raw << 3;  // 9 bit resolution default
        if (data[7] == 0x10) {
            // "count remain" gives full 12 bit resolution
            raw = (raw & 0xFFF0) + RESOLUTION - data[6];
        }
    } else {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00)
            raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20)
            raw = raw & ~3;  // 10 bit res, 187.5 ms
        else if (cfg == 0x40)
            raw = raw & ~1;  // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
    }
    float celsius = (float)raw / 16.0;
    return std::make_pair(celsius, true);
}

template <uint8_t PIN, uint8_t READ_COUNT, uint8_t RESOLUTION>
inline std::pair<float, bool> readSensorValue() {
    std::array<std::pair<float, bool>, READ_COUNT> meas;

    // lambda for deciding whenever a measurement is valid or not
    auto is_valid = [](const std::pair<float, bool>& a) { return a.second; };

    // measure N times from sensor
    portMUX_TYPE mux;
    vPortCPUInitializeMutex(&mux);
    for (uint8_t i = 0; i < READ_COUNT; ++i) {
        // Prevent the RTOS kernel swapping out the task.
        taskENTER_CRITICAL(&mux);
        meas[i] = readDallas18B20<PIN, RESOLUTION>();
        taskEXIT_CRITICAL(&mux);
        printf("TEMP | Measurement %d.: %0.4f %s\n", i + 1, meas[i].first, meas[i].second ? "valid" : "invalid");
    }

    // if neither of the meas was valid, then return immediately
    if (std::none_of(meas.begin(), meas.end(), is_valid)) {
        return std::make_pair(0.0, false);
    }

    // we should only use the most common value among the measurements
    return getMostCommonElement<READ_COUNT>(meas);
}

template <uint8_t SIZE>
inline std::pair<float, bool> getMostCommonElement(const std::array<std::pair<float, bool>, SIZE>& elements) {
    // lambda expression if a current element is equals with the reference or not
    auto is_element_equals = [](float reference, const std::pair<float, bool>& current) {
        return current.second ? std::abs(current.first - reference) < 0.0001 : false;
    };

    auto most_common = std::make_pair(0.0f, false);
    int occurance = 0;

    for (auto elem : elements) {
        int t = std::count_if(elements.begin(), elements.end(),
                              std::bind(is_element_equals, elem.first, std::placeholders::_1));
        if (t > occurance) {
            occurance = t;
            most_common = elem;
        }

        // if current value's occurance is bigger then the half of the array size, then no need to check further
        // elements (there will be no other element which can be more times in the array)
        if (t > SIZE / 2) {
            break;
        }
    }

    printf("TEMP | Most common element: %0.2f (%d times)\n", most_common.first, occurance);

    // if occurance is not greater than SIZE/2, then report value as an invalid measurement
    if (occurance > SIZE / 2) {
        return std::make_pair(most_common.first, true);
    } else {
        return std::make_pair(0.0f, false);
    }
}