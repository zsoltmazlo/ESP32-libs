#pragma once

#include <Adafruit_GFX.h>
#include <Arduino.h>
#include <array>
#include <functional>

#define BLACK 0
#define WHITE 1
#define INVERSE 2

#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

#define SSD1306_LCDWIDTH 128
#define SSD1306_LCDHEIGHT 64

template <uint8_t FRAMES>
class SSD1306 : public Adafruit_GFX {
   public:
    SSD1306(int8_t sclPin, int8_t sdaPin, uint8_t rst);

    void begin(uint32_t frequency = 100000, uint8_t switchvcc = SSD1306_SWITCHCAPVCC);

    void clearDisplay(void);

    void invertDisplay(uint8_t i);

    void display();

    void dim(bool dim);

    void drawPixel(int16_t x, int16_t y, uint16_t color);

    void text(int16_t x, int16_t y, const char *fmt, ...);

    void centerredText(uint16_t x, int16_t y, const char *fmt, ...);

    void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) override;

    void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) override;

    void setCurrentFrame(uint8_t currentFrame) { currentFrame_ = currentFrame; }

    uint8_t getCurrentFrameIndex() const { return currentFrame_; }

    uint8_t getFrameCount() const { return FRAMES; }

    void setFrame(uint8_t position, std::function<void(SSD1306<FRAMES> &)> fn);

    void updateFrames();

    void changeFrame();

    void showIndicators() {
        // always override the top 10 pixel with indicators
        fillRect(0, 0, 128, 10, 0);
        for (uint8_t i = 0; i < FRAMES; ++i) {
            if (i == currentFrame_) {
                fillRect(64 + FRAMES / 2 * -12 + i * 12, 2, 4, 4, 1);
            }
            drawRect(64 + FRAMES / 2 * -12 + i * 12, 2, 4, 4, 1);
        }
    }

   private:
    int8_t sclPin_, sdaPin_, rstPin_;
    uint8_t vccSource_;

    using FrameBuffer = uint8_t[SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH / 8];

    std::array<FrameBuffer, FRAMES> frames_;

    uint8_t currentFrame_;

    std::array<std::function<void(SSD1306<FRAMES> &)>, FRAMES> updaters_;

    inline void drawFastVLineInternal(int16_t x, int16_t y, int16_t h, uint16_t color) __attribute__((always_inline));

    inline void drawFastHLineInternal(int16_t x, int16_t y, int16_t w, uint16_t color) __attribute__((always_inline));

    void ssd1306_command(uint8_t c);
};

/**
 * IMPLEMENTATION
 */

#ifdef __AVR__
#include <avr/pgmspace.h>
#elif defined(ESP8266)
#include <pgmspace.h>
#else
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif

#include <Arduino.h>
#include <stdlib.h>

#include <cstdarg>
#include <cstdio>
#include <utility>

#include <Wire.h>
#include "Adafruit_GFX.h"

#define SSD1306_I2C_ADDRESS 0x3C  // 011110+SA0+RW - 0x3C or 0x3D

#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA

#define SSD1306_SETVCOMDETECT 0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9

#define SSD1306_SETMULTIPLEX 0xA8

#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10

#define SSD1306_SETSTARTLINE 0x40

#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR 0x22

#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8

#define SSD1306_SEGREMAP 0xA0

#define SSD1306_CHARGEPUMP 0x8D
#define SSD1306_DEACTIVATE_SCROLL 0x2E

// the most basic function, set a single pixel
template <uint8_t FRAMES>
inline void SSD1306<FRAMES>::drawPixel(int16_t x, int16_t y, uint16_t color) {
    if ((x < 0) || (x >= width()) || (y < 0) || (y >= height())) {
        return;
    }

    // check rotation, move pixel around if necessary
    switch (getRotation()) {
        case 1:
            std::swap(x, y);
            x = WIDTH - x - 1;
            break;
        case 2:
            x = WIDTH - x - 1;
            y = HEIGHT - y - 1;
            break;
        case 3:
            std::swap(x, y);
            y = HEIGHT - y - 1;
            break;
    }

    // x is which column
    switch (color) {
        case WHITE:
            frames_[currentFrame_][x + (y / 8) * SSD1306_LCDWIDTH] |= (1 << (y & 7));
            break;
        case BLACK:
            frames_[currentFrame_][x + (y / 8) * SSD1306_LCDWIDTH] &= ~(1 << (y & 7));
            break;
        case INVERSE:
            frames_[currentFrame_][x + (y / 8) * SSD1306_LCDWIDTH] ^= (1 << (y & 7));
            break;
    }
}

template <uint8_t FRAMES>
inline SSD1306<FRAMES>::SSD1306(int8_t sclPin, int8_t sdaPin, uint8_t rstPin)
    : Adafruit_GFX(SSD1306_LCDWIDTH, SSD1306_LCDHEIGHT),
      sclPin_(sclPin),
      sdaPin_(sdaPin),
      rstPin_(rstPin),
      currentFrame_(0) {}

template <uint8_t FRAMES>
inline void SSD1306<FRAMES>::begin(uint32_t frequency, uint8_t vccstate) {
    vccSource_ = vccstate;

    // I2C Init
    if (sclPin_ == -1 || sdaPin_ == -1) {
        Wire.begin();
    } else {
        Wire.begin(sdaPin_, sclPin_);
    }

    Wire.setClock(frequency);
    // Setup reset pin direction (used by both SPI and I2C)
    pinMode(rstPin_, OUTPUT);
    digitalWrite(rstPin_, HIGH);
    // VDD (3.3V) goes high at start, lets just chill for a ms
    delay(1);
    // bring reset low
    digitalWrite(rstPin_, LOW);
    // wait 10ms
    delay(10);
    // bring out of reset
    digitalWrite(rstPin_, HIGH);
    // turn on VCC (9V?)

    // Init sequence
    ssd1306_command(SSD1306_DISPLAYOFF);          // 0xAE
    ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);  // 0xD5
    ssd1306_command(0x80);                        // the suggested ratio 0x80

    ssd1306_command(SSD1306_SETMULTIPLEX);  // 0xA8
    ssd1306_command(SSD1306_LCDHEIGHT - 1);

    ssd1306_command(SSD1306_SETDISPLAYOFFSET);    // 0xD3
    ssd1306_command(0x0);                         // no offset
    ssd1306_command(SSD1306_SETSTARTLINE | 0x0);  // line #0
    ssd1306_command(SSD1306_CHARGEPUMP);          // 0x8D
    if (vccSource_ == SSD1306_EXTERNALVCC) {
        ssd1306_command(0x10);
    } else {
        ssd1306_command(0x14);
    }
    ssd1306_command(SSD1306_MEMORYMODE);  // 0x20 horizontal addressing mode
    ssd1306_command(0x00);                // 0x0 act like ks0108
    ssd1306_command(SSD1306_SEGREMAP | 0x1);
    ssd1306_command(SSD1306_COMSCANDEC);

    ssd1306_command(SSD1306_SETCOMPINS);  // 0xDA
    ssd1306_command(0x12);
    ssd1306_command(SSD1306_SETCONTRAST);  // 0x81
    if (vccSource_ == SSD1306_EXTERNALVCC) {
        ssd1306_command(0x9F);
    } else {
        ssd1306_command(0xCF);
    }

    ssd1306_command(SSD1306_SETPRECHARGE);  // 0xd9
    if (vccSource_ == SSD1306_EXTERNALVCC) {
        ssd1306_command(0x22);
    } else {
        ssd1306_command(0xF1);
    }
    ssd1306_command(SSD1306_SETVCOMDETECT);  // 0xDB
    ssd1306_command(0x40);
    ssd1306_command(SSD1306_DISPLAYALLON_RESUME);  // 0xA4
    ssd1306_command(SSD1306_NORMALDISPLAY);        // 0xA6

    ssd1306_command(SSD1306_DEACTIVATE_SCROLL);

    ssd1306_command(SSD1306_DISPLAYON);  //--turn on oled panel
}

template <uint8_t FRAMES>
inline void SSD1306<FRAMES>::invertDisplay(uint8_t i) {
    if (i) {
        ssd1306_command(SSD1306_INVERTDISPLAY);
    } else {
        ssd1306_command(SSD1306_NORMALDISPLAY);
    }
}

template <uint8_t FRAMES>
inline void SSD1306<FRAMES>::text(int16_t x, int16_t y, const char *fmt, ...) {
    static char buffer[64];
    va_list args;
    va_start(args, fmt);
    vsprintf(buffer, fmt, args);
    setCursor(x, y);
    this->printf(buffer);
    va_end(args);
}

template <uint8_t FRAMES>
inline void SSD1306<FRAMES>::centerredText(uint16_t x, int16_t y, const char *fmt, ...) {
    static char buffer[24];
    va_list args;
    va_start(args, fmt);
    vsprintf(buffer, fmt, args);

    // one character is 5+1 pixel = full length: strlen * 6 - 1
    uint8_t offset = (strlen(buffer) * 6 - 1) / 2;

    setCursor(x - offset, y);
    this->printf(buffer);
    va_end(args);
}

template <uint8_t FRAMES>
inline void SSD1306<FRAMES>::changeFrame() {
    // currentFrame_ = (currentFrame_ + 1) % 3;
    uint8_t nextFrame = (currentFrame_ + 1) % 3;

    // first, create temporary save of the current frame
    FrameBuffer backup;
    memcpy(backup, frames_[currentFrame_], sizeof(frames_[currentFrame_]));

    // buffer is constructed as follows:
    // the very first 128B contains the first 8 rows of the display
    //  00000000   00000000   00000000   00000000 ... 00000000
    //  |      |   |      |                           |      |
    // 0,7    0,0 1,7    1,0                        127,7  127,0
    // thus, if we move every byte to the previous position for each 128 B,
    // then we shift the whole display! and every 128th byte could be the
    // next frame's first byte

    // shift 128 pixel
    uint8_t w = SSD1306_LCDWIDTH;
    uint8_t c = 4;
    for (uint8_t shift = 0; shift < w; shift += c) {
        // shift each row
        memcpy(frames_[currentFrame_] + 0 * w, frames_[currentFrame_] + 0 * w + c, sizeof(uint8_t[w - c]));
        memcpy(frames_[currentFrame_] + 1 * w, frames_[currentFrame_] + 1 * w + c, sizeof(uint8_t[w - c]));
        memcpy(frames_[currentFrame_] + 2 * w, frames_[currentFrame_] + 2 * w + c, sizeof(uint8_t[w - c]));
        memcpy(frames_[currentFrame_] + 3 * w, frames_[currentFrame_] + 3 * w + c, sizeof(uint8_t[w - c]));
        memcpy(frames_[currentFrame_] + 4 * w, frames_[currentFrame_] + 4 * w + c, sizeof(uint8_t[w - c]));
        memcpy(frames_[currentFrame_] + 5 * w, frames_[currentFrame_] + 5 * w + c, sizeof(uint8_t[w - c]));
        memcpy(frames_[currentFrame_] + 6 * w, frames_[currentFrame_] + 6 * w + c, sizeof(uint8_t[w - c]));
        memcpy(frames_[currentFrame_] + 7 * w, frames_[currentFrame_] + 7 * w + c, sizeof(uint8_t[w - c]));

        // last columns will be the next column to be shift in
        memcpy(frames_[currentFrame_] + 1 * w - c, frames_[nextFrame] + 0 * w + shift, sizeof(uint8_t[c]));
        memcpy(frames_[currentFrame_] + 2 * w - c, frames_[nextFrame] + 1 * w + shift, sizeof(uint8_t[c]));
        memcpy(frames_[currentFrame_] + 3 * w - c, frames_[nextFrame] + 2 * w + shift, sizeof(uint8_t[c]));
        memcpy(frames_[currentFrame_] + 4 * w - c, frames_[nextFrame] + 3 * w + shift, sizeof(uint8_t[c]));
        memcpy(frames_[currentFrame_] + 5 * w - c, frames_[nextFrame] + 4 * w + shift, sizeof(uint8_t[c]));
        memcpy(frames_[currentFrame_] + 6 * w - c, frames_[nextFrame] + 5 * w + shift, sizeof(uint8_t[c]));
        memcpy(frames_[currentFrame_] + 7 * w - c, frames_[nextFrame] + 6 * w + shift, sizeof(uint8_t[c]));
        memcpy(frames_[currentFrame_] + 8 * w - c, frames_[nextFrame] + 7 * w + shift, sizeof(uint8_t[c]));

        // always override the top 10 pixel with indicators
        fillRect(0, 0, 128, 10, 0);
        for (uint8_t i = 0; i < FRAMES; ++i) {
            drawRect(64 + FRAMES / 2 * -12 + i * 12, 2, 4, 4, 1);
        }
        display();
    }

    // after that, we can restore previous frame and set the next frame as current
    memcpy(frames_[currentFrame_], backup, sizeof(frames_[currentFrame_]));
    currentFrame_ = nextFrame;
    display();
}

template <uint8_t FRAMES>
inline void SSD1306<FRAMES>::setFrame(uint8_t position, std::function<void(SSD1306<FRAMES> &)> fn) {
    updaters_[position] = fn;
}

template <uint8_t FRAMES>
inline void SSD1306<FRAMES>::updateFrames() {
    uint8_t currentFrame = currentFrame_;
    for (uint8_t i = 0; i < FRAMES; ++i) {
        currentFrame_ = i;
        updaters_[i](*this);
    }
    currentFrame_ = currentFrame;
    display();
}

template <uint8_t FRAMES>
inline void SSD1306<FRAMES>::ssd1306_command(uint8_t c) {
    uint8_t control = 0x00;  // Co = 0, D/C = 0
    Wire.beginTransmission(SSD1306_I2C_ADDRESS);
    Wire.write(control);
    Wire.write(c);
    Wire.endTransmission();
}

// Dim the display
// dim = true: display is dimmed
// dim = false: display is normal
template <uint8_t FRAMES>
inline void SSD1306<FRAMES>::dim(bool dim) {
    uint8_t contrast;

    if (dim) {
        contrast = 0;  // Dimmed display
    } else {
        if (vccSource_ == SSD1306_EXTERNALVCC) {
            contrast = 0x9F;
        } else {
            contrast = 0xCF;
        }
    }
    // the range of contrast to too small to be really useful
    // it is useful to dim the display
    ssd1306_command(SSD1306_SETCONTRAST);
    ssd1306_command(contrast);
}

template <uint8_t FRAMES>
inline void SSD1306<FRAMES>::display(void) {
    ssd1306_command(SSD1306_COLUMNADDR);
    ssd1306_command(0);                     // Column start address (0 = reset)
    ssd1306_command(SSD1306_LCDWIDTH - 1);  // Column end address (127 = reset)
    ssd1306_command(SSD1306_PAGEADDR);
    ssd1306_command(0);                            // Page start address (0 = reset)
    ssd1306_command((SSD1306_LCDHEIGHT / 8) - 1);  // Page end address

    // I2C
    for (uint16_t i = 0; i < (SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8); i++) {
        // send a bunch of data in one xmission
        Wire.beginTransmission(SSD1306_I2C_ADDRESS);
        Wire.write(0x40);
        for (uint8_t x = 0; x < 16; x++) {
            Wire.write(frames_[currentFrame_][i]);
            i++;
        }
        i--;
        Wire.endTransmission();
    }
}

// clear everything
template <uint8_t FRAMES>
inline void SSD1306<FRAMES>::clearDisplay(void) {
    memset(frames_[currentFrame_], 0, (SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8));
}

template <uint8_t FRAMES>
inline void SSD1306<FRAMES>::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
    bool bSwap = false;
    switch (rotation) {
        case 1:
            // 90 degree rotation, swap x & y for rotation, then invert x
            bSwap = true;
            std::swap(x, y);
            x = WIDTH - x - 1;
            break;
        case 2:
            // 180 degree rotation, invert x and y - then shift y around for
            // height.
            x = WIDTH - x - 1;
            y = HEIGHT - y - 1;
            x -= (w - 1);
            break;
        case 3:
            // 270 degree rotation, swap x & y for rotation, then invert y  and
            // adjust y for w (not to become h)
            bSwap = true;
            std::swap(x, y);
            y = HEIGHT - y - 1;
            y -= (w - 1);
            break;
    }

    if (bSwap) {
        drawFastVLineInternal(x, y, w, color);
    } else {
        drawFastHLineInternal(x, y, w, color);
    }
}

template <uint8_t FRAMES>
inline void SSD1306<FRAMES>::drawFastHLineInternal(int16_t x, int16_t y, int16_t w, uint16_t color) {
    // Do bounds/limit checks
    if (y < 0 || y >= HEIGHT) {
        return;
    }

    // make sure we don't try to draw below 0
    if (x < 0) {
        w += x;
        x = 0;
    }

    // make sure we don't go off the edge of the display
    if ((x + w) > WIDTH) {
        w = (WIDTH - x);
    }

    // if our width is now negative, punt
    if (w <= 0) {
        return;
    }

    // set up the pointer for  movement through the buffer
    register uint8_t *pBuf = frames_[currentFrame_];
    // adjust the buffer pointer for the current row
    pBuf += ((y / 8) * SSD1306_LCDWIDTH);
    // and offset x columns in
    pBuf += x;

    register uint8_t mask = 1 << (y & 7);

    switch (color) {
        case WHITE:
            while (w--) {
                *pBuf++ |= mask;
            };
            break;
        case BLACK:
            mask = ~mask;
            while (w--) {
                *pBuf++ &= mask;
            };
            break;
        case INVERSE:
            while (w--) {
                *pBuf++ ^= mask;
            };
            break;
    }
}

template <uint8_t FRAMES>
inline void SSD1306<FRAMES>::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
    bool bSwap = false;
    switch (rotation) {
        case 0:
            break;
        case 1:
            // 90 degree rotation, swap x & y for rotation, then invert x and
            // adjust x for h (now to become w)
            bSwap = true;
            std::swap(x, y);
            x = WIDTH - x - 1;
            x -= (h - 1);
            break;
        case 2:
            // 180 degree rotation, invert x and y - then shift y around for
            // height.
            x = WIDTH - x - 1;
            y = HEIGHT - y - 1;
            y -= (h - 1);
            break;
        case 3:
            // 270 degree rotation, swap x & y for rotation, then invert y
            bSwap = true;
            std::swap(x, y);
            y = HEIGHT - y - 1;
            break;
    }

    if (bSwap) {
        drawFastHLineInternal(x, y, h, color);
    } else {
        drawFastVLineInternal(x, y, h, color);
    }
}

template <uint8_t FRAMES>
inline void SSD1306<FRAMES>::drawFastVLineInternal(int16_t x, int16_t __y, int16_t __h, uint16_t color) {
    // do nothing if we're off the left or right side of the screen
    if (x < 0 || x >= WIDTH) {
        return;
    }

    // make sure we don't try to draw below 0
    if (__y < 0) {
        // __y is negative, this will subtract enough from __h to account for
        // __y being 0
        __h += __y;
        __y = 0;
    }

    // make sure we don't go past the height of the display
    if ((__y + __h) > HEIGHT) {
        __h = (HEIGHT - __y);
    }

    // if our height is now negative, punt
    if (__h <= 0) {
        return;
    }

    // this display doesn't need ints for coordinates, use local byte registers
    // for faster juggling
    register uint8_t y = __y;
    register uint8_t h = __h;

    // set up the pointer for fast movement through the buffer
    register uint8_t *pBuf = frames_[currentFrame_];
    // adjust the buffer pointer for the current row
    pBuf += ((y / 8) * SSD1306_LCDWIDTH);
    // and offset x columns in
    pBuf += x;

    // do the first partial byte, if necessary - this requires some masking
    register uint8_t mod = (y & 7);
    if (mod) {
        // mask off the high n bits we want to set
        mod = 8 - mod;

        // note - lookup table results in a nearly 10% performance improvement
        // in fill* functions
        // register uint8_t mask = ~(0xFF >> (mod));
        static uint8_t premask[8] = {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE};
        register uint8_t mask = premask[mod];

        // adjust the mask if we're not going to reach the end of this byte
        if (h < mod) {
            mask &= (0xFF >> (mod - h));
        }

        switch (color) {
            case WHITE:
                *pBuf |= mask;
                break;
            case BLACK:
                *pBuf &= ~mask;
                break;
            case INVERSE:
                *pBuf ^= mask;
                break;
        }

        // fast exit if we're done here!
        if (h < mod) {
            return;
        }

        h -= mod;

        pBuf += SSD1306_LCDWIDTH;
    }

    // write solid bytes while we can - effectively doing 8 rows at a time
    if (h >= 8) {
        if (color == INVERSE) {  // separate copy of the code so we don't impact
                                 // performance of the black/white write version
                                 // with an extra comparison per loop
            do {
                *pBuf = ~(*pBuf);

                // adjust the buffer forward 8 rows worth of data
                pBuf += SSD1306_LCDWIDTH;

                // adjust h & y (there's got to be a faster way for me to do
                // this, but this should still help a fair bit for now)
                h -= 8;
            } while (h >= 8);
        } else {
            // store a local value to work with
            register uint8_t val = (color == WHITE) ? 255 : 0;

            do {
                // write our value in
                *pBuf = val;

                // adjust the buffer forward 8 rows worth of data
                pBuf += SSD1306_LCDWIDTH;

                // adjust h & y (there's got to be a faster way for me to do
                // this, but this should still help a fair bit for now)
                h -= 8;
            } while (h >= 8);
        }
    }

    // now do the final partial byte, if necessary
    if (h) {
        mod = h & 7;
        // this time we want to mask the low bits of the byte, vs the high bits
        // we did above
        // register uint8_t mask = (1 << mod) - 1;
        // note - lookup table results in a nearly 10% performance improvement
        // in fill* functions
        static uint8_t postmask[8] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F};
        register uint8_t mask = postmask[mod];
        switch (color) {
            case WHITE:
                *pBuf |= mask;
                break;
            case BLACK:
                *pBuf &= ~mask;
                break;
            case INVERSE:
                *pBuf ^= mask;
                break;
        }
    }
}
