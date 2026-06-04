#include <LiquidCrystal_I2C.h>
#include <avr/pgmspace.h>

class SmoothBarGraph
{
public:

    SmoothBarGraph(
        LiquidCrystal_I2C& lcd,
        uint8_t col,
        uint8_t row,
        uint8_t width)
        :
        _lcd(lcd),
        _col(col),
        _row(row),
        _width(width)
    {
    }

    void begin()
    {
        uint8_t buffer[8];

        for (uint8_t c = 0; c < 6; c++)
        {
            memcpy_P(buffer, barChars[c], 8);
            _lcd.createChar(c, buffer);
        }
    }

    void ShowBargraph(float value)
    {
        if (value < 0.0f) value = 0.0f;
        if (value > 1.0f) value = 1.0f;

        // Convert directly to "subpixels"
        uint16_t pixels = (uint16_t)(value * (_width * 5) + 0.5f);

        _lcd.setCursor(_col, _row);

        while (pixels >= 5)
        {
            _lcd.write((uint8_t)5);
            pixels -= 5;
        }

        if (pixels > 0)
        {
            _lcd.write((uint8_t)pixels);
        }

        // Remaining empty cells
        uint8_t used =
            (uint8_t)((value * (_width * 5) + 4) / 5);

        while (used < _width)
        {
            _lcd.write((uint8_t)0);
            used++;
        }
    }

private:

    LiquidCrystal_I2C& _lcd;

    const uint8_t _col;
    const uint8_t _row;
    const uint8_t _width;

    // Stored in FLASH not RAM
    static const uint8_t barChars[6][8] PROGMEM;

  
};

const uint8_t SmoothBarGraph::barChars[6][8] PROGMEM =
{
    {0,0,0,0,0,0,0,0},
    {16,16,16,16,16,16,16,16},
    {24,24,24,24,24,24,24,24},
    {28,28,28,28,28,28,28,28},
    {30,30,30,30,30,30,30,30},
    {31,31,31,31,31,31,31,31}
};