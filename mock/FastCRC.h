#pragma once


class FastCRC8
{
public:
    FastCRC8();
    uint8_t smbus(const uint8_t *data, const uint16_t datalen){return 0;}		// Alias CRC-8
    uint8_t maxim(const uint8_t *data, const uint16_t datalen){return 0;}		// Equivalent to _crc_ibutton_update() in crc16.h from avr_libc

    uint8_t smbus_upd(const uint8_t *data, uint16_t datalen){return 0;}		// Call for subsequent calculations with previous seed.
    uint8_t maxim_upd(const uint8_t *data, uint16_t datalen){return 0;}			// Call for subsequent calculations with previous seed.
#if !CRC_SW
    uint8_t generic(const uint8_t polyom, const uint8_t seed, const uint32_t flags, const uint8_t *data, const uint16_t datalen){return 0;} //Not available in non-hw-variant (not T3.x)
#endif
private:
#if CRC_SW
    uint8_t seed;
#else
    uint8_t update(const uint8_t *data, const uint16_t datalen){return 0;}
#endif
};