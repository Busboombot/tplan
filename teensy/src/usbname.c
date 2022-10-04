// From: https://gist.githubusercontent.com/j0hnm4r5/2a063b4ff14418d63fd25d99efd1ae71/raw/fed85199fdc114006c94a89657fae08da6a09866/name.c
// https://medium.com/@j0hnm4r5/changing-teensy-serial-name-and-port-name-2ca76552d26d

#include <usb_names.h>

#define MANUFACTURER_NAME    {'B','u','s','b','o','t'}
#define MANUFACTURER_NAME_LEN    6
#define PRODUCT_NAME        {'S','m','a','l','l','s','t','e','p'}
#define PRODUCT_NAME_LEN    9

#ifndef SERIAL_NUMBER
#define SERIAL_NUMBER  {'_','B', 'u', 's', 'b', 'o', 't', '-', 's', 's', '0', '0', '1' }
#define SERIAL_NUMBER_LEN 13
#endif


struct usb_string_descriptor_struct usb_string_manufacturer_name = {
        2 + MANUFACTURER_NAME_LEN * 2,
        3,
        MANUFACTURER_NAME // Maybe {u"Busbot"} will also work?
};
struct usb_string_descriptor_struct usb_string_product_name = {
        2 + PRODUCT_NAME_LEN * 2,
        3,
        PRODUCT_NAME
};
struct usb_string_descriptor_struct usb_string_serial_number = {
        2 + SERIAL_NUMBER_LEN * 2,
        3,
        SERIAL_NUMBER};