#ifndef __SERIAL_UINPUT__
#define __SERIAL_UINPUT__

#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>

#define IR_MAX_KEYS_NUM 256

int setup_uinputfd(const char *uinput_path, const char *event_dev_name, \
                   const uint16_t key_map[IR_MAX_KEYS_NUM], unsigned delay, unsigned period);
bool process_key_message(int uinputfd, char *message);

#endif // __SERIAL_UINPUT__
