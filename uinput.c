#include <getopt.h>
#include <limits.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>

#include <sys/reboot.h>

#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>

#include <linux/input.h>
#include <linux/uinput.h>

#include <time.h>

#include "uinput.h"

static uint16_t g_key_map[IR_MAX_KEYS_NUM];

static bool write_event(int fd, unsigned type, uint16_t code, int val)
{
    struct input_event event;

    memset(&event, 0, sizeof(event));
    event.type = type;
    event.code = code;
    event.value = val;
    return write(fd, &event, sizeof(event)) == sizeof(event);
}
 
static int register_keys(int fd)
{
    int i;
    for (i=0; i < IR_MAX_KEYS_NUM; ++i)
    {
        if (g_key_map[i] != KEY_UNKNOWN && \
            g_key_map[i] >= KEY_RESERVED &&  \
            g_key_map[i] <= KEY_MAX)
        {
            if (ioctl(fd, UI_SET_KEYBIT, g_key_map[i]) != 0)
            {
                printf("UI_SET_KEYBIT failed for key [%d]\n", (int32_t)g_key_map[i]);
                return 1;
            }
        }
    }
    return 0;
}

static void set_kernel_repeat(int uinputfd, unsigned delay, unsigned period)
{
    if (delay != 0)
    {
        printf("Setting kernel repeat delay to %d\n", delay);
        if (!write_event(uinputfd, EV_REP, REP_DELAY, delay))
            printf("Cannot set kernel repeat delay\n");
    }

    if (period != 0)
    {
        printf("Setting kernel repeat period to %d\n", period);
        if (!write_event(uinputfd, EV_REP, REP_PERIOD, period))
            printf("Cannot set kernel repeat period\n");
    }
}

static uint16_t key2event(uint8_t key_code)
{
    return g_key_map[key_code];
}

static int set_key_map(const uint16_t key_map[IR_MAX_KEYS_NUM])
{
    int i;
    for (i=0; i < IR_MAX_KEYS_NUM; ++i)
    {
        g_key_map[i] = key_map[i] == (uint16_t)-1 ? KEY_UNKNOWN : key_map[i];
    }
    return 0;
}

int setup_uinputfd(const char *uinput_path, const char *event_dev_name, const uint16_t key_map[IR_MAX_KEYS_NUM], unsigned delay, unsigned period)
{
    int fd = -1;
    struct uinput_user_dev dev;
    int ok = 0;

    memset(&dev, 0x00, sizeof(dev));

    fd = open(uinput_path, O_RDWR);
    if (fd == -1)
    {
        printf("Cannot open uinput device: %s\n", uinput_path);
        return -1;
    }

    memset(&dev, 0, sizeof(dev));
    strncpy(dev.name, event_dev_name, sizeof(dev.name) - 1);
    ok = write(fd, &dev, sizeof(dev)) == sizeof(dev)
        && ioctl(fd, UI_SET_EVBIT, EV_KEY) == 0
        && ioctl(fd, UI_SET_EVBIT, EV_REP) == 0
        && set_key_map(key_map) == 0
        && register_keys(fd) == 0
        && ioctl(fd, UI_DEV_CREATE) == 0;

    if (ok)
    {
        set_kernel_repeat(fd, delay, period);
        return fd;
    }

    printf("could not setup uinput\n");
    close(fd);
    return -1;
}

bool process_key_message(int uinputfd, char *message, int key_code)
{
    static struct timespec s_timestamp;

    bool bret = false;
    //int key_code = -1;
    if (message)
    {
        if (message[0] == 'P' || message[0] == 'R')
        {
            if (key_code >= 0 && key_code <= 255)
            {

                uint16_t event_code = key2event((uint8_t)key_code);

                bret = write_event(uinputfd, EV_KEY, event_code, message[0] == 'P' ? 1 : 0);
                if (bret)
                {
                    bret = bret && write_event(uinputfd, EV_SYN, SYN_REPORT, 0);
                }
                else
                {
                    printf("write_event EV_KEY failed\n");
                }
            }
            else
            {
                printf("Wrong key code key_code[%d] message[%s]\n", key_code, message);
            }
        }
    }
    else
    {
        printf("Wrong message message[%p]\n", message);
    }
    return bret;
}


