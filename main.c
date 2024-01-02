#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <termio.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>

#include "uinput.h"
#include "inih/ini.h"

typedef struct
{
    uint8_t address_l;
    uint8_t address_h;
    uint8_t power_key;
    uint8_t flags;
} avr_config;

typedef struct
{
    uint8_t pointer_address_h;
    uint8_t pointer_address_l;
    uint8_t b_flag;
    uint8_t remote_command_1h;
    uint8_t remote_command_1l;
    uint8_t remote_command_0h;
    uint8_t remote_command_0l;
} samsung_ir_command_t;

typedef union
{
    samsung_ir_command_t s;
    uint8_t data[0];
} samsung_ir_command_tu;

typedef struct
{
    char name[256];
    char serial[256];
    char uinput[256];
    uint32_t period;
    uint32_t delay;
    bool reset;         /* request reset at start */
    bool debug;         /* enable debug */
    bool power_confirm; /* power confirmation */
    bool check_avr_cfg;

    avr_config avr_cfg;
    uint16_t key_map[IR_MAX_KEYS_NUM];
    uint32_t ir_key_map[IR_MAX_KEYS_NUM];
} configuration;

typedef struct {
    uint8_t address_l;
    uint8_t address_h;
    uint8_t power_key;
    uint8_t flags;
    uint8_t hash;
} avr_scfg_t;


typedef union {
    avr_scfg_t s;
    uint8_t data[0];
} avr_cfg_t;

int usleep(unsigned int);

static void config_error(const char *section, const char *name, const char *value)
{
    printf("config_handler wrong %s item -> name[%s] value[%s]\n", section, name, value);
}

static int config_handler(void *ptr, const char *section, const char *name, const char *value)
{
    configuration *pconfig = (configuration*)ptr;

    if (0 == strcmp(section, "avr_cfg"))
    {
        if (0 == strcmp(name, "address_l"))
        {
            if (0 == sscanf(value, "%hhu", &(pconfig->avr_cfg.address_l)))
            {
                config_error(section, name, value);
                return 0;
            }
        }

        else if (0 == strcmp(name, "address_h"))
        {
            if (0 == sscanf(value, "%hhu", &(pconfig->avr_cfg.address_h)))
            {
                config_error(section, name, value);
                return 0;
            }
        }

        else if (0 == strcmp(name, "power_key"))
        {
            if (0 == sscanf(value, "%hhu", &(pconfig->avr_cfg.power_key)))
            {
                config_error(section, name, value);
                return 0;
            }
        }

        else if (0 == strcmp(name, "flags"))
        {
            if (0 == sscanf(value, "%hhu", &(pconfig->avr_cfg.flags)))
            {
                config_error(section, name, value);
                return 0;
            }
        }

    }
    else if (0 == strcmp(section, "config"))
    {
        if (0 == strcmp(name, "serial"))
        {
            strncpy(pconfig->serial, value, sizeof(pconfig->serial)-1);
        }
        else if (0 == strcmp(name, "name"))
        {
            strncpy(pconfig->name, value, sizeof(pconfig->name)-1);
        }
        else if (0 == strcmp(name, "uinput"))
        {
            strncpy(pconfig->uinput, value, sizeof(pconfig->uinput)-1);
        }
        else if (0 == strcmp(name, "period"))
        {
            if (0 == sscanf(value, "%u", &(pconfig->period)))
            {
                config_error(section, name, value);
                return 0;
            }
        }
        else if (0 == strcmp(name, "delay"))
        {
            if (0 == sscanf(value, "%u", &(pconfig->delay)))
            {
                config_error(section, name, value);
                return 0;
            }
        }
        else if (0 == strcmp(name, "reset"))
        {
            if (0 != strcmp(value, "1") && 0 != strcmp(value, "0"))
            {
                config_error(section, name, value);
                return 0;
            }
            pconfig->reset = value[0] == '1' ? true : false;
        }
        else if (0 == strcmp(name, "debug"))
        {
            if (0 != strcmp(value, "1") && 0 != strcmp(value, "0"))
            {
                config_error(section, name, value);
                return 0;
            }
            pconfig->debug = value[0] == '1' ? true : false;
        }
        else if (0 == strcmp(name, "power_confirm"))
        {
            if (0 != strcmp(value, "1") && 0 != strcmp(value, "0"))
            {
                config_error(section, name, value);
                return 0;
            }
            pconfig->power_confirm = value[0] == '1' ? true : false;
        }
        else if (0 == strcmp(name, "check_avr_cfg"))
        {
            if (0 != strcmp(value, "1") && 0 != strcmp(value, "0"))
            {
                config_error(section, name, value);
                return 0;
            }
            pconfig->check_avr_cfg = value[0] == '1' ? true : false;
        }
    }
    else if (0 == strcmp(section, "keymap"))
    {
        int32_t key_code = -1;
        sscanf(name, "%d", &key_code);
        if (key_code >= 0 && key_code < IR_MAX_KEYS_NUM)
        {
            if (0 == sscanf(value, "%hu", pconfig->key_map + key_code))
            {
                config_error(section, name, value);
                return 0;
            }
        }
        else
        {
            config_error(section, name, value);
            return 0;
        }
    }
    else if (0 == strcmp(section, "irkeymap"))
    {
        int32_t key_code = -1;
        sscanf(name, "%d", &key_code);
        if (key_code >= 0 && key_code < IR_MAX_KEYS_NUM)
        {
            if (0 == sscanf(value, "0x%X", &pconfig->ir_key_map[key_code]))
            {
                config_error(section, name, value);
                return 0;
            }
        }
        else
        {
            config_error(section, name, value);
            return 0;
        }
    }
    else
    {
        return 0;  /* unknown section/name, error */
    }
    return 1;
}

static int output_rts(int fd, int val)
{
    int retorno;
    int argumento = TIOCM_RTS;
    if(val)
    {
        retorno = ioctl(fd,TIOCMBIS,&argumento);
    }
    else
    {
        retorno = ioctl(fd,TIOCMBIC,&argumento);
    }
    return retorno;
}

static int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }

    // give time to, data to arrive in to the read buffer
    sleep(1);
    tcflush(fd, TCIOFLUSH);

    output_rts(fd, 0);
    return 0;
}

static int find_ir_key_map(const uint32_t ir_key_map[IR_MAX_KEYS_NUM], int received_command)
{
    int i;
    for (i=0; i < IR_MAX_KEYS_NUM; ++i)
    {
        if (ir_key_map[i] == (uint32_t)received_command){
            return i;
        }
    }
    return -1;
}

int main(int argc, char *argv[])
{
    ssize_t read_ret = 0;
    int fd = -1 ;
    int uinputfd = -1;
    char buf[36] = "";
    int rlen = 0;
    char ch = '\0';
    int32_t i = 0;
    char *config_file = "/etc/srcd.ini";
    unsigned int databuffer = 0;
    int key_code=0;
    char pressed[1] = "P";
    char released[1] = "R";
    fd_set set;
    struct timeval timeout;
    bool needs_release=false;
    int last_key_code;

    configuration config;

    memset(&config, 0x00, sizeof(config));
    for (i=0; i<IR_MAX_KEYS_NUM; ++i)
    {
        config.key_map[i] = (uint16_t)-1; // set all event code's as UNKNOWN
    }


    if (argc > 1)
    {
        if (0 == strcmp(argv[1], "--help"))
        {
            printf("Usage: \n");
            printf("\t%s [path to config file] \n", argv[0]);
            printf("\t\t path to config file - default: %s\n", config_file);
            exit(0);
        }
        config_file = argv[1];
    }

    if (ini_parse(config_file, config_handler, &config) < 0)
    {
        printf("Can't load \"%s\"\n", config_file);
        return 1;
    }

    if ('\0' == config.name[0])
    {
        strcpy(config.name, "ATtiny2313-uinput");
    }

    if ('\0' == config.uinput[0])
    {
        strcpy(config.uinput, "/dev/uinput");
    }

    if ('\0' == config.serial[0])
    {
        printf("Please set serial in the config file!\n");
        return 2;
    }

    fd = open(config.serial, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    while (fd < 0)
    {
        printf("Error opening %s: %s\n", config.serial, strerror(errno));
        sleep(1);
        continue;
    }

    uinputfd = setup_uinputfd(config.uinput, config.name, config.key_map, config.delay, config.period);

    /*baudrate B9600, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B115200); //B1200

    if (1 || uinputfd > -1)
    {
        while (1) 
        {
            FD_ZERO(&set);
            FD_SET(fd, &set);

            timeout.tv_sec = 0;
            timeout.tv_usec = 500000;
            int ready = select(fd + 1, &set, NULL, NULL, &timeout);

            if (ready == -1) {
                perror("Error from select");
                break;
            } else if (ready == 0) {
                printf("Timeout occurred, no data received.\n");
                if (needs_release){
                    printf("Released %d.\n", last_key_code);
                    process_key_message(uinputfd, released, last_key_code);
                    needs_release=false;
                }
            } else {
                if (FD_ISSET(fd, &set)) {
                    read_ret = read(fd, &ch, 1);
                    if (read_ret == -1)
                    {
                        if (errno == EINTR)         /* Interrupted --> restart read() */
                            continue;
                        else
                            return -1;              /* Some other error */
                    }
                    else if (read_ret == 0)         /* EOF */
                    {
                        printf("EOF\n");
                        break;
                    }
                    else
                    {
                        if (ch == 'S')
                        {
                            printf("Found S:\n");
                            samsung_ir_command_tu samsung_ir_command;
                            size_t rsize = 0;
                            while (rsize < sizeof(samsung_ir_command))
                            {
                                rsize += read(fd, samsung_ir_command.data + rsize, sizeof(samsung_ir_command) - rsize);
                                usleep(100);
                            }
                            if (rsize > 0)
                            {
                                printf("Microchip IR Command received:\n");
                                int i=0;
                                printf("\tBuffer: \n");
                                for(i=0; i<rsize; i++){
                                    printf("\tdata[%d]: %hhX\n", i, samsung_ir_command.data[i]);
                                }
                                printf("\n");
                                printf("\tpointer_address_h: %hhX\n", samsung_ir_command.s.pointer_address_h);
                                printf("\tpointer_address_l: %hhX\n", samsung_ir_command.s.pointer_address_l);
                                printf("\tb_flag: %hhX\n", samsung_ir_command.s.b_flag);
                                printf("\tremote_command_1h: %hhX\n", samsung_ir_command.s.remote_command_1h);
                                printf("\tremote_command_1l: %hhX\n", samsung_ir_command.s.remote_command_1l);
                                printf("\tremote_command_0h: %hhX\n", samsung_ir_command.s.remote_command_0h);
                                printf("\tremote_command_1l: %hhX\n", samsung_ir_command.s.remote_command_0l);
                                databuffer = samsung_ir_command.s.remote_command_1h << 24 | samsung_ir_command.s.remote_command_1l << 16 | samsung_ir_command.s.remote_command_0h << 8 | samsung_ir_command.s.remote_command_0l;
                                printf("\tdatabuffer: 0x%X\n", databuffer);
                                key_code = find_ir_key_map(config.ir_key_map,databuffer);
                                if (key_code >= 0){
                                    printf("\tkey_code found = %d\n", key_code);
                                    process_key_message(uinputfd, pressed, key_code);
                                    //needs_release = true;
                                    //last_key_code = key_code;
                                    usleep(500);
                                    process_key_message(uinputfd, released, key_code);
                                } else {
                                    printf("\tError retrieving key_code\n");
                                }
                             } else {
                                printf("rsize=0\n");
                             }
                         }
                    }
                }
            }
        }
    }
    else
    {
        printf("setup_uinputfd failed\n");
    }
    return 0;
}