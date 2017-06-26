#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#define MAP_SIZE 0x1000
#define GPIO3_DR 0x20A4000
#define GPIO3_GDIR 0x20A4004

int g_fmem;
int g_map_vaddr = 0;
int g_map_paddr = 0;
uint32_t gpio3_dr = 0;
uint32_t gpio3_gdir = 0;

char *msg[] = { "Board Type:Display\n\r", "Revision:???\n\r"};

int open_mem_file(void)
{
        if (!g_fmem)
                g_fmem = open("/dev/mem", O_RDWR | O_SYNC, 0);

        if (!g_fmem)
                printf("Can't open file /dev/mem\n");
        return 0;
}

int map_address(int address)
{
        // mapped to the same address?
        if ((address & (~(MAP_SIZE - 1))) == g_map_paddr)
                return 0;

        // clear previous mapping
        if (g_map_vaddr)
                munmap((void *)g_map_vaddr, MAP_SIZE);

        // create new mapping
        g_map_vaddr =
                (int)mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
                        g_fmem, address & (~(MAP_SIZE  - 1)));
        // align to boundary
        g_map_paddr = address & (~(MAP_SIZE - 1));

        return 0;
}

int readm(int address, int width)
{
        uint8_t *addr8;
        uint16_t *addr16;
        uint32_t *addr32;

        open_mem_file();
        map_address(address);

        addr32 = (uint32_t *) (g_map_vaddr + (address & (MAP_SIZE - 1)));
        addr16 = (uint16_t *) addr32;
        addr8 = (uint8_t *) addr16;

        switch (width) {
        case 1:
                return *addr8;
        case 2:
                return *addr16;
        case 4:
        default:
                return *addr32;
        }
}

int writem(int address, int width, int value)
{
        uint8_t *addr8;
        uint16_t *addr16;
        uint32_t *addr32;

        open_mem_file();
        map_address(address);

        addr32 = (uint32_t *) (g_map_vaddr + (address & (MAP_SIZE - 1)));
        addr16 = (uint16_t *) addr32;
        addr8 = (uint8_t *) addr16;

        switch (width) {
        case 1:
                *addr8 = (uint8_t)value;
                break;
        case 2:
                *addr16 = (uint16_t)value;
                break;
        case 4:
        default:
                *addr32 = (uint32_t)value;
                break;
        }
        return 0;
}

int set_RTS(int val)
{
        if (((gpio3_gdir >> 20) & 0x1) != 1) {
                gpio3_gdir = readm(GPIO3_GDIR, 4) | (1 << 20);
                writem(GPIO3_GDIR, 4, gpio3_gdir);
        }

        gpio3_dr = readm(GPIO3_DR, 4);

        if (((gpio3_dr >> 20) & 0x1)!= val) {
                gpio3_dr ^= (1 << 20);
                writem(GPIO3_DR, 4, gpio3_dr);
        }
}

int main(int argc, char *argv[])
{
        struct termios newTIO;
        int fd;
        int num_bytes_read;
        unsigned char byte[256];

        fd = open("/dev/ttymxc0", O_RDWR | O_NOCTTY);
        if(fd == -1)
                printf("can't open /dev/ttymxc0\n");

        bzero(&newTIO, sizeof(newTIO));
        cfsetispeed(&newTIO, B115200);
        cfsetospeed(&newTIO, B115200);

        newTIO.c_cflag |= (CLOCAL | CREAD);
        newTIO.c_cflag &= ~PARENB;
        newTIO.c_cflag &= ~CSIZE;
        newTIO.c_cflag |= CS8;
        newTIO.c_cflag &= ~CSTOPB;

        newTIO.c_oflag = 0;
        newTIO.c_lflag = 0;
        newTIO.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        newTIO.c_cc[VTIME] = 0;
        newTIO.c_cc[VMIN] = 1;

        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, &newTIO);

        set_RTS(0);
        while (1) {
                num_bytes_read = read(fd, byte, 256);
                if ((num_bytes_read == 1) && (byte[0] == 't')) {
                set_RTS(1);
                write(fd, msg[0], strlen(msg[0]));
                write(fd, msg[1], strlen(msg[1]));
                tcdrain(fd);
                set_RTS(0);
                break;
                }
        }

        close(fd);

    return 0;
}
