// Simple USB HID console to send direction commands to a microcontroller
// Uses hidapi (https://github.com/libusb/hidapi)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#include <stdint.h>
#include <hidapi/hidapi.h>

// Default Vendor ID and Product ID (change to match your device)
#define DEFAULT_VID 1155
#define DEFAULT_PID 22352

int main(int argc, char **argv)
{
    unsigned short vid = DEFAULT_VID;
    unsigned short pid = DEFAULT_PID;
    int res;

    if (argc >= 3) {
        vid = (unsigned short)strtol(argv[1], NULL, 0);
        pid = (unsigned short)strtol(argv[2], NULL, 0);
    }

    if (hid_init() != 0) {
        fprintf(stderr, "Failed to initialize hidapi\n");
        return 1;
    }

    hid_device *handle = hid_open(vid, pid, NULL);
    if (!handle) {
        fprintf(stderr, "Unable to open device %d:%d\n", vid, pid);
        hid_exit();
        return 2;
    }

    printf("USB HID console connected to %04hx:%04hx\n", vid, pid);
    printf("Enter direction: L (left), R (right), S (stop), Q (quit)\n");

    for (;;) {
        char line[128];
        if (!fgets(line, sizeof(line), stdin)) break;
        // trim
        char cmd = 0;
        for (int i = 0; line[i]; ++i) {
            if (line[i] == 'L' || line[i] == 'l') { cmd = 'L'; break; }
            if (line[i] == 'R' || line[i] == 'r') { cmd = 'R'; break; }
            if (line[i] == 'S' || line[i] == 's') { cmd = 'S'; break; }
            if (line[i] == 'Q' || line[i] == 'q') { cmd = 'Q'; break; }
        }
        if (!cmd) continue;
        if (cmd == 'Q') break;

        // Prepare a simple HID output report. First byte is report ID (0 if unused)
        unsigned char report[65];
        memset(report, 0, sizeof(report));
        report[0] = 0x00; // report id
        report[1] = (unsigned char)cmd; // command: 'L','R','S'

        // Write report (report length is 65 for one-byte report id + 64 data)
        res = hid_write(handle, report, sizeof(report));
        if (res < 0) {
            fprintf(stderr, "hid_write failed\n");
            break;
        }

        printf("Sent command %c (bytes written: %d)\n", cmd, res);
    }

    hid_close(handle);
    hid_exit();
    printf("Exiting console\n");
    return 0;
}
