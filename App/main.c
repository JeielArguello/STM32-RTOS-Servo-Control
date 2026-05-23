// Simple USB HID console to send direction commands to a microcontroller
// Uses hidapi (https://github.com/libusb/hidapi)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#include <stdint.h>
#include <pthread.h>
#include <unistd.h>
#include <hidapi/hidapi.h>

// Default Vendor ID and Product ID (change to match your device)
#define DEFAULT_VID 0x0483
#define DEFAULT_PID 0x5750

typedef struct __attribute__((packed)) {
    uint8_t reportID;
    int32_t position;
    uint32_t velocity;
    uint8_t status_flags;
} HID_Report_t;

static volatile int g_running = 1;

static void *reader_thread(void *arg)
{
    hid_device *handle = (hid_device *)arg;
    unsigned char report[64];
    printf("Reader thread started, waiting for data...\n");

    while (g_running) {
        int bytes_read = hid_read_timeout(handle, report, sizeof(report), 100);
        if (bytes_read > 0) {
            printf("\nRX HID (%d bytes):", bytes_read);
            for (int i = 0; i < bytes_read; ++i) {
                printf(" %02X", report[i]);
            }
            printf("\n");

            if (bytes_read >= (int)sizeof(HID_Report_t)) {
                HID_Report_t telemetry;
                memcpy(&telemetry, report, sizeof(telemetry));
                printf("Telemetry -> reportID: 0x%02X, position: %ld, velocity: %lu, status: 0x%02X\n",
                       telemetry.reportID,
                       (long)telemetry.position,
                       (unsigned long)telemetry.velocity,
                       telemetry.status_flags);
            }

            fflush(stdout);
        }
    }

    return NULL;
}

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

    pthread_t rx_thread;
    if (pthread_create(&rx_thread, NULL, reader_thread, handle) != 0) {
        fprintf(stderr, "Failed to start HID reader thread\n");
        hid_close(handle);
        hid_exit();
        return 3;
    }
    
    for (;;) {
        char line[128];
        printf("Enter direction: L (left), R (right), S (stop), Q (quit)\n");
        if (!fgets(line, sizeof(line), stdin)) break;
        // trim
        char cmd = 0;
        for (int i = 0; line[i]; ++i) {
            if (line[i] == 'L' || line[i] == 'l') { cmd = 'L'; break; }
            if (line[i] == 'R' || line[i] == 'r') { cmd = 'R'; break; }
            if (line[i] == 'S' || line[i] == 's') { cmd = 'S'; break; }
            if (line[i] == 'Q' || line[i] == 'q') { cmd = 'Q'; break; }
        }

        // Also accept a numeric degree input (e.g. "45" or "-30").
        char *endptr = NULL;
        long deg = strtol(line, &endptr, 10);
        if (!cmd) {
            if (endptr != line) {
                // Got a numeric degree
                cmd = 'G'; // G = grado (numeric)
            } else {
                continue;
            }
        }

        if (cmd == 'Q') break;

        // Prepare a simple HID output report. First byte is report ID (0 if unused)
        HID_Report_t report;

        report.reportID = 0x02; // report id
        report.position = 0;
        report.velocity = 0;
        report.status_flags = 0;
        if (cmd == 'L') {
            report.position = -100; // example value for left
            report.velocity = 50;    // example velocity
        } else if (cmd == 'R') {
            report.position = 100;  // example value for right
            report.velocity = 50;   // example velocity
        } else if (cmd == 'S') {
            report.position = 0;    // stop
            report.velocity = 0;
        } else if (cmd == 'G') {
            // Use the numeric degree entered by the user as position
            report.position = (int32_t)deg;
            report.velocity = 0;
            printf("Using numeric degree: %ld -> position set to %d\n", deg, (int)report.position);
        }
        printf("Sending command %c to device...\n", cmd);
        printf("Report data size: %zu bytes\n", sizeof(report));
        // Write report (report length is 65 for one-byte report id + 64 data)
        res = hid_write(handle, (unsigned char*) &report, sizeof(report));
        printf("hid_write returned: %d\n", res);
        if (res < 0) {
            fprintf(stderr, "hid_write failed\n");
            break;
        }

        printf("Sent command %c (bytes written: %d)\n", cmd, res);
    }

    g_running = 0;
    pthread_join(rx_thread, NULL);

    hid_close(handle);
    hid_exit();
    printf("Exiting console\n");
    return 0;
}
