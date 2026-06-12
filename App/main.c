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
#include <time.h>

// Default Vendor ID and Product ID (change to match your device)
#define DEFAULT_VID 0x0483
#define DEFAULT_PID 0x5750

// ANSI color and control codes
#define ANSI_CLEAR_SCREEN "\033[2J"
#define ANSI_HOME "\033[H"
#define ANSI_CURSOR_OFF "\033[?25l"
#define ANSI_CURSOR_ON "\033[?25h"
#define ANSI_BOLD "\033[1m"
#define ANSI_GREEN "\033[32m"
#define ANSI_YELLOW "\033[33m"
#define ANSI_RESET "\033[0m"

typedef struct __attribute__((packed)) {
    uint8_t reportID;
    int32_t position;
    int32_t velocity;
    int16_t rotations;
    uint8_t status_flags;
} HID_Report_t;



// Shared motor status structure
typedef struct {
    int32_t position;
    int32_t velocity;
    int16_t rotations;
    uint8_t status_flags;
    time_t last_update;
    int updated;
} MotorStatus_t;

static volatile int g_running = 1;
static MotorStatus_t g_motor_status = {0};
static pthread_mutex_t g_status_mutex = PTHREAD_MUTEX_INITIALIZER;

static void *reader_thread(void *arg)
{
    hid_device *handle = (hid_device *)arg;
    unsigned char report[64];

    while (g_running) {
        int bytes_read = hid_read_timeout(handle, report, sizeof(report), 100);
        if (bytes_read > 0) {
            if (bytes_read >= (int)sizeof(HID_Report_t)) {
                HID_Report_t telemetry;
                memcpy(&telemetry, report, sizeof(telemetry));

                pthread_mutex_lock(&g_status_mutex);
                g_motor_status.position = telemetry.position;
                g_motor_status.velocity = telemetry.velocity;
                g_motor_status.status_flags = telemetry.status_flags;
                g_motor_status.rotations = telemetry.rotations;
                g_motor_status.last_update = time(NULL);
                g_motor_status.updated = 1;
                pthread_mutex_unlock(&g_status_mutex);
            }
            else {
                fprintf(stderr, "Received incomplete report (%d bytes)\n", bytes_read);
            }
        }else if (bytes_read < 0) {
            fprintf(stderr, "Error reading from device: %ls\n", hid_error(handle));
            g_running = 0;
        }
    }

    return NULL;
}

// Display update thread - continuously redraw status panel
static void *display_thread(void *arg)
{
    (void)arg;
    
    while (g_running) {
        pthread_mutex_lock(&g_status_mutex);
        
        // Save cursor position
        printf("\033[s");
        
        // Move to line 0, column 0 and update only the status area
        printf("\033[H");
        printf("\033[2K");  // Clear line
        printf(ANSI_BOLD ANSI_GREEN "╔════════════════════════════════════╗" ANSI_RESET "\n");
        printf("\033[2K");
        printf("║    MOTOR STATUS MONITOR              ║\n");
        printf("\033[2K");
        printf("╠════════════════════════════════════╣\n");
        printf("\033[2K");
        printf(ANSI_YELLOW "║ Position:    %8ld°              ║" ANSI_RESET "\n", 
               (long)g_motor_status.position);
        printf("\033[2K");
        printf(ANSI_YELLOW "║ Velocity:    %8ld °/s            ║" ANSI_RESET "\n", 
            (long)g_motor_status.velocity);
        printf("\033[2K");
        printf(ANSI_YELLOW "║ Rotations:   %d                    ║" ANSI_RESET "\n", 
            g_motor_status.rotations);
        printf("\033[2K");
        printf(ANSI_YELLOW "║ Status:      0x%02X                    ║" ANSI_RESET "\n", 
            g_motor_status.status_flags);
        printf("\033[2K");
        printf(ANSI_YELLOW "║ Updated:     %s          ║" ANSI_RESET "\n",
               g_motor_status.updated ? "YES" : "NO ");
        printf("\033[2K");
        printf(ANSI_BOLD ANSI_GREEN "╚════════════════════════════════════╝" ANSI_RESET "\n");
        
        // Restore cursor position
        printf("\033[u");
        
        pthread_mutex_unlock(&g_status_mutex);
        fflush(stdout);
        
        // Update every 200ms for smooth real-time display
        usleep(20000);
    }

    return NULL;
}

// Display command menu (fixed position at line 8)
static void display_menu(void)
{
    printf("\n");
    printf(ANSI_BOLD "╔════════════════════════════════════╗\n");
    printf("║       COMMAND MENU                  ║\n");
    printf("╠════════════════════════════════════╣\n");
    printf(ANSI_RESET);
    printf("║ Enter position (degrees): -360 to 360\n");
    printf("║ Enter 'v XXX' to set velocity       ║\n");
    printf("║ Enter 'stop' to halt motor          ║\n");
    printf("║ Enter 'q' to quit                   ║\n");
    printf(ANSI_BOLD ANSI_GREEN "╚════════════════════════════════════╝" ANSI_RESET "\n");
    printf("\n");
    fflush(stdout);
}

// Parse user input
static int parse_input(const char *line, int32_t *position, int32_t *velocity)
{
    // Check for stop command
    if (strstr(line, "stop") || strstr(line, "STOP")) {
        *position = 0;
        *velocity = 0;
        return 1;  // Stop command
    }

    // Check for velocity command (v XXX)
    if (line[0] == 'v' || line[0] == 'V') {
        *velocity = (int32_t)strtol(line + 1, NULL, 10);
        return 2;  // Velocity set command
    }

    // Try to parse as position
    char *endptr = NULL;
    long pos = strtol(line, &endptr, 10);
    if (endptr != line && pos >= -900 && pos <= 900) {
        *position = (int32_t)pos;
        return 0;  // Position command
    }

    return -1;  // Invalid input
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

    // Clear screen and setup display
    printf(ANSI_CLEAR_SCREEN ANSI_HOME);

    printf(ANSI_BOLD ANSI_GREEN "╔════════════════════════════════════╗\n");
    printf("║ USB HID Motor Controller - Connected║\n");
    printf("╠════════════════════════════════════╣\n");
    printf(ANSI_RESET);
    printf("║ Device: %04hx:%04hx\n", vid, pid);
    printf(ANSI_BOLD ANSI_GREEN "╚════════════════════════════════════╝" ANSI_RESET "\n\n");

    pthread_t rx_thread;
    if (pthread_create(&rx_thread, NULL, reader_thread, handle) != 0) {
        fprintf(stderr, "Failed to start HID reader thread\n");
        hid_close(handle);
        hid_exit();
        return 3;
    }

    pthread_t disp_thread;
    if (pthread_create(&disp_thread, NULL, display_thread, NULL) != 0) {
        fprintf(stderr, "Failed to start display thread\n");
        hid_close(handle);
        hid_exit();
        return 4;
    }

    int32_t target_position = 0;
    int32_t target_velocity = 0;

    display_menu();

    for (;;) {
        printf(ANSI_BOLD "Enter command: " ANSI_RESET);
        fflush(stdout);

        char line[128];
        if (!fgets(line, sizeof(line), stdin)) break;

        // Check for quit
        if (line[0] == 'q' || line[0] == 'Q') break;

        int32_t pos = target_position;
        int32_t vel = target_velocity;
        int cmd_type = parse_input(line, &pos, &vel);

        if (cmd_type < 0) {
            printf(ANSI_YELLOW "Invalid input. Try again.\n" ANSI_RESET);
            sleep(1);
            continue;
        }

        // Update targets based on command
        if (cmd_type == 0) {
            // Position command
            target_position = pos;
            printf(ANSI_GREEN "→ Position set to %ld°\n" ANSI_RESET, (long)target_position);
        } else if (cmd_type == 1) {
            // Stop command
            target_position = 0;
            target_velocity = 0;
            printf(ANSI_GREEN "→ Motor stopped.\n" ANSI_RESET);
        } else if (cmd_type == 2) {
            // Velocity command
            target_velocity = vel;
            printf(ANSI_GREEN "→ Velocity set to %ld °/s\n" ANSI_RESET, (long)target_velocity);
        }

        // Send command
        HID_Report_t report;
        report.reportID = 0x02;
        report.position = target_position;
        report.velocity = target_velocity;
        report.rotations = 0;  
        report.status_flags = 0;
        
        res = hid_write(handle, (unsigned char *)&report, sizeof(report));
        
        if (res < 0) {
            fprintf(stderr, ANSI_YELLOW "✗ hid_write failed\n" ANSI_RESET);
            break;
        }

        printf(ANSI_GREEN "✓ Command sent\n" ANSI_RESET);
    }

    g_running = 0;
    pthread_join(rx_thread, NULL);
    pthread_join(disp_thread, NULL);

    printf(ANSI_CURSOR_ON);
    hid_close(handle);
    hid_exit();
    printf("\nConsole closed.\n");
    return 0;
}
