#ifndef MP3_Module_h
#define MP3_Module_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "SoftwareSerial.h"

// playback source settings
#define PLAYBACK_USB		0x01
#define PLAYBACK_SD			0x02
#define PLAYBACK_FLASH		0x03

// cycle mode settings
#define WHOLE_DISC_LOOP		0x00 // Play the whole disc in sequence, and play it in a loop after playing
#define SINGLE_LOOP			0x01 // Play the current track in a loop
#define SINGLE_STOP			0x02 // Stop once after playing the current track (DEFAULT)
#define ALL_RANDOM			0x03 // Randomly play the tracks in the drive letter
#define DIRECTORY_LOOP		0x04 // Play the tracks in the current folder in order, and loop after playing, the directory does not contain sub-directories
#define DIRECTORY_RANDOM	0x05 // Play randomly in the current directory, the directory does not contain subdirectories
#define DIRECTORY_ORDER		0x06 // Play the tracks in the current folder in order, stop after playing, the directory does not contain subdirectories
#define PLAY_IN_SEQUENCE	0x07 // Play the entire track in sequence and stop after 

// EQ settings
#define NORMAL				0x00 // Default
#define POP					0x01
#define ROCK				0x02
#define JAZZ				0x03
#define CLASSIC				0x04

// playback channel settings
#define CHANNEL_MP3			0x00 // Default
#define CHANNEL_AUX			0x01
#define CHANNEL_MP3_AUX		0x02

class MP3{
public:
    void begin();                               // Begin function
    void play();                                // Play
    void pause();                               // Pause
    void mp3_stop();                                // Stop track
    void previous();                            // Previous track
    void next();                                // Next track
    void play_track(uint16_t file);             // Play selected track
    void switch_playback_source(uint8_t src);   // Set playback source (PLAYBACK_USB, PLAYBACK_SD, PLAYBACK_FLASH)
    void volume(uint8_t vol);                   // Set volume (0 - 30)
    void increase_volume();                     // Increase volume by 1
    void decrease_volume();                     // Decrease volume by 1
    void cycle_mode(uint8_t mode);              // Set cycle mode (WHOLE_DISC_LOOP, SINGLE_LOOP, SINGLE_STOP, ALL_RANDOM, DIRECTORY_LOOP, DIRECTORY_RANDOM, DIRECTORY_ORDER, PLAY_IN_SEQUENCE)
    void loop(uint16_t num);                    // Set number of loops
    void eq(uint8_t eq);                        // Set EQ (NORMAL, POP, ROCK, JAZZ, CLASSIC)
    void playback_channel(uint8_t channel);     // Set playback channel (CHANNEL_MP3, CHANNEL_AUX, CHANNEL_MP3_AUX)
    void total_track_time();                    // Get total track time 
    void enable_track_time_report();            // Enable report current song time every second
    void disable_track_time_report();           // Disable report current song time every second

    int get_status();                           // Get current status (0 - STOP, 1 - PLAY, 2 - PAUSE)
    int get_selected_source();                  // Get playback source (0 - USB, 1 - SD, 3 - FLASH)
    int get_playing_source();                   // Get playback source (0 - USB, 1 - SD, 3 - FLASH)
    int get_total_tracks();                     // Get total number of tracks
    int get_current_track();                    // Get current track number

private:
    void send_cmd(uint8_t *cmd);               // Send command function

    enum {CMD_BUF_LEN = 10};
    uint8_t cmd_buf[CMD_BUF_LEN];
    uint8_t reply_buf[CMD_BUF_LEN];
};

#endif

SoftwareSerial COM_SOFT(17, 16); // OR 15 14 first parameter is rx second tx


void MP3::begin()
{
    COM_SOFT.begin(9600);
    delay(10);
}

void MP3::send_cmd(uint8_t *cmd)
{
    uint8_t i;
    uint8_t length;
    uint8_t checksum;

    length = cmd[2] + 4;
    if (length > 10) {
        return;
    }

    cmd[0] = 0xAA;
    for (i = 0; i < length - 1; i++) {
        checksum += cmd[i];
    }
    cmd[length - 1] = checksum;
    
    for (i = 0; i < length; i++){
        COM_SOFT.write(cmd[i]);
    }
}

int MP3::get_status()
{
    while(COM_SOFT.available()) COM_SOFT.read();

    uint8_t i = 0;
    uint8_t length = 0;
    uint8_t checksum = 0;
    
    cmd_buf[1] = 0x01;
    cmd_buf[2] = 0x00;
    send_cmd(cmd_buf);
    
    if (COM_SOFT.available()) {
        while (COM_SOFT.available()) {
            char c = COM_SOFT.read();
            reply_buf[length++] = c;
            delay(2);
        }
    }

    for (i = 0; i < length - 1; i++) {
        checksum += reply_buf[i];
    }
    if (reply_buf[length - 1] == checksum) {
        return reply_buf[3];
    }
    
    return -1;
}

void MP3::play()
{
    cmd_buf[1] = 0x02;
    cmd_buf[2] = 0x00;
    send_cmd(cmd_buf);
}

void MP3::pause()
{
    cmd_buf[1] = 0x03;
    cmd_buf[2] = 0x00;
    send_cmd(cmd_buf);
}

void MP3::mp3_stop()
{
    cmd_buf[1]=0x04;
    cmd_buf[2]=0x00;
    send_cmd(cmd_buf);
}

void MP3::previous()
{
    cmd_buf[1] = 0x05;
    cmd_buf[2] = 0x00;
    send_cmd(cmd_buf);
}

void MP3::next()
{
    cmd_buf[1] = 0x06;
    cmd_buf[2] = 0x00;
    send_cmd(cmd_buf);
}

void MP3::play_track(uint16_t file)
{
    cmd_buf[1] = 0x07;
    cmd_buf[2] = 0x02;
    cmd_buf[3] = (uint8_t)(file >> 8);
    cmd_buf[4] = (uint8_t)file;
    send_cmd(cmd_buf);
}

int MP3::get_selected_source()
{
    while(COM_SOFT.available()) COM_SOFT.read();

    uint8_t i = 0;
    uint8_t length = 0;
    uint8_t checksum = 0;
    
    cmd_buf[1] = 0x09;
    cmd_buf[2] = 0x00;
    send_cmd(cmd_buf);
    
    if (COM_SOFT.available()) {
        while (COM_SOFT.available()) {
            char c = COM_SOFT.read();
            reply_buf[length++] = c;
            delay(2);
        }
    }

    for (i = 0; i < length - 1; i++) {
        checksum += reply_buf[i];
    }
    if (reply_buf[length - 1] == checksum) {
        return reply_buf[3];
    }
    
    return -1;
}

int MP3::get_playing_source()
{
    while(COM_SOFT.available()) COM_SOFT.read();

    uint8_t i = 0;
    uint8_t length = 0;
    uint8_t checksum = 0;
    
    cmd_buf[1] = 0x0A;
    cmd_buf[2] = 0x00;
    send_cmd(cmd_buf);
    
    if (COM_SOFT.available()) {
        while (COM_SOFT.available()) {
            char c = COM_SOFT.read();
            reply_buf[length++] = c;
            delay(2);
        }
    }

    for (i = 0; i < length - 1; i++) {
        checksum += reply_buf[i];
    }
    if (reply_buf[length - 1] == checksum) {
        return reply_buf[3];
    }
    
    return -1;
}

void MP3::switch_playback_source(uint8_t src)
{
    cmd_buf[1] = 0x0B;
    cmd_buf[2] = 0x01;
    cmd_buf[3] = src;
    send_cmd(cmd_buf);
}

int MP3::get_total_tracks()
{
    while(COM_SOFT.available()) COM_SOFT.read();

    uint8_t i = 0;
    uint8_t length = 0;
    uint8_t checksum = 0;
    
    cmd_buf[1] = 0x0C;
    cmd_buf[2] = 0x00;
    send_cmd(cmd_buf);
    
    if (COM_SOFT.available()) {
        while (COM_SOFT.available()) {
            char c = COM_SOFT.read();
            reply_buf[length++] = c;    
            delay(2);
        }
    }

    for (i = 0; i < length - 1; i++) {
        checksum += reply_buf[i];
    }

    if (reply_buf[length - 1] == checksum) {
        return (reply_buf[3] << 8) | reply_buf[4];
    }
    
    return -1;
}

int MP3::get_current_track()
{
    while(COM_SOFT.available()) COM_SOFT.read();

    uint8_t i = 0;
    uint8_t length = 0;
    uint8_t checksum = 0;
    
    cmd_buf[1] = 0x0D;
    cmd_buf[2] = 0x00;
    send_cmd(cmd_buf);
    
    if (COM_SOFT.available()) {
        while (COM_SOFT.available()) {
            char c = COM_SOFT.read();
            reply_buf[length++] = c;    
            delay(2);
        }
    }

    for (i = 0; i < length - 1; i++) {
        checksum += reply_buf[i];
    }

    if (reply_buf[length - 1] == checksum) {
        return (reply_buf[3] << 8) | reply_buf[4];
    }
    
    return -1;
}

void MP3::volume(uint8_t vol)
{
    cmd_buf[1] = 0x13;
    cmd_buf[2] = 0x01;
    cmd_buf[3] = vol;
    send_cmd(cmd_buf);
}

void MP3::increase_volume()
{
    cmd_buf[1] = 0x14;
    cmd_buf[2] = 0x00;
    send_cmd(cmd_buf);
}

void MP3::decrease_volume()
{
    cmd_buf[1] = 0x15;
    cmd_buf[2] = 0x00;
    send_cmd(cmd_buf);
}

void MP3::cycle_mode(uint8_t mode)
{
    cmd_buf[1] = 0x18;
    cmd_buf[2] = 0x01;
    cmd_buf[3] = mode;
    send_cmd(cmd_buf);
}

void MP3::loop(uint16_t num)
{
    cmd_buf[1] = 0x19;
    cmd_buf[2] = 0x02;
    cmd_buf[3] = (uint8_t)(num >> 8);
    cmd_buf[4] = (uint8_t)num;
    send_cmd(cmd_buf);
}

void MP3::eq(uint8_t mode)
{
    cmd_buf[1] = 0x1A;
    cmd_buf[2] = 0x01;
    cmd_buf[3] = mode;
    send_cmd(cmd_buf);
}

void MP3::playback_channel(uint8_t channel)
{
    cmd_buf[1] = 0x1D;
    cmd_buf[2] = 0x01;
    cmd_buf[3] = channel;
    send_cmd(cmd_buf);
}

void MP3::total_track_time()
{
    cmd_buf[1] = 0x24;
    cmd_buf[2] = 0x00;
    send_cmd(cmd_buf);
}

void MP3::enable_track_time_report()
{
    cmd_buf[1] = 0x25;
    cmd_buf[2] = 0x00;
    send_cmd(cmd_buf);    
}

void MP3::disable_track_time_report()
{
    cmd_buf[1] = 0x26;
    cmd_buf[2] = 0x00;
    send_cmd(cmd_buf);    
}

MP3 mp3;

void setup() {
  mp3.begin();

  mp3.volume(10); // Set max volume (1-30)
  Serial.begin(9600);
  
}

void loop() {
  Serial.println("he");
  mp3.play_track(2); // Play 2nd track
  delay(3000);

  mp3.play_track(1); // Play 1st track
  delay(3000);
  mp3.stop();
}
