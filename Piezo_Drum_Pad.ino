#include <ss_oled.h>         // Small Simple OLED Library
#include <EEPROM.h>          // EEPROM Library
#include <AltSoftSerial.h>   // External MIDI Input - must use pin 8 RX & pin 9 TX

#define MIDI_BAUD_RATE 31250
#define SERIAL_BAUD_RATE 115200

// Pad constants
#define PADS 6 // Number of pads
#define PAD_ATTACK 20 // Time to capture peak (milliseconds)
#define PAD_DECAY 20 // Decay time to half peak (milliseconds)
#define PAD_MAX_DEBOUNCE 60 // Max debounce time (milliseconds)
#define NOTE_OFF_DELAY 20 // Delay before sending Pad Note Off (milliseconds)
#define PAD_QUIET 100 // Default quiet threshold
#define PAD_SENSITIVITY 1000 // Signal to cause max. velocity, PAD_QUIET +1 min, 1023 max.
#define PAD_MIN_VELOCITY 15 // Must be less than 127

// Digital Pad constants
#define DPADS 2 // Number of digital pad inputs
#define DPAD_DEBOUNCE 20

// Pedal constants
#define PEDALS 3 // Number of pedal inputs
#define PEDAL_DEBOUNCE 20
#define KICK_PEDAL 0
#define HI_HAT_PEDAL 1
#define SPARE_PEDAL 2

// Menu Switch constants
#define BUTTONS 2 // Number of pushbutton inputs
#define BUTTON_DEBOUNCE 100
#define LEFT 0
#define RIGHT 1
#define SETNRUN_DEBOUNCE 250

// Hardware values
#define MAX_ADC 1023
#define MAX_TRIGGERS    PADS + DPADS + PEDALS + 1
#define HIGH_HAT_CLOSED PADS + DPADS + PEDALS

// Midi values
#define NOTE_OFF       0x80   // three bytes
#define NOTE_ON        0x90   // three bytes
#define POLY_PRESSURE  0xA0   // three bytes
#define CONTROL_CHANGE 0xB0   // three bytes
#define PROG_CHANGE    0xC0   // two bytes
#define CHAN_PRESSURE  0xD0   // two bytes
#define PITCH_BEND     0xE0   // three bytes
#define SYSX_START     0xF0   // starts sysex message
#define MIDI_TIME_CODE 0xF1   // two bytes
#define SONG_POSITION  0xF2   // three bytes
#define SONG_SELECT    0xF3   // two bytes
//undefined reserved   0xF4
//undefined reserved   0xF5
#define TUNE_REQUEST   0xF6   // one byte
#define SYSX_STOP      0xF7   // ends sysex message
#define CLOCK          0xF8   // one byte (system realtime)
//undefined reserved   0xF9
#define START          0xFA   // one byte (system realtime)
#define CONTINUE       0xFB   // one byte (system realtime)
#define STOP           0xFC   // one byte (system realtime)
//undefined reserved   0xFD
#define ACTIVE_SENSE   0xFE   // one byte (system realtime)
#define MIDI_RESET     0xFF   // one byte (system realtime)
#define DRUM_CHANNEL   0x09   //Defacto percussion channel
#define CHAN_VOLUME    0x07
#define ALL_NOTES_OFF  0x78
#define NULL_VALUE     0

// Menu items
#define MAX_KITS 19
#define MAX_DRUM 87
#define MIN_DRUM 13
#define OLED_STRING_LENGTH 8
#define OLED_MAX_LINES 3
#define MENU_STEP_DELAY 300
#define MENU_SINGLE_DELAY 600
#define MENU_VOLUME 60
#define MAX_VOL 127
#define MIN_VOL 10
#define PRESET_SIZE 40
#define MAX_PRESETS 24
#define VOL_CHANGE 5

// Digital Input pins
#define SERIAL_IN 0      // Code Upload & USB MIDI Input
#define SERIAL_OUT 1     // Serial Monitor & USB MIDI Output
#define RIGHT_BUTTON 2   // Right Menu Button
#define LEFT_BUTTON 3    // Leftt Menu Button
#define HI_HAT_CHOKE 4   // Hi-Hat Choke Footswitch
#define MODE_SWITCH 5    // Set not Run Toggle Switch
#define DPAD_L 6         // Digital Pad Left (Piezo thru NFET)
#define SPARE_INPUT 7    // Spare Footswitch (Start Stop)
#define EXT_MIDI_IN 8    // Midi from Keyboard or QY70
#define EXT_MIDI_OUT 9   // Midi to QY70
// Pin 10 = OLED SCL     // Port B2 assigned in OLED Init below
#define KICK_DRUM 11     // Kick Drum Footswitch
// Pin 12 = OLED SDA     // Port B4 assigned in OLED Init below
#define DPAD_R 13        // Digital Pad Right (Piezo thru NFET)

// Setup software MIDI link for External Keyboard In and QY70 Out
AltSoftSerial Ext_Midi(EXT_MIDI_IN, EXT_MIDI_OUT);

void dummydummydummy() {} //Forces compiler to place implicit prototypes here for menu array

// macros for manipulating single bit Booleans
#define BB_TRUE(bp,bb)    bp |= bb
#define BB_FALSE(bp,bb)   bp &= ~(bb)
#define BB_READ(bp,bb)    bool(bp & bb)
#define BB_NOT(bp,bb)     bp ^= bb   // XOR

// Global Variables
unsigned int bb_bools = 0x0EC4;  // 16 bits replaces 16 booleans, #defined as powers of two
#define enable_debug 1  // default FALSE
#define usb_midi 2      // default FALSE
float velocity = 0;
unsigned long aquire_time = 0;
char line_buffer[OLED_STRING_LENGTH];
#define switches 4     // default TRUE

// Drum variables
byte current_drum = MIN_DRUM;
byte current_chan_vol = 100;
byte hi_hat_pad = 4;
byte drum_note[MAX_TRIGGERS]   = {38, 50, 48, 47, 46,  51,  49,  45, 36, 44,  32,  42};
byte drum_volume[MAX_TRIGGERS] = {80, 80, 80, 80, 127, 127, 100, 80, 80, 127, 100, 127};
// Snare, Tom1, Tom2, Tom3, HHOpen, Ride, Crash, Tom4, Kick, HHPedal, Spare(Sticks), HHClosed

// Pad variables
int pad_threshold[PADS]; // Trigger threshold
int pad_debounce[PADS];
unsigned long pad_timer[PADS];
byte pad_on_sent = 0xFF;   // used as single bit booleans
byte pad_off_sent[PADS];
int pad_highscore[PADS];

// Digital Pad Variables
unsigned long dpad_timer[DPADS] = {0, 0};
byte dpad_pressed = 0x00;   // used as single bit booleans
byte dpad_on_sent[DPADS] = {0, 0};
byte dpad_highscore[DPADS] = {0, 0};

// Pedal Variables
unsigned long pedal_timer[PEDALS] = {0, 0, 0};
byte pedal_pressed = 0x00;   // used as single bit booleans

// Menu Switch Variables
unsigned long run_timer = 0;
unsigned long set_timer = 0;
unsigned long left_timer = 0;
unsigned long right_timer = 0;
#define nleft_button 8    // Default False  (left button single shot)
#define nright_button 16  // Default False  (right button single shot)

// Menu Structure Variables
#define set_nrun 32        // default FALSE
byte main_menu = 0;
byte play_menu = 0;
#define menu_up_ndown 64   // default TRUE
#define single_step 128    // default TRUE
unsigned long menu_timer = 0;
byte kit_in_use = 0;  // n maps to Kit n+1
PROGMEM const byte kit_type[] = {1, 2, 3, 4, 9, 10, 17, 18, 25, 26, 27, 28, 29, 30, 33, 34, 41, 49, 113};
byte edit_drum = drum_note[0];
byte edit_dvol = drum_volume[0];
char dec_char[4];
byte current_pad = 0;
#define triggered 256     // default FALSE
char name_str[] = {"NEW-SONG"};
byte name_pointer = 0;
byte preset_pointer = MAX_PRESETS;
byte prev_pointer = MAX_PRESETS;
byte next_pointer = MAX_PRESETS;
#define start_stop_enable 512   // default TRUE
#define start_nstop 1024        // default TRUE

// MIDI Routing Variables
byte ext_status = STOP;
byte ext_data = STOP;
byte usb_status = STOP;
byte usb_data = STOP;
#define ext_midi_echo 2048     // default TRUE
#define qy_direct 4096         // default FALSE

// Fixed Menu Strings
PROGMEM const char start_str[][OLED_STRING_LENGTH]  = {{"DRUMKIT "}, {"by Chris"}, {"WELCOME "}};
PROGMEM const char dump_str[][OLED_STRING_LENGTH]   = {{" EEPROM "}, {" SERIAL "}, {"  DUMP  "}};
PROGMEM const char write_str[][OLED_STRING_LENGTH]  = {{"STORE TO"}, {"Preset  "}, {"        "}};
PROGMEM const char read_str[][OLED_STRING_LENGTH]   = {{" SELECT "}, {"Preset  "}, {"        "}};
PROGMEM const char wait_str[][OLED_STRING_LENGTH]   = {{"        "}, {"  WAIT  "}, {"        "}};
PROGMEM const char hihat_str[][OLED_STRING_LENGTH]  = {{" SELECT "}, {" HI-HAT "}, {" PAD    "}};
PROGMEM const char vol_str[][OLED_STRING_LENGTH]    = {{" VOL UP "}, {"VOL DOWN"}, {"PRESS ->"}};
PROGMEM const char spare_str[][OLED_STRING_LENGTH]  = {{"FOOT SW "}, {"PLAY || "}, {"DISABLED"},
                                                       {"ENABLED "}};
PROGMEM const char debug_str[][OLED_STRING_LENGTH]  = {{" SERIAL "}, {" DEBUG  "}, {"DISABLED"},
                                                       {"ENABLED "}};
PROGMEM const char echo_str[][OLED_STRING_LENGTH]   = {{"EXT MIDI"}, {"  ECHO  "}, {"DISABLED"},
                                                       {"ENABLED "}};
PROGMEM const char direct_str[][OLED_STRING_LENGTH] = {{"QY MIDI "}, {" DIRECT "}, {"DISABLED"},
                                                       {"ENABLED "}};
PROGMEM const char baud_str[][OLED_STRING_LENGTH]   = {{"SET BAUD"}, {" 31,250 "}, {"115,200 "},
                                                       {"USB MIDI"}, {" SERIAL "}, {"BAUDRATE"}};
PROGMEM const char menu_str[][OLED_STRING_LENGTH]   = {{"PLAYMODE"}, {"SET DRUM"}, {"Key     "},   
             {"Vol     "}, {"SET NAME"}, {"DEFAULT "},        {"u"},        {"d"},        {"^"}};
PROGMEM const char kit_str[][OLED_STRING_LENGTH]    = {{"SET KIT "}, {"PR-CH   "}, {" STD 1  "},
             {" STD 2  "}, {"  DRY   "}, {" BRIGHT "}, {"  ROOM  "}, {"  DARK  "}, {" ROCK 1 "},
             {" ROCK 2 "}, {"ELECTRO "}, {"ANALOG 1"}, {"ANALOG 2"}, {"DANCE qy"}, {"HIP HOP "},
             {" JUNGLE "}, {" JAZZ 1 "}, {" JAZZ 2 "}, {" BRUSH  "}, {"SYMPHONY"}, {"DANCE kb"}};
PROGMEM const char pad_str[][OLED_STRING_LENGTH]    = {{"SET PAD1"}, {"SET PAD2"}, {"SET PAD3"},
                           {"SET PAD4"}, {"SET PAD5"}, {"SET PAD6"}, {"SET PAD7"}, {"SET PAD8"},
                                         {"KICK PDL"}, {"HH PEDAL"}, {"SPARE IP"}, {"HHCLOSED"}};
PROGMEM const char drum_str[][OLED_STRING_LENGTH]   = {{"SurdoMut"}, {"SurdoOpn"}, {"  Hi Q  "},
  {"WhipSlap"}, {"ScratchH"}, {"ScratchL"}, {"FngrSnap"}, {" Click  "}, {"Metronom"}, {"MetrBell"},
  {"SeqClckL"}, {"SeqClckH"}, {"BrushTap"}, {"BrushSwl"}, {"BrushSlp"}, {"BrushTap"}, {"SnareRll"},
  {"Castanet"}, {"SnareSft"}, {" Sticks "}, {"KickSoft"}, {"Open Rim"}, {"KickTght"}, {"  Kick  "},
  {"SideStck"}, {" Snare  "}, {"HandClap"}, {"SnreTght"}, {"FlorTomL"}, {"HiHtClsd"}, {"FlorTomH"},
  {"HiHatPdl"}, {"Low Tom "}, {"HiHatOpn"}, {"Mid TomL"}, {"Mid TomH"}, {"CrashCym"}, {"High Tom"},
  {"Ride Cym"}, {"ChinaCym"}, {"Ride Cym"}, {"Tamborin"}, {"SplshCym"}, {"Cowbell "}, {"CrshCym2"},
  {"Vibrslap"}, {"RideCym2"}, {"Bongo H "}, {"Bongo L "}, {"CongaHMt"}, {"CongaHOp"}, {"Conga L "},
  {"TimbaleH"}, {"TimbaleL"}, {"Agogo H "}, {"Agogo L "}, {" Cabasa "}, {"Maracas "}, {"SambWslH"},
  {"SambWslL"}, {"Guiro S "}, {"Guiro L "}, {" Claves "}, {"WoodBlkH"}, {"WoodBlkL"}, {"CuicaMut"},
  {"CuicaOpn"}, {"TriangMt"}, {"TriangOp"}, {" Shaker "}, {"JinglBls"}, {"  Bell  "}, {" Horse  "},
  {"BrdTweet"}, {"Firework"}};

// Menu function arrays
typedef void (* PM_Func) ();
PROGMEM const PM_Func menu_page[] = {print_kit_page,     print_drumap,      print_drum_vol,   print_hihat_page,
                                     print_name,         print_write_page,  print_baud_page,  print_echo_page,
                                     print_direct_page,  print_spare_page,  print_dump_page,  print_debug_page};

PROGMEM const PM_Func menu_function[] = {set_kit,        edit_drumap,       edit_drum_vol,    set_hihat,
                                         edit_name,      write_eeprom,      set_baud,         set_echo,
                                         set_direct,     set_spare,         dump_eeprom,      set_debug};

PROGMEM const PM_Func play_page[] = {print_run_page,     print_read_page,   print_chanvol_up, print_chanvol_dn};

PROGMEM const PM_Func play_function[] = {change_kit,     change_preset,     send_chan_vol_up, send_chan_vol_dn};

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);  // changed later if Mode switch is in Play position
  Ext_Midi.begin(MIDI_BAUD_RATE);  // always 31,250 baud
  
  // Pad Initialisation
  for (byte x = 0; x < PADS; x++) {
    pad_off_sent[x] = 0;
    pad_highscore[x] = 0;
    pad_timer[x] = 0;
    pad_debounce[x] = PAD_MAX_DEBOUNCE;
    pad_threshold[x] = PAD_QUIET;
  }

  // Digital Input Initialisation
  //pinMode(SERIAL_IN, INPUT);  // already input
  digitalWrite(SERIAL_IN, HIGH);
  pinMode(RIGHT_BUTTON, INPUT);
  digitalWrite(RIGHT_BUTTON, HIGH);
  pinMode(LEFT_BUTTON, INPUT);
  digitalWrite(LEFT_BUTTON, HIGH);
  pinMode(SPARE_INPUT, INPUT);
  digitalWrite(SPARE_INPUT, HIGH);
  pinMode(MODE_SWITCH, INPUT);
  digitalWrite(MODE_SWITCH, HIGH);
  pinMode(DPAD_L, INPUT);
  digitalWrite(DPAD_L, HIGH);
  pinMode(HI_HAT_CHOKE, INPUT);
  digitalWrite(HI_HAT_CHOKE, HIGH);
  pinMode(DPAD_R, INPUT);
  digitalWrite(DPAD_R, HIGH);
  pinMode(KICK_DRUM, INPUT);
  digitalWrite(KICK_DRUM, HIGH);
  pinMode(EXT_MIDI_IN, INPUT);
  digitalWrite(EXT_MIDI_IN, HIGH);
  pinMode(EXT_MIDI_OUT, OUTPUT);
  digitalWrite(EXT_MIDI_OUT, HIGH);

  // Midi baud rate selected if Mode switch is in Play position during boot
  if (not digitalRead(MODE_SWITCH)) {
    Serial.end();
    delay(100);
    Serial.begin(MIDI_BAUD_RATE);
    BB_TRUE(bb_bools, usb_midi);
  }

  // Screen Initialisation
  void (oledInit(OLED_128x64, 0, 0, 0xB4, 0xB2, 100000L)); // PB4 pin 12 = SDA, PB2 pin 10 = SCL
  oledFill(0x0, 1);
  print_start_page();
  delay(2000);
  print_line_P(0, &baud_str[5]);
  print_line_P(1, &baud_str[2]);
  print_line_P(2, &baud_str[4]);
  delay(2000);
  oledSetContrast(32);
  if (not digitalRead(MODE_SWITCH)) {
    print_line_P(0, &menu_str[0]);
    print_line_P(1, &kit_str[2]);
    print_line_P(2, &menu_str[5]);
  }

  // Send initial program change
  send_kit_change(pgm_read_byte_near(kit_type + kit_in_use));
}

void loop() {
  // Deal with set nrun switch
  aquire_time = millis();
  if (not digitalRead(MODE_SWITCH) && BB_READ(bb_bools, set_nrun)) {
    if (aquire_time - run_timer > SETNRUN_DEBOUNCE) {   //debounce set_nrun switch
      run_menu(BUTTONS);
      BB_FALSE(bb_bools, set_nrun);
    }
  } else {
    run_timer = aquire_time;
  }
  if (digitalRead(MODE_SWITCH) && not BB_READ(bb_bools, set_nrun)) {
    if (aquire_time - set_timer > SETNRUN_DEBOUNCE) {   //debounce set_nrun switch
      settings_menu(BUTTONS);
      BB_TRUE(bb_bools, set_nrun);
      send_all_notes_off();
    }
  } else {
    set_timer = aquire_time;
  }
  // Deal with external MIDI stream
  if (Ext_Midi.available() > 0) {
    byte ext_midi_byte;
    ext_midi_byte = Ext_Midi.read();
    if (BB_READ(bb_bools, qy_direct)) {
      send_usb_single(ext_midi_byte);
    } else {
      if (ext_midi_byte >= NOTE_OFF) {    // Status byte received
        if (ext_midi_byte >= CLOCK) {     // System realtime message
          send_usb_single(ext_midi_byte);
          if (BB_READ(bb_bools, ext_midi_echo)) {
            send_midi_single(ext_midi_byte);
          }
        } else {
          ext_status = ext_midi_byte;
          if ((ext_midi_byte > SONG_SELECT)    // Single byte message
            || (ext_midi_byte == SYSX_START)) {
            send_usb_single(ext_midi_byte);
            if (BB_READ(bb_bools, ext_midi_echo)) {
              send_midi_single(ext_midi_byte);
            }
          }
          ext_data = STOP;
        }
      } else {                                 // Data byte received
        if (ext_status == SYSX_START) {        // Sysex data received
          send_usb_single(ext_midi_byte);
          if (BB_READ(bb_bools, ext_midi_echo)) {
            send_midi_single(ext_midi_byte);
          }
        } else {
          if (((ext_status & 0xF0) == PROG_CHANGE)    // Double byte message
            || ((ext_status & 0xF0) == CHAN_PRESSURE)
            || (ext_status == SONG_SELECT)
            || (ext_status == MIDI_TIME_CODE)) {
            send_usb_double(ext_status, ext_midi_byte);
            if (BB_READ(bb_bools, ext_midi_echo)) {
              send_midi_double(ext_status, ext_midi_byte);
            }
          } else {                                    // Triple byte message
            if (ext_data == STOP) {
              ext_data = ext_midi_byte;
            } else {
              send_usb_triple(ext_status, ext_data, ext_midi_byte);
              if (BB_READ(bb_bools, ext_midi_echo)) {
                send_midi_triple(ext_status, ext_data, ext_midi_byte);
              }
              ext_data = STOP;
            }
          }
        }
      }
    }
  }
  // Deal with USB MIDI stream
  if (Serial.available() > 0) {
    byte usb_midi_byte;
    usb_midi_byte = Serial.read();
    if (BB_READ(bb_bools, qy_direct)) {
      send_midi_single(usb_midi_byte);
    } else {
      if (usb_midi_byte >= NOTE_OFF) {    // Status byte received
        if (usb_midi_byte >= CLOCK) {     // System realtime message
          send_midi_single(usb_midi_byte);
        } else {
          usb_status = usb_midi_byte;
          if  ((usb_midi_byte > SONG_SELECT)    // Single byte message
            || (usb_midi_byte == SYSX_START)) {
            send_midi_single(usb_midi_byte);
          }
          usb_data = STOP;
        }
      } else {                                  // Data byte received
        if (usb_status == SYSX_START) {         // Sysex data received
          send_midi_single(usb_midi_byte);
        } else {
          if (((usb_status & 0xF0) == PROG_CHANGE)   // Double byte message
            || ((usb_status & 0xF0) == CHAN_PRESSURE)
            || (usb_status == SONG_SELECT)
            || (usb_status == MIDI_TIME_CODE)) {
            send_midi_double(usb_status, usb_midi_byte);
          } else {                                   // Triple byte message
            if (usb_data == STOP) {
              usb_data = usb_midi_byte;
            } else {
              send_midi_triple(usb_status, usb_data, usb_midi_byte);
              usb_data = STOP;
            }
          }
        }
      }
    }
  }
  // Deal with piezo inputs
  current_drum = 0;
  BB_FALSE(bb_bools, triggered);
  for (byte x = 0; x < PADS; x++) {
    int pad_reading = analogRead(x);
    aquire_time = millis();
    if (pad_reading > pad_highscore[x]) pad_highscore[x] = pad_reading;
    if (aquire_time - pad_timer[x] > PAD_ATTACK) {    // wait until after attack window
      if (not BB_READ(pad_on_sent, 0x01 << x)) {      // midi note not sent yet
        send_pad_data(drum_note[x], pad_highscore[x], drum_volume[x]);
        pad_off_sent[x] = drum_note[x];
        current_pad = current_drum;                   // used for pad edit menu
        BB_NOT(bb_bools, menu_up_ndown);
        BB_TRUE(bb_bools, triggered);
        BB_TRUE(pad_on_sent, 0x01 << x);
        pad_threshold[x] = pad_highscore[x];
        pad_highscore[x] = PAD_QUIET;
      }
      if (pad_reading >= pad_threshold[x]) {
        if (aquire_time - pad_timer[x] <= pad_debounce[x]) {  // within debounce period, so extend debounce
          pad_debounce[x] = byte(aquire_time - pad_timer[x]) + PAD_DECAY;
        } else {                                              // outside debounce period, so new hit
          pad_highscore[x] = pad_reading;
          pad_threshold[x] = MAX_ADC;
          pad_debounce[x] = PAD_ATTACK + PAD_DECAY;
          pad_timer[x] = aquire_time;
          BB_FALSE(pad_on_sent, 0x01 << x);
        }
      } else {  // No new hits detected
        if (not (aquire_time - pad_timer[x]) <= pad_debounce[x]) {  // reduce threshold and extend debounce
          pad_threshold[x] = pad_threshold[x] >> 1;
          if (pad_threshold[x] < PAD_QUIET) pad_threshold[x] = PAD_QUIET;
          pad_debounce[x] = pad_debounce[x] + PAD_DECAY;
          if (pad_debounce[x] >= PAD_MAX_DEBOUNCE) pad_debounce[x] = PAD_MAX_DEBOUNCE;
        }
      }
      if ((aquire_time - pad_timer[x] > PAD_ATTACK + NOTE_OFF_DELAY) && pad_off_sent[x] != 0) {
        send_drum_note_off(pad_off_sent[x]);      // send Note Off after delay
        pad_off_sent[x] = 0;
      }
    }
    current_drum++;
  }
  // Deal with digital pad inputs
  for (byte x = 0; x < DPADS; x++) {
    BB_FALSE(bb_bools, switches);
    if (x == 0) {
      if (digitalRead(DPAD_L)) BB_TRUE(bb_bools, switches);
    }
    if (x == 1) {
      if (digitalRead(DPAD_R)) BB_TRUE(bb_bools, switches);
    }
    aquire_time = millis();
    if (aquire_time - dpad_timer[x] > DPAD_DEBOUNCE) {
      if (dpad_on_sent[x] != 0) {
        send_drum_note_off(dpad_on_sent[x]);
        dpad_on_sent[x] = 0;
        dpad_timer[x] = aquire_time;
      } else {
        if (BB_READ(dpad_pressed, 0x01 << x)) {    // Dpad hit during debounce period
          dpad_on_sent[x] = drum_note[current_drum];
          current_pad = current_drum;
          BB_NOT(bb_bools, menu_up_ndown);
          send_drum_note_on(dpad_on_sent[x], dpad_highscore[x]);
          BB_TRUE(bb_bools, triggered);
          BB_FALSE(dpad_pressed, 0x01 << x);
          dpad_timer[x] = aquire_time;
        } else {
          if (not BB_READ(bb_bools, switches)) {
            BB_TRUE(dpad_pressed, 0x01 << x);
            dpad_timer[x] = aquire_time;
            dpad_highscore[x] = drum_volume[current_drum];
          }
        }
      }
    } else {
      if (BB_READ(bb_bools, switches)) {
        if (dpad_highscore[x] == drum_volume[current_drum]) {
          unsigned long highscore;
          highscore = ((aquire_time - dpad_timer[x]) << 3) + 16;
          if (highscore >= drum_volume[current_drum]) {
            dpad_highscore[x] = drum_volume[current_drum];
          } else {
            dpad_highscore[x] = byte(highscore);
          }
        }
      }
    }
    current_drum++;
  }
  // Deal with pedal inputs
  for (byte x = 0; x < PEDALS; x++) {
    BB_FALSE(bb_bools, switches);
    if (x == 0) {
      if (digitalRead(KICK_DRUM)) BB_TRUE(bb_bools, switches);
    }
    if (x == 1) {
      if (digitalRead(HI_HAT_CHOKE)) BB_TRUE(bb_bools, switches);
    }
    if (x == 2) {
      if (digitalRead(SPARE_INPUT)) BB_TRUE(bb_bools, switches);
    }
    aquire_time = millis();
    if (aquire_time - pedal_timer[x] > PEDAL_DEBOUNCE) {
      if (BB_READ(bb_bools, switches)) {
        if (BB_READ(pedal_pressed, 0x01 << x)) {
          send_drum_note_off(drum_note[current_drum]);
          if (x == HI_HAT_PEDAL) swap_hi_hat();   // If HiHat opens, swap drum notes
          pedal_timer[x] = aquire_time;
          BB_FALSE(pedal_pressed, 0x01 << x);
        }
      } else {
        if (not BB_READ(pedal_pressed, 0x01 << x)) {
          BB_TRUE(pedal_pressed, 0x01 << x);
          if ((x == SPARE_PEDAL) && BB_READ(bb_bools, start_stop_enable)) {
            if (BB_READ(bb_bools, start_nstop)) {
              send_midi_single(START);
              send_usb_single(START);
            } else {
              send_midi_single(STOP);
              send_usb_single(STOP);
            }
            BB_NOT(bb_bools, start_nstop);
          } else {
            send_drum_note_on(drum_note[current_drum], drum_volume[current_drum]);
            if (x == HI_HAT_PEDAL) swap_hi_hat();   // If HiHat closes, swap drum notes
          }
          current_pad = current_drum;
          BB_NOT(bb_bools, menu_up_ndown);
          BB_TRUE(bb_bools, triggered);
          pedal_timer[x] = aquire_time;
        }
      }
    }
    current_drum++;
  }
  // Deal with menu buttons
  aquire_time = millis();
  if (not digitalRead(LEFT_BUTTON)) {
    if (aquire_time - left_timer > BUTTON_DEBOUNCE) {   //debounce left button
      if (BB_READ(bb_bools, nleft_button)) {
        BB_FALSE(bb_bools, nleft_button);
        if (BB_READ(bb_bools, set_nrun)) {
          settings_menu(LEFT);
        } else {
          run_menu(LEFT);
        }
      }
    }
  } else {
    BB_TRUE(bb_bools, nleft_button);
    left_timer = aquire_time;
  }
  if (not digitalRead(RIGHT_BUTTON)) {
    if (aquire_time - right_timer > BUTTON_DEBOUNCE) {   //debounce right button
      if (BB_READ(bb_bools, nright_button)) {
        BB_FALSE(bb_bools, nright_button);
        if (BB_READ(bb_bools, set_nrun)) {
          settings_menu(RIGHT);
        } else {
          run_menu(RIGHT);
        }
      }
    }
  } else {
    BB_TRUE(bb_bools, nright_button);
    right_timer = aquire_time;
  }
  // Allow menus to run triggered via pads etc.
  if (BB_READ(bb_bools, triggered)) {
    if (BB_READ(bb_bools, set_nrun)) {
      settings_menu(BUTTONS);
    } else {
      if (play_menu == 1) run_menu(BUTTONS);  //only runs on Preset Select
    }
  }
}

void swap_hi_hat() {   //swap notes and relative volumes
  byte temp_hh;
  temp_hh = drum_note[hi_hat_pad];
  drum_note[hi_hat_pad] = drum_note[HIGH_HAT_CLOSED];
  drum_note[HIGH_HAT_CLOSED] = temp_hh;
  temp_hh = drum_volume[hi_hat_pad];
  drum_volume[hi_hat_pad] = drum_volume[HIGH_HAT_CLOSED];
  drum_volume[HIGH_HAT_CLOSED] = temp_hh;
}

void send_midi_triple(byte command, byte data1, byte data2) {
  Ext_Midi.write(command);
  Ext_Midi.write(data1);
  Ext_Midi.write(data2);
}

void send_midi_double(byte command, byte data) {
  Ext_Midi.write(command);
  Ext_Midi.write(data);
}

void send_midi_single(byte command) {
  Ext_Midi.write(command);
}

void send_usb_triple(byte command, byte data1, byte data2) {
  if (BB_READ(bb_bools, enable_debug)) {
    Serial.print(command, HEX);
    Serial.print(F(" "));
    Serial.print(data1, HEX);
    Serial.print(F(" "));
    Serial.println(data2, HEX);
  } else {
    Serial.write(command);
    Serial.write(data1);
    Serial.write(data2);
  }
}

void send_usb_double(byte command, byte data) {
  if (BB_READ(bb_bools, enable_debug)) {
    Serial.print(command, HEX);
    Serial.print(F(" "));
    Serial.println(data, HEX);
  } else {
    Serial.write(command);
    Serial.write(data);
  }
}

void send_usb_single(byte command) {
  if (BB_READ(bb_bools, enable_debug)) {
    Serial.println(command, HEX);
  } else {
    Serial.write(command);
  }
}

void send_pad_data(byte note, int highscore, int max_velocity) {
  velocity = ((highscore / float(PAD_SENSITIVITY - PAD_QUIET)) *
              float(max_velocity - PAD_MIN_VELOCITY)) + PAD_MIN_VELOCITY;
  if (velocity > max_velocity) velocity = max_velocity;
  send_drum_note_on(note, byte (velocity));
}

void send_drum_note_on(byte note, byte data) {
  send_midi_triple((DRUM_CHANNEL) | NOTE_ON, note, data);
  send_usb_triple((DRUM_CHANNEL) | NOTE_ON, note, data);
}

void send_drum_note_off(byte note) {
  send_midi_triple((DRUM_CHANNEL) | NOTE_OFF, note, NULL_VALUE);
  send_usb_triple((DRUM_CHANNEL) | NOTE_OFF, note, NULL_VALUE);
}

void send_kit_change(byte voice) {
  send_midi_double((DRUM_CHANNEL) | PROG_CHANGE, voice - 1);
  send_usb_double((DRUM_CHANNEL) | PROG_CHANGE, voice - 1);
}

void print_xy(byte tab, byte line, char * string_pointer) {
  if (line >= OLED_MAX_LINES - 1) line = 6;
  if (line == 1) line = 3;
  if (tab >= OLED_STRING_LENGTH) tab = 7;
  tab = tab << 4;
  oledWriteString(0, tab, line, string_pointer, FONT_STRETCHED, 0, 1); //3 lines of 8 characters
}

void print_char_P(byte tab, byte line, unsigned int string_pointer) {
  memcpy_P (&line_buffer, string_pointer, 1);
  print_xy(tab, line, line_buffer);
}

void print_line(byte line, char * string_pointer) {
  print_xy(0, line, string_pointer);
}

void print_line_P(byte line, unsigned int string_pointer) {
  memcpy_P (&line_buffer, string_pointer, OLED_STRING_LENGTH);
  print_line(line, line_buffer);
}

void print_blank_line(byte line) {
  memcpy_P (&line_buffer, wait_str[0], OLED_STRING_LENGTH);
  print_line(line, line_buffer);
}

void print_decimal(byte tab, byte line, byte value) {
  itoa(value, dec_char, DEC);
  print_xy(tab, line, dec_char);
}

void print_wait_page() {
  for (byte i = 0; i < OLED_MAX_LINES; i++) {
    memcpy_P (&line_buffer, wait_str[i], OLED_STRING_LENGTH);
    print_line(i, line_buffer);
  }
}

void print_start_page() {
  for (byte i = 0; i < OLED_MAX_LINES; i++) {
    memcpy_P (&line_buffer, start_str[i], OLED_STRING_LENGTH);
    print_line(i, line_buffer);
  }
}

void send_kit() {
  send_kit_change(pgm_read_byte_near(kit_type + kit_in_use));
}

void set_kit() {
  if (++kit_in_use >= MAX_KITS) kit_in_use = 0;
  send_kit();
  print_kit_page();
}

void print_baud_page() {
  print_line_P(0, &baud_str[0]);
  if (BB_READ(bb_bools, usb_midi)) {
    print_line_P(1, &baud_str[1]);
    print_line_P(2, &baud_str[3]);
  } else {
    print_line_P(1, &baud_str[2]);
    print_line_P(2, &baud_str[4]);
  }
}

void set_baud() {
  Serial.end();
   if (BB_READ(bb_bools, usb_midi)) {
    BB_FALSE(bb_bools, usb_midi);
    Serial.begin(SERIAL_BAUD_RATE);
  } else {
    BB_TRUE(bb_bools, usb_midi);
    Serial.begin(MIDI_BAUD_RATE);
  }
  print_baud_page();
}

void print_echo_page() {
  print_line_P(0, &echo_str[0]);
  print_line_P(1, &echo_str[1]);
  if (BB_READ(bb_bools, ext_midi_echo)) {
    print_line_P(2, &echo_str[3]);
  } else {
    print_line_P(2, &echo_str[2]);
  }
}

void set_echo() {
  BB_NOT(bb_bools, ext_midi_echo);
  print_echo_page();
}

void print_direct_page() {
  print_line_P(0, &direct_str[0]);
  print_line_P(1, &direct_str[1]);
  if (BB_READ(bb_bools, qy_direct)) {
    print_line_P(2, &direct_str[3]);
  } else {
    print_line_P(2, &direct_str[2]);
  }
}

void set_direct() {
  BB_NOT(bb_bools, qy_direct);
  print_direct_page();
}

void print_hihat_page() {
  if (BB_READ(bb_bools, triggered) && (current_pad < 8)) hi_hat_pad = current_pad;
  print_line_P(0, &hihat_str[0]);
  print_line_P(1, &hihat_str[1]);
  print_line_P(2, &hihat_str[2]);
  print_decimal(5, 2, hi_hat_pad + 1);
}

void set_hihat() {
  print_hihat_page();
}

void print_spare_page() {
  print_line_P(0, &spare_str[0]);
  print_line_P(1, &spare_str[1]);
  if (BB_READ(bb_bools, start_stop_enable)) {
    print_line_P(2, &spare_str[3]); ;
  } else {
    print_line_P(2, &spare_str[2]);
  }
}

void set_spare() {
  BB_NOT(bb_bools, start_stop_enable);
  print_spare_page();
}

void print_debug_page() {
  print_line_P(0, &debug_str[0]);
  print_line_P(1, &debug_str[1]);
  print_line_P(2, &debug_str[2]);
  if (BB_READ(bb_bools, enable_debug)) {
    print_line_P(2, &debug_str[3]);
  } else {
    print_line_P(2, &debug_str[2]);
  }
}

void set_debug() {
  BB_NOT(bb_bools, enable_debug);
  print_debug_page();
}

void dump_eeprom() {
  byte data_pointer = 0;
  byte pr_pointer = 0;
  print_wait_page();
  Serial.println();
  Serial.print(F("EE Dump  Name"));
  serial_space(6);
  Serial.print(F("Kit  Drum Note Array"));
  serial_space(22);
  Serial.print(F("Drum Volume Array"));
  serial_space(32);
  Serial.println(F("Hi-Hat Pad"));
  for (byte pr_pointer = 0 ; pr_pointer < MAX_PRESETS ; pr_pointer++) {
    data_pointer = 0;
    Serial.print(F("PRESET "));
    Serial.print(pr_pointer + 1);
    serial_space(1);
    if (EEPROM[(pr_pointer * PRESET_SIZE) + data_pointer] == 255) {
      Serial.println(F("BLANK"));
      delay(100);
    } else {
      for (byte index = 0 ; index < OLED_STRING_LENGTH ; index++) {     // Preset Name
        Serial.write(EEPROM[(pr_pointer * PRESET_SIZE) + data_pointer++]);
        delay(10);
      }
      serial_space(2);
      Serial.print(EEPROM[(pr_pointer * PRESET_SIZE) + data_pointer++]);   // Kit Type
      if (EEPROM[(pr_pointer * PRESET_SIZE) + (data_pointer - 1)] < 10) serial_space(1);
      if (EEPROM[(pr_pointer * PRESET_SIZE) + (data_pointer - 1)] < 100) serial_space(1);
      serial_space(2);
      delay(10);
      for (byte index = 0 ; index < MAX_TRIGGERS ; index++) {   // Drum Note
        Serial.print(EEPROM[(pr_pointer * PRESET_SIZE) + data_pointer++]);
        serial_space(1);
        delay(10);
      }
      serial_space(1);
      for (byte index = 0 ; index < MAX_TRIGGERS ; index++) {    // Drum Volume
        Serial.print(EEPROM[(pr_pointer * PRESET_SIZE) + data_pointer++]);
        if (EEPROM[(pr_pointer * PRESET_SIZE) + (data_pointer - 1)] < 100) serial_space(1);
        serial_space(1);
        delay(10);
      }
      serial_space(1);
      Serial.print(EEPROM[(pr_pointer * PRESET_SIZE) + data_pointer++]);   // Hi Hat Pad
      Serial.println();
    }
  }
  Serial.println(F("Finished"));
  print_dump_page();
}

void serial_space(byte spaces) {
  for (byte index = 0 ; index < spaces ; index++) {
    Serial.print(F(" "));
  }
}

void write_eeprom() {
  byte data_pointer = 0;
  print_wait_page();
  for (byte index = 0 ; index < OLED_STRING_LENGTH ; index++) {
    EEPROM.update((preset_pointer * PRESET_SIZE) + data_pointer++, name_str[index]);
  }
  EEPROM.update((preset_pointer * PRESET_SIZE) + data_pointer++, kit_in_use);
  for (byte index = 0 ; index < MAX_TRIGGERS ; index++) {
    EEPROM.update((preset_pointer * PRESET_SIZE) + data_pointer++, drum_note[index]);
  }
  for (byte index = 0 ; index < MAX_TRIGGERS ; index++) {
    EEPROM.update((preset_pointer * PRESET_SIZE) + data_pointer++, drum_volume[index]);
  }
  EEPROM.update((preset_pointer * PRESET_SIZE) + data_pointer++, hi_hat_pad);
  delay(500);
  print_write_page();
}

void pad_to_preset() {
  if (BB_READ(bb_bools, triggered) && (current_pad < 8)) {
    if (preset_pointer == current_pad * 3) {
      preset_pointer = (current_pad * 3) + 1;
    } else {
      if (preset_pointer == (current_pad * 3) + 1) {
        preset_pointer = (current_pad * 3) + 2;
      } else {
        preset_pointer = current_pad * 3;
      }
    }
  }
}

void print_write_page() {
  char eeprom_name_str[OLED_STRING_LENGTH];
  pad_to_preset();
  for (byte x = 0 ; x < OLED_STRING_LENGTH ; x++) {
    eeprom_name_str[x] = EEPROM[(preset_pointer * PRESET_SIZE) + x];
  }
  print_line_P(0, &write_str[0]);
  print_line_P(1, &write_str[1]);
  print_line_P(2, &write_str[2]);
  print_decimal(6, 1, preset_pointer + 1);
  if (EEPROM[preset_pointer * PRESET_SIZE] != 255) print_line(2, eeprom_name_str);
}

bool edit_drumap_sequence() {
  BB_TRUE(bb_bools, nright_button);  // triggers continuous key presses
  if (millis() - menu_timer > MENU_SINGLE_DELAY + MENU_STEP_DELAY) {
    BB_TRUE(bb_bools, single_step);
  } else {
    if (millis() - menu_timer > MENU_SINGLE_DELAY && BB_READ(bb_bools, single_step)) {
      BB_FALSE(bb_bools, single_step);
    } else {
      if (millis() - menu_timer < MENU_STEP_DELAY || BB_READ(bb_bools, single_step)) return false;
    }
  }
  menu_timer = millis();
  if (BB_READ(bb_bools, menu_up_ndown)) {     // reverses on subsequent pad hits
    if (++edit_drum > MAX_DRUM) edit_drum = MIN_DRUM;
  } else {
    if (--edit_drum < MIN_DRUM) edit_drum = MAX_DRUM;
  }
  return true;
}

void edit_drumap() {
  if (edit_drumap_sequence()) {
    drum_note[current_pad] = edit_drum;
    send_drum_note_on(drum_note[current_pad], MENU_VOLUME);
    delay(20);
    send_drum_note_off(drum_note[current_pad]);
    print_drumap();
  }
}

void print_drumap() {
  edit_drum = drum_note[current_pad];
  print_line_P(0, &pad_str[current_pad]);
  print_line_P(1, &drum_str[edit_drum - MIN_DRUM]);
  print_line_P(2, &menu_str[2]);
  print_decimal(4, 2, drum_note[current_pad]);
  if (BB_READ(bb_bools, menu_up_ndown)) {
    print_char_P(7, 2, &menu_str[6]);
  } else {
    print_char_P(7, 2, &menu_str[7]);
  }
}

bool edit_drum_vol_seq() {
  BB_TRUE(bb_bools, nright_button);  // triggers continuous key presses
  if (millis() - menu_timer > MENU_SINGLE_DELAY + MENU_STEP_DELAY) {
    BB_TRUE(bb_bools, single_step);
  } else {
    if (millis() - menu_timer > MENU_SINGLE_DELAY && BB_READ(bb_bools, single_step)) {
      BB_FALSE(bb_bools, single_step);
    } else {
      if (millis() - menu_timer < MENU_STEP_DELAY || BB_READ(bb_bools, single_step)) return false;
    }
  }
  menu_timer = millis();
  if (++++++++++edit_dvol > MAX_VOL) edit_dvol = MIN_VOL;
  return true;
}

void edit_drum_vol() {
  if (edit_drum_vol_seq()) {
    drum_volume[current_pad] = edit_dvol;
    send_drum_note_on(drum_note[current_pad], drum_volume[current_pad]);
    delay(20);
    send_drum_note_off(drum_note[current_pad]);
    print_drum_vol();
  }
}

void print_drum_vol() {
  edit_dvol = drum_volume[current_pad];
  edit_drum = drum_note[current_pad];
  print_line_P(0, &pad_str[current_pad]);
  print_line_P(1, &drum_str[edit_drum - MIN_DRUM]);
  print_line_P(2, &menu_str[3]);
  print_decimal(4, 2, drum_volume[current_pad]);
}

void edit_name() {
  if (++name_pointer >= OLED_STRING_LENGTH) name_pointer = 0;
  print_name();
}

void print_name() {
  if (name_pointer >= OLED_STRING_LENGTH) name_pointer = 0; // bug:- initially takes value 48, unknown reason
  if (BB_READ(bb_bools, triggered) && (current_pad < 9)) {
    if (name_str[name_pointer] == (current_pad * 3) + 65) {
      name_str[name_pointer] = (current_pad * 3) + 66;
    } else {
      if (name_str[name_pointer] == (current_pad * 3) + 66) {
        name_str[name_pointer] = (current_pad * 3) + 67;
      } else {
        name_str[name_pointer] = (current_pad * 3) + 65;
      }
    }
    if (name_str[name_pointer] == 91) name_str[name_pointer] = 32;
  }
  print_line_P(0, &menu_str[4]);
  print_line_P(1, &wait_str[0]);
  print_line_P(2, &wait_str[0]);
  print_line(1, name_str);
  print_char_P(name_pointer, 2, &menu_str[8]);
}

void print_kit_page() {
  print_line_P(0, &kit_str[0]);
  print_line_P(1, &kit_str[1]);
  print_line_P(2, &kit_str[kit_in_use + 2]);
  if (pgm_read_byte_near(kit_type + kit_in_use) < 100) {
    print_decimal(6, 1, pgm_read_byte_near(kit_type + kit_in_use));
  } else {
    print_decimal(5, 1, pgm_read_byte_near(kit_type + kit_in_use));
  }
}

void print_dump_page() {
  for (byte i = 0; i < OLED_MAX_LINES; i++) {
    memcpy_P (&line_buffer, dump_str[i], OLED_STRING_LENGTH);
    print_line(i, line_buffer);
  }
}

void print_name_str(byte line) {
  char eeprom_name_str[] = {32, 66, 76, 65, 78, 75, 32, 32};
  if (preset_pointer < MAX_PRESETS) {
    if (EEPROM[preset_pointer * PRESET_SIZE] != 255) {
      for (byte x = 0 ; x < OLED_STRING_LENGTH ; x++) {
        eeprom_name_str[x] = EEPROM[(preset_pointer * PRESET_SIZE) + x];
      }
    }
    print_line(line, eeprom_name_str);
  }
}

void read_eeprom() {
  byte data_pointer = 0;
  print_wait_page();
  for (byte index = 0 ; index < OLED_STRING_LENGTH ; index++) {
    name_str[index] = EEPROM[(preset_pointer * PRESET_SIZE) + data_pointer++];
  }
  kit_in_use = EEPROM[(preset_pointer * PRESET_SIZE) + data_pointer++];
  for (byte index = 0 ; index < MAX_TRIGGERS ; index++) {
    drum_note[index] = EEPROM[(preset_pointer * PRESET_SIZE) + data_pointer++];
  }
  for (byte index = 0 ; index < MAX_TRIGGERS ; index++) {
    drum_volume[index] = EEPROM[(preset_pointer * PRESET_SIZE) + data_pointer++];
  }
  hi_hat_pad = EEPROM[(preset_pointer * PRESET_SIZE) + data_pointer++];
  delay(500);
  send_kit();
}

void print_read_page() {
  if (next_pointer == preset_pointer) {
    prev_pointer = preset_pointer;
  } else {
    preset_pointer = next_pointer;
  }
  pad_to_preset();
  next_pointer = preset_pointer;
  print_line_P(0, &read_str[0]);
  print_line_P(1, &read_str[1]);
  print_line_P(2, &read_str[2]);
  print_decimal(6, 1, preset_pointer + 1);
  print_name_str(2);
  preset_pointer = prev_pointer;
}

void change_preset() {
  preset_pointer = next_pointer;
  if (EEPROM[preset_pointer * PRESET_SIZE] != 255) read_eeprom();
  print_read_page();
}

void print_run_page() {
  print_line_P(0, &menu_str[0]);
  print_line_P(1, &kit_str[kit_in_use + 2]);
  if (preset_pointer < MAX_PRESETS) {
    print_line_P(2, &wait_str[0]);  // blank
  } else {
    print_line_P(2, &menu_str[5]);
  }
  if (preset_pointer < MAX_PRESETS) print_name_str(2);
}

void change_kit() {
  if (++kit_in_use >= MAX_KITS) kit_in_use = 0;
  send_kit();
  print_run_page();
}

void send_all_notes_off() {
  for (byte x = 0 ; x < 16 ; x++) {
    send_midi_triple(byte (CONTROL_CHANGE | x), ALL_NOTES_OFF, NULL_VALUE);
    send_usb_triple(byte (CONTROL_CHANGE | x), ALL_NOTES_OFF, NULL_VALUE);
    delay(10);
  }
}

void print_chanvol_up() {
  print_line_P(0, &vol_str[0]);
  print_line_P(1, &wait_str[0]);
  print_line_P(2, &vol_str[2]);
  print_decimal(3, 1, current_chan_vol);
}

void print_chanvol_dn() {
  print_line_P(0, &vol_str[1]);
  print_line_P(1, &wait_str[0]);
  print_line_P(2, &vol_str[2]);
  print_decimal(3, 1, current_chan_vol);
}

void send_chan_vol(byte volume) {
    send_midi_triple(byte (CONTROL_CHANGE | DRUM_CHANNEL), CHAN_VOLUME, volume);
    send_usb_triple(byte (CONTROL_CHANGE | DRUM_CHANNEL), CHAN_VOLUME, volume);
    delay(10);
}

void send_chan_vol_up() {
  current_chan_vol = current_chan_vol + 5;
  if (current_chan_vol > MAX_VOL) {
    current_chan_vol = MAX_VOL - 2;
  } else {
    send_chan_vol(current_chan_vol);
    print_decimal(3, 1, current_chan_vol);
  }
}

void send_chan_vol_dn() {
  current_chan_vol = current_chan_vol - 5;
  if (current_chan_vol < MIN_VOL) {
    current_chan_vol = MIN_VOL;
  } else {
    send_chan_vol(current_chan_vol);
    print_blank_line(1);
    print_decimal(3, 1, current_chan_vol);
  }
}

void settings_menu(byte button) {
  switch (button) {
    case (LEFT):
      if (++main_menu >= sizeof(menu_page) >> 1) main_menu = 0;
      ((PM_Func) pgm_read_word (&menu_page[main_menu])) ();
      break;
    case (RIGHT):
      ((PM_Func) pgm_read_word (&menu_function[main_menu])) ();
      break;
    default:
      ((PM_Func) pgm_read_word (&menu_page[main_menu])) ();
      break;
  }
}

void run_menu(byte button) {
  switch (button) {
    case (LEFT):
      if (++play_menu >= sizeof(play_page) >> 1) play_menu = 0;
      ((PM_Func) pgm_read_word (&play_page[play_menu])) ();
      break;
    case (RIGHT):
      ((PM_Func) pgm_read_word (&play_function[play_menu])) ();
      break;
    default:
      ((PM_Func) pgm_read_word (&play_page[play_menu])) ();
      break;
  }
}
