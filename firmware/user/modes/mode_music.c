/*==============================================================================
 * Includes
 *============================================================================*/

#include <osapi.h>
#include <math.h>
#include <user_interface.h>

#include "user_main.h"
#include "mode_music.h"

#include "hpatimer.h"
#include "buzzer.h"
#include "custom_commands.h"

#include "oled.h"
#include "bresenham.h"
#include "font.h"

#include "embeddedout.h"

#include "buttons.h"

#include "mode_tiltrads.h"

/*==============================================================================
 * Defines
 *============================================================================*/

#define TICK_HEIGHT 2
#define CURSOR_HEIGHT 4
#define BAR_X_MARGIN 0
#define BAR_Y_MARGIN (FONT_HEIGHT_TOMTHUMB + CURSOR_HEIGHT + 1)

#define PITCH_THRESHOLD 32

#define lengthof(x) (sizeof(x) / sizeof(x[0]))

#define REST_BIT 0x10000 // Largest note is 144, which is 0b10010000

#define DEFAULT_PAUSE 5

/*==============================================================================
 * Enums
 *============================================================================*/

typedef enum
{
    THIRTYSECOND_NOTE = 3,
    THIRTYSECOND_REST = 3 | REST_BIT,
    SIXTEENTH_NOTE    = 6,
    SIXTEENTH_REST    = 6 | REST_BIT,
    EIGHTH_NOTE       = 12,
    EIGHTH_REST       = 12 | REST_BIT,
    QUARTER_NOTE      = 24,
    QUARTER_REST      = 24 | REST_BIT,
    HALF_NOTE         = 48,
    HALF_REST         = 48 | REST_BIT,
    WHOLE_NOTE        = 96,
    WHOLE_REST        = 96 | REST_BIT,

    TRIPLET_SIXTYFOURTH_NOTE  = 1,
    TRIPLET_SIXTYFOURTH_REST  = 1 | REST_BIT,
    TRIPLET_THIRTYSECOND_NOTE = 2,
    TRIPLET_THIRTYSECOND_REST = 2 | REST_BIT,
    TRIPLET_SIXTEENTH_NOTE    = 4,
    TRIPLET_SIXTEENTH_REST    = 4 | REST_BIT,
    TRIPLET_EIGHTH_NOTE       = 8,
    TRIPLET_EIGHTH_REST       = 8 | REST_BIT,
    TRIPLET_QUARTER_NOTE      = 16,
    TRIPLET_QUARTER_REST      = 16 | REST_BIT,
    TRIPLET_HALF_NOTE         = 32,
    TRIPLET_HALF_REST         = 32 | REST_BIT,
    TRIPLET_WHOLE_NOTE        = 64,
    TRIPLET_WHOLE_REST        = 64 | REST_BIT,

    DOTTED_SIXTEENTH_NOTE = 9,
    DOTTED_SIXTEENTH_REST = 9 | REST_BIT,
    DOTTED_EIGHTH_NOTE    = 18,
    DOTTED_EIGHTH_REST    = 18 | REST_BIT,
    DOTTED_QUARTER_NOTE   = 36,
    DOTTED_QUARTER_REST   = 36 | REST_BIT,
    DOTTED_HALF_NOTE      = 72,
    DOTTED_HALF_REST      = 72 | REST_BIT,
    DOTTED_WHOLE_NOTE     = 144,
    DOTTED_WHOLE_REST     = 144 | REST_BIT,
} rhythmNote_t;

/*==============================================================================
 * Prototypes
 *============================================================================*/

void ICACHE_FLASH_ATTR musicEnterMode(void);
void ICACHE_FLASH_ATTR musicExitMode(void);
void ICACHE_FLASH_ATTR musicButtonCallback(uint8_t state __attribute__((unused)),
        int button, int down);
void ICACHE_FLASH_ATTR musicAccelerometerHandler(accel_t* accel);
void ICACHE_FLASH_ATTR musicUpdateDisplay(void);
void ICACHE_FLASH_ATTR musicBeatTimerFunc(void* arg __attribute__((unused)));
notePeriod_t ICACHE_FLASH_ATTR arpModify(notePeriod_t note, int8_t arpInterval);
notePeriod_t ICACHE_FLASH_ATTR getCurrentNote(void);
char* ICACHE_FLASH_ATTR noteToStr(notePeriod_t note);
void ICACHE_FLASH_ATTR plotBar(uint8_t yOffset);
void ICACHE_FLASH_ATTR noteToColor( led_t* led, notePeriod_t note, uint8_t brightness);
void ICACHE_FLASH_ATTR paramSwitchTimerFunc(void* arg __attribute__((unused)));

/*==============================================================================
 * Structs
 *============================================================================*/

typedef struct
{
    uint8_t bpmMultiplier;
    uint8_t bpm;
} bpm_t;

typedef struct
{
    rhythmNote_t note;
    int8_t arp;
} rhythmArp_t;

typedef struct
{
    // The parameter's name
    char* name;
    // The rhythm
    const rhythmArp_t* rhythm;
    const uint16_t rhythmLen;
    // Inter-note pause
    const uint16_t interNotePauseMs;
} rhythm_t;

typedef struct
{
    // The parameter's name
    char* name;
    // The notes
    const notePeriod_t* notes;
    const uint16_t notesLen;
} scale_t;

/*==============================================================================
 * Variables
 *============================================================================*/

// The swadge mode
swadgeMode musicMode =
{
    .modeName = "music",
    .fnEnterMode = musicEnterMode,
    .fnExitMode = musicExitMode,
    .fnButtonCallback = musicButtonCallback,
    .wifiMode = NO_WIFI,
    .fnEspNowRecvCb = NULL,
    .fnEspNowSendCb = NULL,
    .fnAccelerometerCallback = musicAccelerometerHandler,
    .menuImageData = mnu_music_0,
    .menuImageLen = sizeof(mnu_music_0)
};

// The state data
struct
{
    // Track motion
    int16_t roll;
    int16_t pitch;

    // Track rhythm
    os_timer_t beatTimer;
    uint32_t timeUs;
    uint32_t lastCallTimeUs;
    int16_t rhythmNoteIdx;
    uint8_t bpmIdx;

    // Track the button
    bool shouldPlay;
    uint8_t rhythmIdx;
    uint8_t scaleIdx;
    bool modifyBpm;
    os_timer_t paramSwitchTimer;
} music;

/*==============================================================================
 * Const Variables
 *============================================================================*/

// All the scales
#define LOWER_OCTAVE
#ifdef LOWER_OCTAVE
const notePeriod_t scl_M_Penta[] = {C_4, D_4, E_4, G_4, A_4, C_5, C_5, D_5, E_5, G_5, A_5, C_6, };
const notePeriod_t scl_m_Penta[] = {C_4, D_SHARP_4, F_4, G_4, A_SHARP_4, C_5, C_5, D_SHARP_5, F_5, G_5, A_SHARP_5, C_6, };
const notePeriod_t scl_m_Blues[] = {C_4, D_SHARP_4, F_4, F_SHARP_4, G_4, A_SHARP_4, C_5, C_5, D_SHARP_5, F_5, F_SHARP_5, G_5, A_SHARP_5, C_6, };
const notePeriod_t scl_M_Blues[] = {C_4, D_4, D_SHARP_4, E_4, G_4, A_4, C_5, C_5, D_5, D_SHARP_5, E_5, G_5, A_5, C_6, };
const notePeriod_t scl_Major[] = {C_4, D_4, E_4, F_4, G_4, A_4, B_4, C_5, C_5, D_5, E_5, F_5, G_5, A_5, B_5, C_6, };
const notePeriod_t scl_Minor_Aeolian[] = {C_4, D_4, D_SHARP_4, F_4, G_4, G_SHARP_4, A_SHARP_4, C_5, C_5, D_5, D_SHARP_5, F_5, G_5, G_SHARP_5, A_SHARP_5, C_6, };
const notePeriod_t scl_Harm_Minor[] = {C_4, D_4, D_SHARP_4, F_4, G_4, G_SHARP_4, B_4, C_5, C_5, D_5, D_SHARP_5, F_5, G_5, G_SHARP_5, B_5, C_6, };
const notePeriod_t scl_Dorian[] = {C_4, D_4, D_SHARP_4, F_4, G_4, A_4, A_SHARP_4, C_5, C_5, D_5, D_SHARP_5, F_5, G_5, A_5, A_SHARP_5, C_6, };
const notePeriod_t scl_Phrygian[] = {C_4, C_SHARP_4, D_SHARP_4, F_4, G_4, G_SHARP_4, A_SHARP_4, C_5, C_5, C_SHARP_5, D_SHARP_5, F_5, G_5, G_SHARP_5, A_SHARP_5, C_6, };
const notePeriod_t scl_Lydian[] = {C_4, D_4, E_4, F_SHARP_4, G_4, A_4, B_4, C_5, C_5, D_5, E_5, F_SHARP_5, G_5, A_5, B_5, C_6, };
const notePeriod_t scl_Mixolydian[] = {C_4, D_4, E_4, F_4, G_4, A_4, A_SHARP_4, C_5, C_5, D_5, E_5, F_5, G_5, A_5, A_SHARP_5, C_6, };
const notePeriod_t scl_Locrian[] = {C_4, C_SHARP_4, D_SHARP_4, F_4, F_SHARP_4, G_SHARP_4, A_SHARP_4, C_5, C_5, C_SHARP_5, D_SHARP_5, F_5, F_SHARP_5, G_SHARP_5, A_SHARP_5, C_6, };
const notePeriod_t scl_Dom_Bebop[] = {C_4, D_4, E_4, F_4, G_4, A_4, A_SHARP_4, B_4, C_5, C_5, D_5, E_5, F_5, G_5, A_5, A_SHARP_5, B_5, C_6, };
const notePeriod_t scl_M_Bebop[] = {C_4, D_4, E_4, F_4, G_4, G_SHARP_4, A_SHARP_4, B_4, C_5, C_5, D_5, E_5, F_5, G_5, G_SHARP_5, A_SHARP_5, B_5, C_6, };
const notePeriod_t scl_Whole_Tone[] = {C_4, D_4, E_4, F_SHARP_4, G_SHARP_4, A_SHARP_4, C_5, C_5, D_5, E_5, F_SHARP_5, G_SHARP_5, A_SHARP_5, C_6, };
const notePeriod_t scl_Dacs[] = {C_4, D_SHARP_4, F_4, F_SHARP_4, G_4, A_4, A_SHARP_4, C_5, D_SHARP_5, F_5, F_SHARP_5, G_5, A_5, A_SHARP_5, C_6};
const notePeriod_t scl_Chromatic[] = {C_4, C_SHARP_4, D_4, D_SHARP_4, E_4, F_4, F_SHARP_4, G_4, G_SHARP_4, A_4, A_SHARP_4, B_4, C_5, C_5, C_SHARP_5, D_5, D_SHARP_5, E_5, F_5, F_SHARP_5, G_5, G_SHARP_5, A_5, A_SHARP_5, B_5, C_6, };
#else
const notePeriod_t scl_M_Penta[] = {C_5, D_5, E_5, G_5, A_5, C_6, C_6, D_6, E_6, G_6, A_6, C_7, };
const notePeriod_t scl_m_Penta[] = {C_5, D_SHARP_5, F_5, G_5, A_SHARP_5, C_6, C_6, D_SHARP_6, F_6, G_6, A_SHARP_6, C_7, };
const notePeriod_t scl_m_Blues[] = {C_5, D_SHARP_5, F_5, F_SHARP_5, G_5, A_SHARP_5, C_6, C_6, D_SHARP_6, F_6, F_SHARP_6, G_6, A_SHARP_6, C_7, };
const notePeriod_t scl_M_Blues[] = {C_5, D_5, D_SHARP_5, E_5, G_5, A_5, C_6, C_6, D_6, D_SHARP_6, E_6, G_6, A_6, C_7, };
const notePeriod_t scl_Major[] = {C_5, D_5, E_5, F_5, G_5, A_5, B_5, C_6, C_6, D_6, E_6, F_6, G_6, A_6, B_6, C_7, };
const notePeriod_t scl_Minor_Aeolian[] = {C_5, D_5, D_SHARP_5, F_5, G_5, G_SHARP_5, A_SHARP_5, C_6, C_6, D_6, D_SHARP_6, F_6, G_6, G_SHARP_6, A_SHARP_6, C_7, };
const notePeriod_t scl_Harm_Minor[] = {C_5, D_5, D_SHARP_5, F_5, G_5, G_SHARP_5, B_5, C_6, C_6, D_6, D_SHARP_6, F_6, G_6, G_SHARP_6, B_6, C_7, };
const notePeriod_t scl_Dorian[] = {C_5, D_5, D_SHARP_5, F_5, G_5, A_5, A_SHARP_5, C_6, C_6, D_6, D_SHARP_6, F_6, G_6, A_6, A_SHARP_6, C_7, };
const notePeriod_t scl_Phrygian[] = {C_5, C_SHARP_5, D_SHARP_5, F_5, G_5, G_SHARP_5, A_SHARP_5, C_6, C_6, C_SHARP_6, D_SHARP_6, F_6, G_6, G_SHARP_6, A_SHARP_6, C_7, };
const notePeriod_t scl_Lydian[] = {C_5, D_5, E_5, F_SHARP_5, G_5, A_5, B_5, C_6, C_6, D_6, E_6, F_SHARP_6, G_6, A_6, B_6, C_7, };
const notePeriod_t scl_Mixolydian[] = {C_5, D_5, E_5, F_5, G_5, A_5, A_SHARP_5, C_6, C_6, D_6, E_6, F_6, G_6, A_6, A_SHARP_6, C_7, };
const notePeriod_t scl_Locrian[] = {C_5, C_SHARP_5, D_SHARP_5, F_5, F_SHARP_5, G_SHARP_5, A_SHARP_5, C_6, C_6, C_SHARP_6, D_SHARP_6, F_6, F_SHARP_6, G_SHARP_6, A_SHARP_6, C_7, };
const notePeriod_t scl_Dom_Bebop[] = {C_5, D_5, E_5, F_5, G_5, A_5, A_SHARP_5, B_5, C_6, C_6, D_6, E_6, F_6, G_6, A_6, A_SHARP_6, B_6, C_7, };
const notePeriod_t scl_M_Bebop[] = {C_5, D_5, E_5, F_5, G_5, G_SHARP_5, A_SHARP_5, B_5, C_6, C_6, D_6, E_6, F_6, G_6, G_SHARP_6, A_SHARP_6, B_6, C_7, };
const notePeriod_t scl_Whole_Tone[] = {C_5, D_5, E_5, F_SHARP_5, G_SHARP_5, A_SHARP_5, C_6, C_6, D_6, E_6, F_SHARP_6, G_SHARP_6, A_SHARP_6, C_7, };
const notePeriod_t scl_Dacs[] = {C_5, D_SHARP_5, F_5, F_SHARP_5, G_5, A_5, A_SHARP_5, C_6, D_SHARP_6, F_6, F_SHARP_6, G_6, A_6, A_SHARP_6, C_7};
const notePeriod_t scl_Chromatic[] = {C_5, C_SHARP_5, D_5, D_SHARP_5, E_5, F_5, F_SHARP_5, G_5, G_SHARP_5, A_5, A_SHARP_5, B_5, C_6, C_6, C_SHARP_6, D_6, D_SHARP_6, E_6, F_6, F_SHARP_6, G_6, G_SHARP_6, A_6, A_SHARP_6, B_6, C_7, };
#endif

const scale_t scales[] =
{
    {
        .name = "Ma Pent",
        .notes = scl_M_Penta,
        .notesLen = lengthof(scl_M_Penta)
    },
    {
        .name = "mi Pent",
        .notes = scl_m_Penta,
        .notesLen = lengthof(scl_m_Penta)
    },
    {
        .name = "Ma Blu",
        .notes = scl_M_Blues,
        .notesLen = lengthof(scl_M_Blues)
    },
    {
        .name = "mi Blu",
        .notes = scl_m_Blues,
        .notesLen = lengthof(scl_m_Blues)
    },
    {
        .name = "Major",
        .notes = scl_Major,
        .notesLen = lengthof(scl_Major)
    },
    {
        .name = "Minor",
        .notes = scl_Minor_Aeolian,
        .notesLen = lengthof(scl_Minor_Aeolian)
    },
    {
        .name = "Harm Minor",
        .notes = scl_Harm_Minor,
        .notesLen = lengthof(scl_Harm_Minor)
    },
    {
        .name = "Dorian",
        .notes = scl_Dorian,
        .notesLen = lengthof(scl_Dorian)
    },
    {
        .name = "Phrygian",
        .notes = scl_Phrygian,
        .notesLen = lengthof(scl_Phrygian)
    },
    {
        .name = "Lydian",
        .notes = scl_Lydian,
        .notesLen = lengthof(scl_Lydian)
    },
    {
        .name = "Mixolydian",
        .notes = scl_Mixolydian,
        .notesLen = lengthof(scl_Mixolydian)
    },
    {
        .name = "Locrian",
        .notes = scl_Locrian,
        .notesLen = lengthof(scl_Locrian)
    },
    {
        .name = "Dom Bebop",
        .notes = scl_Dom_Bebop,
        .notesLen = lengthof(scl_Dom_Bebop)
    },
    {
        .name = "Ma Bebop",
        .notes = scl_M_Bebop,
        .notesLen = lengthof(scl_M_Bebop)
    },
    {
        .name = "Whole Tone",
        .notes = scl_Whole_Tone,
        .notesLen = lengthof(scl_Whole_Tone)
    },
    {
        .name = "DACs",
        .notes = scl_Dacs,
        .notesLen = lengthof(scl_Dacs)
    },
    {
        .name = "Chromatic",
        .notes = scl_Chromatic,
        .notesLen = lengthof(scl_Chromatic)
    },
};

// All the rhythms
const bpm_t bpms[] =
{
    {.bpmMultiplier = 10, .bpm = 250},
    {.bpmMultiplier = 11, .bpm = 227},
    {.bpmMultiplier = 13, .bpm = 192},
    {.bpmMultiplier = 15, .bpm = 167},
    {.bpmMultiplier = 18, .bpm = 139},
    {.bpmMultiplier = 22, .bpm = 114},
    {.bpmMultiplier = 29, .bpm = 86},
    {.bpmMultiplier = 41, .bpm = 61},
};

const rhythmArp_t constant[] =
{
    {.note = TRIPLET_SIXTYFOURTH_NOTE, .arp = 1},
};

const rhythmArp_t one_note[] =
{
    {.note = EIGHTH_NOTE, .arp = 1},
};

const rhythmArp_t octaves[] =
{
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 13},
};

const rhythmArp_t fifth[] =
{
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 8},
};

const rhythmArp_t major_tri[] =
{
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 5},
    {.note = EIGHTH_NOTE, .arp = 8},
};

const rhythmArp_t minor_tri[] =
{
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 4},
    {.note = EIGHTH_NOTE, .arp = 8},
};

const rhythmArp_t major_7[] =
{
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 1},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 5},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 8},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 11},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 8},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 5},
};

const rhythmArp_t minor_7[] =
{
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 1},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 4},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 8},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 10},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 8},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 4},
};

const rhythmArp_t dom_7[] =
{
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 1},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 5},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 8},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 10},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 8},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 4},
};

const rhythmArp_t swing[] =
{
    {.note = TRIPLET_QUARTER_NOTE, .arp = 1},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 1},
};

const rhythmArp_t syncopa[] =
{
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_REST, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_REST, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_REST, .arp = 1},
};

const rhythmArp_t dw_stabs[] =
{
    {.note = SIXTEENTH_REST, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_REST, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 1},
};

const rhythmArp_t legendary[] =
{
    {.note = QUARTER_NOTE, .arp = 1},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 1},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 1},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 1},
    {.note = QUARTER_NOTE, .arp = 1},
    {.note = QUARTER_NOTE, .arp = 1},
};

const rhythmArp_t j_dawg[] =
{
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_REST, .arp = 1},
    {.note = EIGHTH_REST, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_REST, .arp = 1},
    {.note = EIGHTH_REST, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_REST, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_REST, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
};

const rhythmArp_t fight[] =
{
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
};

const rhythmArp_t the_goat[] =
{
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_REST, .arp = 1},
    {.note = DOTTED_EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
};

const rhythmArp_t sgp[] =
{
    {.note = DOTTED_EIGHTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = DOTTED_QUARTER_NOTE, .arp = 1},
    {.note = DOTTED_EIGHTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
};

const rhythmArp_t fourth_rock[] =
{
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 1},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 1},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 1},
    {.note = QUARTER_NOTE, .arp = 1},
    {.note = QUARTER_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = QUARTER_NOTE, .arp = 1},
};

const rhythmArp_t dub[] =
{
    {.note = QUARTER_NOTE, .arp = 1},
    {.note = QUARTER_NOTE, .arp = 1},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 13},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 13},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 13},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 13},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 13},
    {.note = TRIPLET_EIGHTH_NOTE, .arp = 13},
};

const rhythmArp_t octavio[] =
{
    {.note = EIGHTH_NOTE, .arp = 13},
    {.note = EIGHTH_NOTE, .arp = 13},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 13},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 8},
    {.note = EIGHTH_NOTE, .arp = 1},
};

const rhythmArp_t cha_cha[] =
{
    {.note = QUARTER_NOTE, .arp = 8},
    {.note = EIGHTH_NOTE, .arp = 4},
    {.note = QUARTER_NOTE, .arp = 9},
    {.note = QUARTER_NOTE, .arp = 4},
    {.note = QUARTER_NOTE, .arp = 8},
    {.note = QUARTER_NOTE, .arp = 4},
    {.note = EIGHTH_NOTE, .arp = 9},
    {.note = QUARTER_NOTE, .arp = 9},
    {.note = QUARTER_NOTE, .arp = 4},
};

const rhythmArp_t its_a_me[] =
{
    {.note = EIGHTH_NOTE, .arp = 5},
    {.note = EIGHTH_NOTE, .arp = 5},
    {.note = EIGHTH_REST, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 5},
    {.note = EIGHTH_REST, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 5},
    {.note = EIGHTH_REST, .arp = 1},
    {.note = QUARTER_NOTE, .arp = 8},
    {.note = QUARTER_REST, .arp = 1},
    {.note = QUARTER_NOTE, .arp = -6},
    {.note = QUARTER_REST, .arp = 1},
};

const rhythmArp_t so_strange[] =
{
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 5},
    {.note = SIXTEENTH_NOTE, .arp = 8},
    {.note = SIXTEENTH_NOTE, .arp = 12},
    {.note = SIXTEENTH_NOTE, .arp = 13},
    {.note = SIXTEENTH_NOTE, .arp = 12},
    {.note = SIXTEENTH_NOTE, .arp = 8},
    {.note = SIXTEENTH_NOTE, .arp = 5},
};

const rhythmArp_t sans[] =
{
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 13},
    {.note = SIXTEENTH_REST, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 8},
    {.note = SIXTEENTH_REST, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 7},
    {.note = EIGHTH_NOTE, .arp = 6},
    {.note = EIGHTH_NOTE, .arp = 4},
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 4},
    {.note = SIXTEENTH_NOTE, .arp = 6},
};

const rhythmArp_t fifteen_sixteen[] =
{
    {.note = SIXTEENTH_NOTE, .arp = 1},
    {.note = SIXTEENTH_NOTE, .arp = 8},
    {.note = SIXTEENTH_NOTE, .arp = 13},
    {.note = SIXTEENTH_NOTE, .arp = 15},
    {.note = SIXTEENTH_NOTE, .arp = 8},
    {.note = SIXTEENTH_NOTE, .arp = 13},
    {.note = SIXTEENTH_NOTE, .arp = 15},
    {.note = SIXTEENTH_NOTE, .arp = 18},
    {.note = SIXTEENTH_NOTE, .arp = 8},
    {.note = SIXTEENTH_NOTE, .arp = 18},
    {.note = SIXTEENTH_NOTE, .arp = 17},
    {.note = SIXTEENTH_NOTE, .arp = 8},
    {.note = SIXTEENTH_NOTE, .arp = 17},
    {.note = SIXTEENTH_NOTE, .arp = 15},
    {.note = SIXTEENTH_NOTE, .arp = 13},
};

const rhythmArp_t sb[] =
{
    {.note = EIGHTH_REST, .arp = 1},
    {.note = EIGHTH_NOTE, .arp = 12},
    {.note = DOTTED_EIGHTH_NOTE, .arp = 13},
    {.note = DOTTED_EIGHTH_NOTE, .arp = 12},
    {.note = EIGHTH_NOTE, .arp = 13},
    {.note = DOTTED_EIGHTH_NOTE, .arp = 8},
    {.note = SIXTEENTH_REST, .arp = 1},
};

const rhythm_t rhythms[] =
{
    {
        .name = "Slide",
        .rhythm = constant,
        .rhythmLen = lengthof(constant),
        .interNotePauseMs = 0
    },
    {
        .name = "Qrtr",
        .rhythm = one_note,
        .rhythmLen = lengthof(one_note),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "octave",
        .rhythm = octaves,
        .rhythmLen = lengthof(octaves),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "fifth",
        .rhythm = fifth,
        .rhythmLen = lengthof(fifth),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "Maj 3",
        .rhythm = major_tri,
        .rhythmLen = lengthof(major_tri),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "Min 3",
        .rhythm = minor_tri,
        .rhythmLen = lengthof(minor_tri),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "Maj 7",
        .rhythm = major_7,
        .rhythmLen = lengthof(major_7),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "Min 7",
        .rhythm = minor_7,
        .rhythmLen = lengthof(minor_7),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "Dom 7",
        .rhythm = dom_7,
        .rhythmLen = lengthof(dom_7),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "swing",
        .rhythm = swing,
        .rhythmLen = lengthof(swing),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "syncop",
        .rhythm = syncopa,
        .rhythmLen = lengthof(syncopa),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "dw_stabs",
        .rhythm = dw_stabs,
        .rhythmLen = lengthof(dw_stabs),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "lgnd",
        .rhythm = legendary,
        .rhythmLen = lengthof(legendary),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "j-dawg",
        .rhythm = j_dawg,
        .rhythmLen = lengthof(j_dawg),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "fight!",
        .rhythm = fight,
        .rhythmLen = lengthof(fight),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "goat",
        .rhythm = the_goat,
        .rhythmLen = lengthof(the_goat),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "sgp",
        .rhythm = sgp,
        .rhythmLen = lengthof(sgp),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "Mars",
        .rhythm = fourth_rock,
        .rhythmLen = lengthof(fourth_rock),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "dub",
        .rhythm = dub,
        .rhythmLen = lengthof(dub),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "octavio",
        .rhythm = octavio,
        .rhythmLen = lengthof(octavio),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "chacha",
        .rhythm = cha_cha,
        .rhythmLen = lengthof(cha_cha),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "its-a-me",
        .rhythm = its_a_me,
        .rhythmLen = lengthof(its_a_me),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "strange",
        .rhythm = so_strange,
        .rhythmLen = lengthof(so_strange),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "sans?",
        .rhythm = sans,
        .rhythmLen = lengthof(sans),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "15/16",
        .rhythm = fifteen_sixteen,
        .rhythmLen = lengthof(fifteen_sixteen),
        .interNotePauseMs = DEFAULT_PAUSE
    },
    {
        .name = "sb",
        .rhythm = sb,
        .rhythmLen = lengthof(sb),
        .interNotePauseMs = DEFAULT_PAUSE
    }
};

const notePeriod_t allNotes[] =
{
    C_2,
    C_SHARP_2,
    D_2,
    D_SHARP_2,
    E_2,
    F_2,
    F_SHARP_2,
    G_2,
    G_SHARP_2,
    A_2,
    A_SHARP_2,
    B_2,

    C_3,
    C_SHARP_3,
    D_3,
    D_SHARP_3,
    E_3,
    F_3,
    F_SHARP_3,
    G_3,
    G_SHARP_3,
    A_3,
    A_SHARP_3,
    B_3,

    C_4,
    C_SHARP_4,
    D_4,
    D_SHARP_4,
    E_4,
    F_4,
    F_SHARP_4,
    G_4,
    G_SHARP_4,
    A_4,
    A_SHARP_4,
    B_4,

    C_5,
    C_SHARP_5,
    D_5,
    D_SHARP_5,
    E_5,
    F_5,
    F_SHARP_5,
    G_5,
    G_SHARP_5,
    A_5,
    A_SHARP_5,
    B_5,

    C_6,
    C_SHARP_6,
    D_6,
    D_SHARP_6,
    E_6,
    F_6,
    F_SHARP_6,
    G_6,
    G_SHARP_6,
    A_6,
    A_SHARP_6,
    B_6,

    C_7,
    C_SHARP_7,
    D_7,
    D_SHARP_7,
    E_7,
    F_7,
    F_SHARP_7,
    G_7,
    G_SHARP_7,
    A_7,
    A_SHARP_7,
    B_7,

    C_8,
    C_SHARP_8,
    D_8,
    D_SHARP_8,
    E_8,
    F_8,
    F_SHARP_8,
    G_8,
    G_SHARP_8,
    A_8,
    A_SHARP_8,
    B_8,
};

/*============================================================================
 * Functions
 *==========================================================================*/

/**
 * Initializer for music
 */
void ICACHE_FLASH_ATTR musicEnterMode(void)
{
    // If the swadge is muted
    if(getIsMutedOption())
    {
        // Unmute it and init the buzzer
        setMuteOverride(true);
        initBuzzer();
        setBuzzerNote(SILENCE);
    }

    // Clear everything
    memset(&music, 0, sizeof(music));

    // Set default BPM to 114
    music.bpmIdx = 5;

    // Set a timer to tick every 1ms, forever
    os_timer_disarm(&music.beatTimer);
    os_timer_setfn(&music.beatTimer, musicBeatTimerFunc, NULL);
    os_timer_arm(&music.beatTimer, 1, true);

    // Draw an initial display
    musicUpdateDisplay();

    // Request to do everything faster
    setAccelPollTime(50);
    setOledDrawTime(50);
    enableDebounce(false);

    // Set up a timer to handle the parameter button
    os_timer_disarm(&music.paramSwitchTimer);
    os_timer_setfn(&music.paramSwitchTimer, paramSwitchTimerFunc, NULL);
}

/**
 * Called when music is exited
 */
void ICACHE_FLASH_ATTR musicExitMode(void)
{
    os_timer_disarm(&music.beatTimer);
}

/**
 * @brief Button callback function, plays notes and switches parameters
 *
 * @param state  A bitmask of all button states, unused
 * @param button The button which triggered this event
 * @param down   true if the button was pressed, false if it was released
 */
void ICACHE_FLASH_ATTR musicButtonCallback(
    uint8_t state __attribute__((unused)), int button, int down)
{
    switch(button)
    {
        case 0:
        {
            // Center
            if(down)
            {
                // Cycle the scale
                music.scaleIdx = (music.scaleIdx + 1) % lengthof(scales);
            }
            break;
        }
        case 1:
        {
            // Left
            if(down)
            {
                // Start a timer to either switch the mode or change the BPM
                os_timer_arm(&music.paramSwitchTimer, 1000, false);
                music.modifyBpm = false;
            }
            else // Released
            {
                // Stop the timer no matter what
                os_timer_disarm(&music.paramSwitchTimer);

                // Button released while timer is still active, switch the mode
                if(false == music.modifyBpm)
                {
                    // cycle params
                    music.rhythmIdx = (music.rhythmIdx + 1) % lengthof(rhythms);
                    music.timeUs = 0;
                    music.rhythmNoteIdx = -1;
                    music.lastCallTimeUs = 0;
                    musicUpdateDisplay();
                }
                // Clear this flag on release, always
                music.modifyBpm = false;
            }

            break;
        }
        case 2:
        {
            // Right, track whether a note should be played or not
            if(false == music.modifyBpm)
            {
                music.rhythmNoteIdx = -1;
                music.lastCallTimeUs = 0;
                music.shouldPlay = down;
            }
            else
            {
                music.shouldPlay = false;
            }

            break;
        }
        default:
        {
            break;
        }
    }
}

/**
 * Timer started when the 'choose' button is pressed.
 * If it expires before the button is released, set the flag to modify BPM
 * If it doesn't expire before the button is released, switch the mode params
 *
 * @param arg unused
 */
void ICACHE_FLASH_ATTR paramSwitchTimerFunc(void* arg __attribute__((unused)))
{
    music.modifyBpm = true;
    music.shouldPlay = false;
}

/**
 * @brief Callback function for accelerometer values
 * Use the current vector to find pitch and roll, then update the display
 *
 * @param accel The freshly read accelerometer values
 */
void ICACHE_FLASH_ATTR musicAccelerometerHandler(accel_t* accel)
{
    // Only find values when the swadge is pointed up
    if(accel-> x < 0)
    {
        return;
    }

    // Find the roll and pitch in radians
    float rollF = atanf(accel->y / (float)accel->x);
    float pitchF = atanf((-1 * accel->z) / sqrtf((accel->y * accel->y) + (accel->x * accel->x)));

    // Normalize the values to [0,1]
    rollF = ((rollF) / M_PI) + 0.5f;
    pitchF = ((pitchF) / M_PI) + 0.5f;

    // Round and scale to OLED_WIDTH
    // this maps 30 degrees to the far left and 150 degrees to the far right
    // (30 / 180) == 0.167, (180 - (2 * 30)) / 180 == 0.666
    music.roll = roundf(((rollF - 0.167f) * OLED_WIDTH) / 0.666f);
    if(music.roll >= OLED_WIDTH)
    {
        music.roll = OLED_WIDTH - 1;
    }
    else if(music.roll < 0)
    {
        music.roll = 0;
    }
    music.pitch = roundf(pitchF * OLED_WIDTH);
    if(music.pitch >= OLED_WIDTH)
    {
        music.pitch = OLED_WIDTH - 1;
    }

    // Check if the BPM should be adjusted
    if(true == music.modifyBpm)
    {
        // Get the number of BPMs
        uint8_t numBpms = lengthof(bpms);
        // Scale the roll to the BPM range and reverse it
        music.bpmIdx = (int)(rollF * numBpms);
        if(music.bpmIdx >= numBpms)
        {
            music.bpmIdx = numBpms - 1;
        }
        music.bpmIdx = numBpms - music.bpmIdx - 1;
    }

    // os_printf("roll %6d pitch %6d, x %4d, y %4d, z %4d, \n",
    // music.roll, music.pitch,
    // accel->x, accel->y, accel->z);

    musicUpdateDisplay();
}

/**
 * Update the display by drawing the current state of affairs
 */
void ICACHE_FLASH_ATTR musicUpdateDisplay(void)
{
    clearDisplay();

    // Plot the bars
    plotBar(OLED_HEIGHT - BAR_Y_MARGIN - 1);
    plotBar(OLED_HEIGHT - BAR_Y_MARGIN - (2 * CURSOR_HEIGHT + 5));

    // Draw the cursor if the BPM isn't being modified
    if(false == music.modifyBpm)
    {
        if(music.pitch < PITCH_THRESHOLD)
        {
            // Plot the cursor
            plotLine(music.roll, OLED_HEIGHT - BAR_Y_MARGIN - (2 * CURSOR_HEIGHT + 5) - CURSOR_HEIGHT,
                     music.roll, OLED_HEIGHT - BAR_Y_MARGIN - (2 * CURSOR_HEIGHT + 5) + CURSOR_HEIGHT,
                     WHITE);
        }
        else
        {
            // Plot the cursor
            plotLine(music.roll, OLED_HEIGHT - BAR_Y_MARGIN - 1 - CURSOR_HEIGHT,
                     music.roll, OLED_HEIGHT - BAR_Y_MARGIN - 1 + CURSOR_HEIGHT,
                     WHITE);
        }
    }

    // Plot the title
    plotText(
        OLED_WIDTH - getTextWidth(scales[music.scaleIdx].name, IBM_VGA_8),
        0,
        scales[music.scaleIdx].name,
        IBM_VGA_8, WHITE);
    plotText(
        0,
        0,
        rhythms[music.rhythmIdx].name,
        IBM_VGA_8, WHITE);

    // Plot the BPM
    char bpmStr[8] = {0};
    ets_snprintf(bpmStr, sizeof(bpmStr), "%d", bpms[music.bpmIdx].bpm);
    plotText(0, FONT_HEIGHT_IBMVGA8 + 4, bpmStr, IBM_VGA_8, WHITE);

    // Underline it if it's being modified
    if(true == music.modifyBpm)
    {
        for(uint8_t i = 2; i < 4; i++)
        {
            plotLine(
                0,
                (2 * FONT_HEIGHT_RADIOSTARS) + 4 + i,
                getTextWidth(bpmStr, IBM_VGA_8),
                (2 * FONT_HEIGHT_RADIOSTARS) + 4 + i,
                WHITE);
        }
    }

    // Plot the note if BPM isn't being modified
    if(false == music.modifyBpm)
    {
        plotCenteredText(0, 20, OLED_WIDTH - 1, noteToStr(getCurrentNote()), RADIOSTARS, WHITE);
    }

    // Plot the button funcs
    plotText(0, OLED_HEIGHT - FONT_HEIGHT_TOMTHUMB, "Rhythm (BPM)", TOM_THUMB, WHITE);
    plotCenteredText(0, OLED_HEIGHT - FONT_HEIGHT_TOMTHUMB, OLED_WIDTH, "Scale", TOM_THUMB, WHITE);
    plotText(OLED_WIDTH - getTextWidth("Play", TOM_THUMB), OLED_HEIGHT - FONT_HEIGHT_TOMTHUMB, "Play", TOM_THUMB, WHITE);
}

/**
 * @brief Plot a horizontal bar indicating where the note boundaries are
 *
 * @param yOffset The Y Offset of the middle of the bar, not the ticks
 */
void ICACHE_FLASH_ATTR plotBar(uint8_t yOffset)
{
    // Plot the main bar
    plotLine(
        BAR_X_MARGIN,
        yOffset,
        OLED_WIDTH - BAR_X_MARGIN,
        yOffset,
        WHITE);

    // Plot tick marks at each of the note boundaries
    uint8_t tick;
    for(tick = 0; tick < (scales[music.scaleIdx].notesLen / 2) + 1; tick++)
    {
        uint8_t x = BAR_X_MARGIN + ( (tick * ((OLED_WIDTH - 1) - (BAR_X_MARGIN * 2))) /
                                     (scales[music.scaleIdx].notesLen / 2)) ;
        plotLine(x, yOffset - TICK_HEIGHT,
                 x, yOffset + TICK_HEIGHT,
                 WHITE);
    }
}

/**
 * @brief Called every 1ms to handle the rhythm, arpeggios, and setting the buzzer
 *
 * @param arg unused
 */
void ICACHE_FLASH_ATTR musicBeatTimerFunc(void* arg __attribute__((unused)))
{
    // Keep track of time with microsecond precision
    int32_t currentCallTimeUs = system_get_time();

    // There's some special handling if we're playing the first note of a rhythm
    bool playFirstNote = false;
    if(0 == music.lastCallTimeUs)
    {
        // Initialize lastCallTimeUs, play the first note
        music.lastCallTimeUs = currentCallTimeUs;
        playFirstNote = true;
    }
    else
    {
        // Figure out the delta between calls in microseconds, increment time
        music.timeUs += (currentCallTimeUs - music.lastCallTimeUs);
        music.lastCallTimeUs = currentCallTimeUs;
    }

    // If time crossed a rhythm boundary, or is the first note, do something different
    uint32_t rhythmIntervalUs = (1000 * bpms[music.bpmIdx].bpmMultiplier * ((~REST_BIT) &
                                 rhythms[music.rhythmIdx].rhythm[music.rhythmNoteIdx].note));
    led_t leds[NUM_LIN_LEDS] = {{0}};
    if(music.timeUs >= rhythmIntervalUs || true == playFirstNote)
    {
        if(music.timeUs >= rhythmIntervalUs)
        {
            // Reset the time, but not for the first note
            music.timeUs -= rhythmIntervalUs;
        }
        // Move to the next rhythm element
        music.rhythmNoteIdx = (music.rhythmNoteIdx + 1) % rhythms[music.rhythmIdx].rhythmLen;

        // See if we should actually play the note or not
        if(!music.shouldPlay || (rhythms[music.rhythmIdx].rhythm[music.rhythmNoteIdx].note & REST_BIT))
        {
            setBuzzerNote(SILENCE);
            noteToColor(&leds[0], getCurrentNote(), 0x10);
        }
        else
        {
            // Only play the note if BPM isn't being modified
            if(false == music.modifyBpm)
            {
                // Arpeggiate as necessary
                setBuzzerNote(arpModify(getCurrentNote(),
                                        rhythms[music.rhythmIdx].rhythm[music.rhythmNoteIdx].arp));
            }
            noteToColor(&leds[0], getCurrentNote(), 0x40);
        }
    }
    else if(music.timeUs >= rhythmIntervalUs - (1000 * rhythms[music.rhythmIdx].interNotePauseMs))
    {
        setBuzzerNote(SILENCE);
        noteToColor(&leds[0], getCurrentNote(), 0x10);
    }
    else
    {
        // Don't set LEDs if nothing changed
        return;
    }

    // Copy LED color from the first LED to all of them
    for(uint8_t ledIdx = 1; ledIdx < NUM_LIN_LEDS; ledIdx++)
    {
        leds[ledIdx].r = leds[0].r;
        leds[ledIdx].g = leds[0].g;
        leds[ledIdx].b = leds[0].b;
    }
    setLeds(leds, sizeof(leds));
}

/**
 * @return the current note the angle coresponds to. doesn't matter if it should
 * be played right now or not
 */
notePeriod_t ICACHE_FLASH_ATTR getCurrentNote(void)
{
    // Get the index of the note to play based on roll
    uint8_t noteIdx = (music.roll * (scales[music.scaleIdx].notesLen / 2)) / OLED_WIDTH;
    // See if we should play the higher note
    if(music.pitch < PITCH_THRESHOLD)
    {
        uint8_t offset = scales[music.scaleIdx].notesLen / 2;
        return scales[music.scaleIdx].notes[noteIdx + offset];
    }
    else
    {
        return scales[music.scaleIdx].notes[noteIdx];
    }
}

/**
 * @brief Arpeggiate a note
 *
 * @param note The root note to arpeggiate
 * @param arpInterval The interval to arpeggiate it by
 * @return notePeriod_t The arpeggiated note
 */
notePeriod_t ICACHE_FLASH_ATTR arpModify(notePeriod_t note, int8_t arpInterval)
{
    // Don't need to do anything for these
    if(1 == arpInterval || 0 == arpInterval)
    {
        return note;
    }

    // First find the note in the list of all notes
    for(uint16_t i = 0; i < lengthof(allNotes); i++)
    {
        if(note == allNotes[i])
        {
            if(arpInterval < 0)
            {
                // Then shift down by arpInterval
                while(++arpInterval)
                {
                    i--;
                }
            }
            else if(arpInterval > 0)
            {
                // Then shift up by arpInterval
                while(--arpInterval)
                {
                    i++;
                }
            }

            // Then return the arpeggiated note
            return allNotes[i];
        }
    }
    return note;
}

/**
 * @brief Translate a musical note to a color
 *
 * @param led The led_t to write the color data to
 * @param note The note to translate to color
 * @param brightness The brightness of the LED
 */
void ICACHE_FLASH_ATTR noteToColor( led_t* led, notePeriod_t note, uint8_t brightness)
{
    // First find the note in the list of all notes
    for(uint16_t idx = 0; idx < lengthof(allNotes); idx++)
    {
        if(note == allNotes[idx])
        {
            idx = idx % 12;
            idx = (idx * 255) / 12;

            uint32_t col = EHSVtoHEX(idx, 0xFF, brightness);
            led->r = (col >> 16) & 0xFF;
            led->g = (col >>  8) & 0xFF;
            led->b = (col >>  0) & 0xFF;
            return;
        }
    }
}

/**
 * @brief Translate a musical note to a string. Only covers the notes we play
 *
 * @param note The note to translate to color
 * @return A null terminated string for this note
 */
char* ICACHE_FLASH_ATTR noteToStr(notePeriod_t note)
{
    switch(note)
    {
        case SILENCE:
        {
            return "SILENCE";
        }
        case C_0:
        {
            return "C0";
        }
        case C_SHARP_0:
        {
            return "C#0";
        }
        case D_0:
        {
            return "D0";
        }
        case D_SHARP_0:
        {
            return "D#0";
        }
        case E_0:
        {
            return "E0";
        }
        case F_0:
        {
            return "F0";
        }
        case F_SHARP_0:
        {
            return "F#0";
        }
        case G_0:
        {
            return "G0";
        }
        case G_SHARP_0:
        {
            return "G#0";
        }
        case A_0:
        {
            return "A0";
        }
        case A_SHARP_0:
        {
            return "A#0";
        }
        case B_0:
        {
            return "B0";
        }
        case C_1:
        {
            return "C1";
        }
        case C_SHARP_1:
        {
            return "C#1";
        }
        case D_1:
        {
            return "D1";
        }
        case D_SHARP_1:
        {
            return "D#1";
        }
        case E_1:
        {
            return "E1";
        }
        case F_1:
        {
            return "F1";
        }
        case F_SHARP_1:
        {
            return "F#1";
        }
        case G_1:
        {
            return "G1";
        }
        case G_SHARP_1:
        {
            return "G#1";
        }
        case A_1:
        {
            return "A1";
        }
        case A_SHARP_1:
        {
            return "A#1";
        }
        case B_1:
        {
            return "B1";
        }
        case C_2:
        {
            return "C2";
        }
        case C_SHARP_2:
        {
            return "C#2";
        }
        case D_2:
        {
            return "D2";
        }
        case D_SHARP_2:
        {
            return "D#2";
        }
        case E_2:
        {
            return "E2";
        }
        case F_2:
        {
            return "F2";
        }
        case F_SHARP_2:
        {
            return "F#2";
        }
        case G_2:
        {
            return "G2";
        }
        case G_SHARP_2:
        {
            return "G#2";
        }
        case A_2:
        {
            return "A2";
        }
        case A_SHARP_2:
        {
            return "A#2";
        }
        case B_2:
        {
            return "B2";
        }
        case C_3:
        {
            return "C3";
        }
        case C_SHARP_3:
        {
            return "C#3";
        }
        case D_3:
        {
            return "D3";
        }
        case D_SHARP_3:
        {
            return "D#3";
        }
        case E_3:
        {
            return "E3";
        }
        case F_3:
        {
            return "F3";
        }
        case F_SHARP_3:
        {
            return "F#3";
        }
        case G_3:
        {
            return "G3";
        }
        case G_SHARP_3:
        {
            return "G#3";
        }
        case A_3:
        {
            return "A3";
        }
        case A_SHARP_3:
        {
            return "A#3";
        }
        case B_3:
        {
            return "B3";
        }
        case C_4:
        {
            return "C4";
        }
        case C_SHARP_4:
        {
            return "C#4";
        }
        case D_4:
        {
            return "D4";
        }
        case D_SHARP_4:
        {
            return "D#4";
        }
        case E_4:
        {
            return "E4";
        }
        case F_4:
        {
            return "F4";
        }
        case F_SHARP_4:
        {
            return "F#4";
        }
        case G_4:
        {
            return "G4";
        }
        case G_SHARP_4:
        {
            return "G#4";
        }
        case A_4:
        {
            return "A4";
        }
        case A_SHARP_4:
        {
            return "A#4";
        }
        case B_4:
        {
            return "B4";
        }
        case C_5:
        {
            return "C5";
        }
        case C_SHARP_5:
        {
            return "C#5";
        }
        case D_5:
        {
            return "D5";
        }
        case D_SHARP_5:
        {
            return "D#5";
        }
        case E_5:
        {
            return "E5";
        }
        case F_5:
        {
            return "F5";
        }
        case F_SHARP_5:
        {
            return "F#5";
        }
        case G_5:
        {
            return "G5";
        }
        case G_SHARP_5:
        {
            return "G#5";
        }
        case A_5:
        {
            return "A5";
        }
        case A_SHARP_5:
        {
            return "A#5";
        }
        case B_5:
        {
            return "B5";
        }
        case C_6:
        {
            return "C6";
        }
        case C_SHARP_6:
        {
            return "C#6";
        }
        case D_6:
        {
            return "D6";
        }
        case D_SHARP_6:
        {
            return "D#6";
        }
        case E_6:
        {
            return "E6";
        }
        case F_6:
        {
            return "F6";
        }
        case F_SHARP_6:
        {
            return "F#6";
        }
        case G_6:
        {
            return "G6";
        }
        case G_SHARP_6:
        {
            return "G#6";
        }
        case A_6:
        {
            return "A6";
        }
        case A_SHARP_6:
        {
            return "A#6";
        }
        case B_6:
        {
            return "B6";
        }
        case C_7:
        {
            return "C7";
        }
        case C_SHARP_7:
        {
            return "C#7";
        }
        case D_7:
        {
            return "D7";
        }
        case D_SHARP_7:
        {
            return "D#7";
        }
        case E_7:
        {
            return "E7";
        }
        case F_7:
        {
            return "F7";
        }
        case F_SHARP_7:
        {
            return "F#7";
        }
        case G_7:
        {
            return "G7";
        }
        case G_SHARP_7:
        {
            return "G#7";
        }
        case A_7:
        {
            return "A7";
        }
        case A_SHARP_7:
        {
            return "A#7";
        }
        case B_7:
        {
            return "B7";
        }
        case C_8:
        {
            return "C8";
        }
        case C_SHARP_8:
        {
            return "C#8";
        }
        case D_8:
        {
            return "D8";
        }
        case D_SHARP_8:
        {
            return "D#8";
        }
        case E_8:
        {
            return "E8";
        }
        case F_8:
        {
            return "F8";
        }
        case F_SHARP_8:
        {
            return "F#8";
        }
        case G_8:
        {
            return "G8";
        }
        case G_SHARP_8:
        {
            return "G#8";
        }
        case A_8:
        {
            return "A8";
        }
        case A_SHARP_8:
        {
            return "A#8";
        }
        case B_8:
        {
            return "B8";
        }
        default:
        {
            return "";
        }
    }
}
