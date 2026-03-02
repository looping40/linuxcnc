#pragma once
#include <stdint.h>

// Contrat entre xhcEsp32 (ESP32) et openpendant (LinuxCNC).
// Ne jamais modifier les tailles sans mettre à jour les deux projets simultanément.

// ─── Bits de hal_status_bits ──────────────────────────────────────────────────
#define HAL_BIT_ESTOP       (1u << 0)
#define HAL_BIT_HOMED       (1u << 1)
#define HAL_BIT_MACHINE_ON  (1u << 2)
#define HAL_BIT_RUNNING     (1u << 3)
#define HAL_BIT_PAUSED      (1u << 4)
#define HAL_BIT_PROBING     (1u << 5)

// ─── LinuxCNC → ESP32 : OUTPUT report, Report ID 2, 64 octets ────────────────
typedef struct __attribute__((packed)) {
    uint8_t  report_id;         // Toujours 2
    float    dro_x;             // Position workspace X mm (IEEE 754)
    float    dro_y;             // Position workspace Y mm
    float    dro_z;             // Position workspace Z mm
    float    dro_a;             // Position workspace A mm/deg
    float    mach_x;            // Position machine X mm
    float    mach_y;            // Position machine Y mm
    float    mach_z;            // Position machine Z mm
    float    mach_a;            // Position machine A mm/deg
    int32_t  feed_rate;         // Vitesse d'avance effective (mm/min)
    int32_t  spindle_rpm;       // Vitesse broche effective (tr/min)
    uint16_t feed_ovr;          // Override avance (0–120 %)
    uint16_t spindle_ovr;       // Override broche (0–120 %)
    uint8_t  step_active;       // Index pas actif côté HAL (0–10, table STEP_MULT)
    uint32_t hal_status_bits;   // Voir HAL_BIT_* ci-dessus
    char     current_tool[14];  // Nom outil ex: "Fraise 6mm\0"
} pendant_tx_packet_t;
// static_assert(sizeof(pendant_tx_packet_t) == 64, "pendant_tx_packet_t must be 64 bytes");

// ─── Bits de btn_state ────────────────────────────────────────────────────────
#define BTN_START_PAUSE  (1u << 0)
#define BTN_GOTO_ZERO    (1u << 1)
#define BTN_REWIND       (1u << 2)
#define BTN_PROBE_Z      (1u << 3)
#define BTN_MACRO_3      (1u << 4)
#define BTN_HALF         (1u << 5)
#define BTN_ZERO         (1u << 6)
#define BTN_SAFE_Z       (1u << 7)
#define BTN_HOME         (1u << 8)
#define BTN_MACRO_1      (1u << 9)
#define BTN_MACRO_2      (1u << 10)
#define BTN_SPINDLE      (1u << 11)
#define BTN_STEP_UP      (1u << 12)
#define BTN_STEP_DOWN    (1u << 13)
#define BTN_MODE         (1u << 14)
#define BTN_STOP         (1u << 15)
#define BTN_RESET        (1u << 16)
#define BTN_ESTOP        (1u << 17)

// ─── Valeurs de wheel_mode ────────────────────────────────────────────────────
// Ces valeurs définissent le protocole V2.
// globals.h (wheel_mode_en) doit être mis à jour pour correspondre (Phase 2).
#define WHEEL_MODE_OFF         0x00u
#define WHEEL_MODE_X           0x11u
#define WHEEL_MODE_Y           0x12u
#define WHEEL_MODE_Z           0x13u
#define WHEEL_MODE_A           0x14u
#define WHEEL_MODE_ADJ_SPINDLE 0x15u
#define WHEEL_MODE_ADJ_FEED    0x16u

// ─── ESP32 → LinuxCNC : INPUT report, Report ID 1, 10 octets ─────────────────
typedef struct __attribute__((packed)) {
    uint8_t  report_id;         // Toujours 1
    uint32_t btn_state;         // Bitmask boutons (voir BTN_* ci-dessus)
    uint8_t  wheel_mode;        // Axe sélectionné (voir WHEEL_MODE_*)
    int16_t  wheel_abs;         // Position absolue roue ÷4 (immunité perte paquet)
    uint8_t  step_req;          // Index pas demandé par l'utilisateur (0–10)
    uint8_t  reserved;
} pendant_rx_packet_t;
// static_assert(sizeof(pendant_rx_packet_t) == 10, "pendant_rx_packet_t must be 10 bytes");
