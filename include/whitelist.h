#pragma once
#include <Arduino.h>

struct WhitelistEntry {
  uint16_t id;
  const char* name;
};

// Deine erlaubten EC3000-Steckdosen mit Klarnamen
const WhitelistEntry whitelist[] = {
  { 0xBA3D, "Waschmaschine" },
  { 0xBF2C, "Trockner"      },
  { 0xBF3A, "Geschirrspueler"},
  { 0x926A, "Kuehlschrank"},
  { 0xC33A, "Gefrierschrank"},
  { 0xBDD2, "Geschirrspueler 2"},
  { 0xBE4A, "Kuehlschrank Keller"   },
  { 0xB7A1, ""},
  { 0xB7C3, ""}
};

const size_t whitelistSize = sizeof(whitelist) / sizeof(whitelist[0]);
