/* Whitelist EC3000-IDs

#pragma once

const char* whitelist[] = {
  "BF8C",  // Trockner
  "BACD",  // Waschmaschine
  "BF3D",  // Geschirrspüler
  "BE7A",   // Kühlschrank
  "9168",
  "B7D1",
  "B7C3",
  "BDDF",
  "C13B"
};

const int whitelistSize = sizeof(whitelist) / sizeof(whitelist[0]);
*/

#pragma once
#include <Arduino.h>

struct WhitelistEntry {
  uint16_t id;
  const char* name;
};

// Deine erlaubten EC3000-Steckdosen mit Klarnamen
const WhitelistEntry whitelist[] = {
  { 0xBACD, "Waschmaschine" },
  { 0xBF8C, "Trockner"      },
  { 0xBF3D, "Geschirrspueler"},
  { 0x9168, "Kuehlschrank"},
  { 0xC13B, "Gefrierschrank"},
  { 0xBDDF, "Geschirrspueler Keller"},
  { 0xBE7A, "Kuehlschrank Keller"   },
  { 0xB7D1, ""},
  { 0xB7C3, ""}
};

const size_t whitelistSize = sizeof(whitelist) / sizeof(whitelist[0]);
