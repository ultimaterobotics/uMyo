#pragma once

// uMyo v3.1 hardware configuration.
//
// RGB LED pin mapping differs across PCB batches.
//
// Default (April 2025+ batch / "RGB_update" boards):
//   R = P0.06, G = P0.07, B = P0.08
//
// Older batches:
//   R = P0.08, G = P0.07, B = P0.06
//
// If you need the older mapping, change the three defines below (or pass -DUMYO_LED_PIN_*).

#ifndef UMYO_LED_PIN_R
#define UMYO_LED_PIN_R 6
#endif

#ifndef UMYO_LED_PIN_G
#define UMYO_LED_PIN_G 7
#endif

#ifndef UMYO_LED_PIN_B
#define UMYO_LED_PIN_B 8
#endif

#ifndef UMYO_LED_PIN_CORR
#define UMYO_LED_PIN_CORR 26
#endif