#ifndef _PCIEFD_CONFIG_H_
#define _PCIEFD_CONFIG_H_

// Please note that not using DMA is not officially supported, and may lead to
// unexpected behaviour!
#define USE_DMA               1
#define USE_BUS_ANALYZER      0
#define USE_PATTERN_GENERATOR 0
#define USE_ADJUSTABLE_PLL    0

// Use version 2 for now, later versions are needed for bus analyzer/pattern generator support
#define LOOPBACK_VERSION      2

#endif
