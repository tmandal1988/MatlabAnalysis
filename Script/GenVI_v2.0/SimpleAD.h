/******************************************************************************
 * Copyright 2012 NetBurner, Inc.  ALL RIGHTS RESERVED
 *   Permission is hereby granted to purchasers of NetBurner Hardware
 *   to use or modify this computer program for any use as long as the
 *   resultant program is only executed on NetBurner provided hardware.
 *
 *   No other rights to use this program or it's derivatives in part or
 *   in whole are granted.
 *
 *   It may be possible to license this or other NetBurner software for
 *   use on non-NetBurner Hardware.
 *   Please contact sales@Netburner.com for more information.
 *
 *   NetBurner makes no representation or warranties
 *   with respect to the performance of this computer program, and
 *   specifically disclaims any responsibility for any damages,
 *   special or consequential, connected with the use of this program.
 *
 *---------------------------------------------------------------------
 * NetBurner, Inc.
 * 5405 Morehouse Drive
 * San Diego, California 92121
 *
 * information available at:  http://www.netburner.com
 * E-Mail info@netburner.com
 *
 * Support is available: E-Mail support@netburner.com
 *
 *****************************************************************************/

void InitSingleEndAD();    // Setup the A/D converter to be ready to run

static inline void StartAD()
{
   sim2.adc.sr = 0xffff;   // Clear status codes
   sim2.adc.cr1 |= 0x2000; // Starting A2D
}           // Start A/D conversion set.

static inline bool ADDone()
{
   if (sim2.adc.sr & 0x0800)
      return true;
   else
      return false;
}             // Return true if the conversion is complete

static inline WORD GetADResult(int ch) //Get the AD Result
{
   return sim2.adc.rslt[ch];
}  // Return the AD Result








