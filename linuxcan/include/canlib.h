/**
 * \section LICENSE
 * <pre style="white-space: pre-wrap">
 * This software is dual licensed under the following two licenses:
 * BSD-new and GPLv2. You may use either one. See the included
 * COPYING file for details.
 *
 * License: BSD-new
 * ===============================================================================
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the \<organization\> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL \<COPYRIGHT HOLDER\> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * License: GPLv2
 * ===============================================================================
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * ---------------------------------------------------------------------------
 * </pre>
 *
 * \section DESCRIPTION
 *
 *   Definitions for the CANLIB API.
 *
 * \file canlib.h
 * \author Kvaser AB
 *
 * \defgroup General                 General
 * \defgroup CAN                     CAN
 * \defgroup ObjectBuffers           Object buffers
 */

#ifndef _CANLIB_H_
#define _CANLIB_H_

#include <stdlib.h>

#   define CANLIB_DECLARE_ALL
typedef unsigned char BYTE;
typedef unsigned int DWORD;
typedef unsigned int HANDLE;
typedef unsigned int BOOL;
#include "canstat.h"

/** Handle to an opened circuit. */
typedef int canHandle;
/** Handle to an opened circuit. */
   typedef canHandle CanHandle;

/**
 * \ingroup CAN
 */
typedef struct canNotifyData {
  void *tag;
  int eventType;
  union {
    struct {
      unsigned long time;
    } busErr;
    struct {
      long id;
      unsigned long time;
    } rx;
    struct {
      long id;
      unsigned long time;
    } tx;
    struct {
      unsigned char busStatus;
      unsigned char txErrorCounter;
      unsigned char rxErrorCounter;
      unsigned long time;
    } status;
  } info;
} canNotifyData;


/** Notify message sent to the application window */
# define WM__CANLIB 648

/**
 * \name canOPEN_xxx
 * \anchor canOPEN_xxx
 *
 * These defines are used in \ref canOpenChannel().
 *
 * @{
 */

// The canWANT_xxx names are also obsolete, use canOPEN_xxx instead for new developments.
#define canWANT_EXCLUSIVE               0x0008
#define canWANT_EXTENDED                0x0010
#define canWANT_VIRTUAL                 0x0020

/**
 * Don't allow sharing of this circuit between applications.
 *
 * This define is used in \ref canOpenChannel()
 */
#define canOPEN_EXCLUSIVE               0x0008

/**
 * This flag causes two things to happen:
 *
 * \li The call will fail if the specified circuit doesn't allow extended CAN
 *     (CAN 2.0B).
 *
 * \li If no frame-type flag is specified in a call to \ref canWrite, it is assumed
 *     that extended CAN should be used.
 *
 * This define is used in \ref canOpenChannel().
 */
#define canOPEN_REQUIRE_EXTENDED        0x0010

/**
 * Allow opening of virtual channels as well as physical channels.
 *
 * This define is used in \ref canOpenChannel().
 *
 * \sa \ref page_user_guide_virtual_info
 */
# define canOPEN_ACCEPT_VIRTUAL         0x0020


/**
 * The channel will accept messages with DLC (Data Length Code) greater than
 * 8. If this flag is not used, a message with DLC > 8 will always be
 * reported or transmitted as a message with DLC = 8. If the
 * \ref canOPEN_ACCEPT_LARGE_DLC flag is used, the message will be sent and/or
 * received with the true DLC, which can be at most 15.
 *
 * \note The length of the message is always at most 8.
 *
 * This define is used in \ref canOpenChannel().
 */
# define canOPEN_ACCEPT_LARGE_DLC       0x0200  // DLC can be greater than 8

/**
 * The channel will use the CAN FD protocol. This also means that messages with
 * \ref canFDMSG_xxx flags can now be used.
 *
 * This define is used in \ref canOpenChannel().
 */
# define canOPEN_CAN_FD                 0x0400

/**
 * The channel will use the CAN FD NON-ISO protocol. This also means that
 * messages with \ref canFDMSG_xxx flags can now be used.
 *
 * This define is used in \ref canOpenChannel().
 */
# define canOPEN_CAN_FD_NONISO          0x0800
/** @} */

/**
 * \ingroup CAN
 * \name canFILTER_xxx
 * \anchor canFILTER_xxx
 *
 * Flags for \ref canAccept().
 *
 * @{
 */
#define canFILTER_ACCEPT        1
#define canFILTER_REJECT        2
/** Sets the code for standard (11-bit) identifiers. */
#define canFILTER_SET_CODE_STD  3
/** Sets the mask for standard (11-bit) identifiers. */
#define canFILTER_SET_MASK_STD  4
/** Sets the code for extended (29-bit) identifiers. */
#define canFILTER_SET_CODE_EXT  5
/** Sets the mask for extended (29-bit) identifiers. */
#define canFILTER_SET_MASK_EXT  6

#define canFILTER_NULL_MASK     0L
/** @} */


/**
 * \ingroup CAN
 * \name canDRIVER_xxx
 * \anchor canDRIVER_xxx
 *
 * CAN driver types - not all are supported on all cards.
 *
 * @{
 */

/**
 * The "normal" driver type (push-pull). This is the default.
 */
#define canDRIVER_NORMAL           4

/**
 * Sets the CAN controller in Silent Mode; that is, it doesn't send anything,
 * not even ACK bits, on the bus.  Reception works as usual.
 *
 * \note The values 2,3,5,6,7 are reserved values for compatibility reasons.
 */
#define canDRIVER_SILENT           1

/**
 * Self-reception. Not implemented.
 */
#define canDRIVER_SELFRECEPTION    8

/**
 * The driver is turned off. Not implemented in all types of hardware.
 */
#define canDRIVER_OFF              0

/** @} */


/**
 * \ingroup CAN
 * \anchor BAUD_xxx
 * \anchor canBITRATE_xxx
 * \name canBITRATE_xxx
 *
 * Common bus speeds. Used in \ref canSetBusParams() and \ref canSetBusParamsC200().
 * The values are translated in canlib, \ref canTranslateBaud().
 *
 * \note The \ref BAUD_xxx names are only retained for compability.
 *
 * \sa \ref page_user_guide_misc_bitrate
 *
 * @{
 */


/** Used in \ref canSetBusParams() and \ref canSetBusParamsC200(). Indicate a bitrate of 1 Mbit/s. */
#define canBITRATE_1M        (-1)
/** Used in \ref canSetBusParams() and \ref canSetBusParamsC200(). Indicate a bitrate of 500 kbit/s. */
#define canBITRATE_500K      (-2)
/** Used in \ref canSetBusParams() and \ref canSetBusParamsC200(). Indicate a bitrate of 250 kbit/s. */
#define canBITRATE_250K      (-3)
/** Used in \ref canSetBusParams() and \ref canSetBusParamsC200(). Indicate a bitrate of 125 kbit/s. */
#define canBITRATE_125K      (-4)
/** Used in \ref canSetBusParams() and \ref canSetBusParamsC200(). Indicate a bitrate of 100 kbit/s. */
#define canBITRATE_100K      (-5)
/** Used in \ref canSetBusParams() and \ref canSetBusParamsC200(). Indicate a bitrate of 62 kbit/s. */
#define canBITRATE_62K       (-6)
/** Used in \ref canSetBusParams() and \ref canSetBusParamsC200(). Indicate a bitrate of 50 kbit/s. */
#define canBITRATE_50K       (-7)
/** Used in \ref canSetBusParams() and \ref canSetBusParamsC200(). Indicate a bitrate of 83 kbit/s. */
#define canBITRATE_83K       (-8)
/** Used in \ref canSetBusParams() and \ref canSetBusParamsC200(). Indicate a bitrate of 10 kbit/s. */
#define canBITRATE_10K       (-9)

// CAN FD Bit Rates
/** Used in \ref canSetBusParamsFd(). Indicates a bitrate of  0.5 Mbit/s and sampling point at 80%. */
#define canFD_BITRATE_500K_80P     (-1000)
/** Used in \ref canSetBusParamsFd(). Indicates a bitrate of  1.0 Mbit/s and sampling point at 80%. */
#define canFD_BITRATE_1M_80P       (-1001)
/** Used in \ref canSetBusParamsFd(). Indicates a bitrate of  2.0 Mbit/s and sampling point at 80%. */
#define canFD_BITRATE_2M_80P       (-1002)
/** Used in \ref canSetBusParamsFd(). Indicates a bitrate of  4.0 Mbit/s and sampling point at 80%. */
#define canFD_BITRATE_4M_80P       (-1003)
/** Used in \ref canSetBusParamsFd(). Indicates a bitrate of  8.0 Mbit/s and sampling point at 60%. */
#define canFD_BITRATE_8M_60P       (-1004)

/** The \ref BAUD_xxx names are deprecated, use \ref canBITRATE_1M instead. */
#define BAUD_1M              (-1)
/** The \ref BAUD_xxx names are deprecated, use \ref canBITRATE_500K instead. */
#define BAUD_500K            (-2)
/** The \ref BAUD_xxx names are deprecated, use \ref canBITRATE_250K instead. */
#define BAUD_250K            (-3)
/** The \ref BAUD_xxx names are deprecated, use \ref canBITRATE_125K instead. */
#define BAUD_125K            (-4)
/** The \ref BAUD_xxx names are deprecated, use \ref canBITRATE_100K instead. */
#define BAUD_100K            (-5)
/** The \ref BAUD_xxx names are deprecated, use \ref canBITRATE_62K instead. */
#define BAUD_62K             (-6)
/** The \ref BAUD_xxx names are deprecated, use \ref canBITRATE_50K instead. */
#define BAUD_50K             (-7)
/** The \ref BAUD_xxx names are deprecated, use \ref canBITRATE_83K instead. */
#define BAUD_83K             (-8)
/** @} */


//
// Define CANLIBAPI unless it's done already.
// (canlib.c provides its own definitions of CANLIBAPI, DLLIMPORT
// and DLLEXPORT before including this file.)
//
#ifndef CANLIBAPI
#   define CANLIBAPI
#   define __stdcall
#endif


#ifdef __cplusplus
extern "C" {
#endif

/**
 * \ingroup General
 *
 * \source_cs       <b>static void canInitializeLibrary(void);</b>
 *
 * \source_delphi   <b>procedure canInitializeLibrary;    </b>
 * \source_end
 * This function must be called before any other functions is used.  It will
 * initialize the driver.
 *
 * You may call \ref canInitializeLibrary() more than once. The actual
 * initialization will take place only once.
 *
 * Any errors encountered during library initialization will be "silent" and an
 * appropriate \ref canERR_xxx error code will be returned later on when
 * \ref canOpenChannel() (or any other API call that requires initialization) is
 * called.
 *
 * \sa \ref page_code_snippets_examples
 *
 */
void CANLIBAPI canInitializeLibrary (void);

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canClose(int handle);</b>
 *
 * \source_delphi   <b>function canClose(handle: canHandle): canStatus;</b>
 * \source_end
 *
 * Closes the channel associated with the handle. If no other threads
 * are using the CAN circuit, it is taken off bus. The handle can not be
 * used for further references to the channel, so any variable containing
 * it should be zeroed.
 *
 * \ref canClose() will almost always return \ref canOK; the specified handle is closed
 * on an best-effort basis.
 *
 * \param[in]  hnd  An open handle to a CAN channel.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_code_snippets_examples
 * \sa \ref canOpenChannel(), \ref canBusOn(), \ref canBusOff()
 */
canStatus CANLIBAPI canClose (const CanHandle hnd);

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canBusOn(int handle);</b>
 *
 * \source_delphi   <b>function canBusOn(handle: canHandle): canStatus;    </b>
 * \source_end
 *
 * Takes the specified channel on-bus.
 *
 * If you are using multiple handles to the same physical channel, for example
 * if you are writing a threaded application, you must call \ref canBusOn() once for
 * each handle. The same applies to \ref canBusOff() - the physical channel will not
 * go off bus until the last handle to the channel goes off bus.
 *
 * \param[in]  hnd  An open handle to a CAN channel.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_code_snippets_examples
 * \sa \ref canBusOff(), \ref canResetBus()
 *
 */
canStatus CANLIBAPI canBusOn (const CanHandle hnd);

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canBusOff(int handle);</b>
 *
 * \source_delphi   <b>function canBusOff(handle: canHandle): canStatus; </b>
 * \source_end
 *
 * Takes the specified channel off-bus.
 *
 * \param[in]  hnd  An open handle to a CAN channel.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_code_snippets_examples
 * \sa \ref canBusOn(), \ref canResetBus()
 *
 */
canStatus CANLIBAPI canBusOff (const CanHandle hnd);

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canSetBusParams(int handle, int freq, int tseg1, int tseg2, int sjw, int noSamp, int syncmode); </b>
 *
 * \source_delphi   <b>function canSetBusParams(handle: canHandle; freq: Longint; tseg1, tseg2, sjw, noSamp, syncmode: Cardinal): canStatus;     </b>
 * \source_end
 *
 * This function sets the bus timing parameters for the specified CAN controller.
 *
 * The library provides default values for \a tseg1, \a tseg2, \a sjw and \a
 * noSamp when \a freq is specified to one of the
 * pre-defined constants, \ref canBITRATE_xxx.
 *
 * If \a freq is any other value, no default values are supplied by the
 * library.
 *
 * If you are using multiple handles to the same physical channel, for example
 * if you are writing a threaded application, you must call \ref canBusOff() once
 * for each handle. The same applies to \ref canBusOn() - the physical channel will
 * not go off bus until the last handle to the channel goes off bus.
 *
 * \note Use \ref canSetBusParamsC200() to set the bus timing parameters in the
 *  ubiquitous 82c200 bit-timing register format.
 *
 * \param[in]  hnd       An open handle to a CAN controller.
 * \param[in]  freq      Bit rate (measured in bits per second); or one of the
 *                       predefined constants \ref canBITRATE_xxx, which are described below.
 * \param[in]  tseg1     Time segment 1, that is, the number of quanta from (but not
 *                       including) the Sync Segment to the sampling point.
 * \param[in]  tseg2     Time segment 2, that is, the number of quanta from the sampling
 *                       point to the end of the bit.
 * \param[in]  sjw       The Synchronization Jump Width; can be 1,2,3, or 4.
 * \param[in]  noSamp    The number of sampling points; can be 1 or 3.
 * \param[in]  syncmode  Unsupported and ignored.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_code_snippets_bit_rate, \ref page_user_guide_misc_bitrate,
 * \ref page_user_guide_init_bit_rate, \ref page_code_snippets_examples
 * \sa \ref canSetBusParamsC200(), \ref canGetBusParams()
 *
 */
canStatus CANLIBAPI canSetBusParams (const CanHandle hnd,
                                     long freq,
                                     unsigned int tseg1,
                                     unsigned int tseg2,
                                     unsigned int sjw,
                                     unsigned int noSamp,
                                     unsigned int syncmode);

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canSetBusParamsFd(int hnd, int freq_brs, int tseg1_brs, int tseg2_brs, int sjw_brs);</b>
 *
 * \source_delphi   <b>function canSetBusParamsFd(hnd: canHandle; freq_brs: Longint; tseg1_brs, tseg2_brs, sjw_brs): canStatus;</b>
 * \source_end
 *
 * This function sets the bus timing parameters for the specified
 * CAN FD controller.
 *
 * The library provides default values for \a tseg1_brs, \a tseg2_brs,
 * \a sjw_brs and \a freq_brs is specified to one of the pre-defined
 * constants, \ref canBITRATE_xxx.
 *
 * If \a freq_brs is any other value, no default values are supplied
 * by the library.
 *
 * \param[in]  hnd        An open handle to a CAN controller.
 * \param[in]  freq_brs   Bit rate (measured in bits per second); or one of the
 *                        predefined constants \ref canBITRATE_xxx, which are described below.
 * \param[in]  tseg1_brs  Time segment 1, that is, the number of quanta from (but not
 *                        including) the Sync Segment to the sampling point.
 * \param[in]  tseg2_brs  Time segment 2, that is, the number of quanta from the sampling
 *                        point to the end of the bit.
 * \param[in]  sjw_brs    The Synchronization Jump Width; can be 1,2,3, or 4.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 */
canStatus CANLIBAPI canSetBusParamsFd(const CanHandle hnd,
                                      long freq_brs,
                                      unsigned int tseg1_brs,
                                      unsigned int tseg2_brs,
                                      unsigned int sjw_brs);


/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canGetBusParams(int handle, out long freq, out int tseg1, out int tseg2, out int sjw, out int noSamp, out int syncmode);</b>
 *
 * \source_delphi   <b>function canGetBusParams(handle: canHandle; var freq: Longint; var tseg1, tseg2, sjw, noSamp, syncmode: Cardinal): canStatus;     </b>
 * \source_end
 *
 * This function retrieves the current bus parameters for the specified
 * channel.
 *
 * The anatomy of a CAN bit is discussed in detail at Kvaser's
 * web site at <a href="http://www.kvaser.com">www.kvaser.com</a>.
 *
 * \param[in]  hnd       An open handle to a CAN controller.
 * \param[out] freq      Bit rate (bits per second).
 * \param[out] tseg1     Time segment 1, that is, the number of quanta from (but not
 *                       including) the Sync Segment to the sampling point.
 * \param[out] tseg2     Time segment 2, that is, the number of quanta from the sampling
 *                       point to the end of the bit.
 * \param[out] sjw       The Synchronization Jump Width; can be 1,2,3, or 4.
 * \param[out] noSamp    The number of sampling points; can be 1 or 3.
 * \param[out] syncmode  Unsupported, always read as one.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_code_snippets_bit_rate, \ref page_user_guide_init_bit_rate
 * \sa \ref canSetBusParams(), \ref canSetBusParamsC200()
 *
 */
canStatus CANLIBAPI canGetBusParams (const CanHandle hnd,
                                     long  *freq,
                                     unsigned int *tseg1,
                                     unsigned int *tseg2,
                                     unsigned int *sjw,
                                     unsigned int *noSamp,
                                     unsigned int *syncmode);


/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canGetBusParamsFd(int hnd, out long freq_brs, out int tseg1_brs, out int tseg2_brs, out int sjw_brs);</b>
 *
 * \source_delphi   <b>function canGetBusParamsFd(hnd: canHandle; var freq_brs: Longint; var tseg1_brs, tseg2_brs, sjw_brs): canStatus;</b>
 * \source_end
 *
 * This function retrieves the current bus parameters for the specified
 * CAN FD channel.
 *
 * \param[in]  hnd         An open handle to a CAN FD controller.
 * \param[out] freq_brs    Bit rate (bits per second).
 * \param[out] tseg1_brs   Time segment 1, that is, the number of quanta from (but not
 *                         including) the Sync Segment to the sampling point.
 * \param[out] tseg2_brs   Time segment 2, that is, the number of quanta from the sampling
 *                         point to the end of the bit.
 * \param[out] sjw_brs     The Synchronization Jump Width; can be 1,2,3, or 4.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 */
canStatus CANLIBAPI canGetBusParamsFd(const CanHandle hnd,
                                      long  *freq_brs,
                                      unsigned int *tseg1_brs,
                                      unsigned int *tseg2_brs,
                                      unsigned int *sjw_brs);
/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canSetBusOutputControl(int handle, int drivertype);</b>
 *
 * \source_delphi   <b>function canSetBusOutputControl(handle: canHandle; drivertype: Cardinal): canStatus;     </b>
 * \source_end
 *
 * This function sets the driver type for a CAN controller. This corresponds
 * loosely to the bus output control register in the CAN controller, hence the
 * name of this function. CANLIB does not allow for direct manipulation of the
 * bus output control register; instead, symbolic constants are used to select
 * the desired driver type.
 *
 * \note Not all CAN driver types are supported on all cards.
 *
 * \param[in]  hnd         A handle to an open circuit.
 * \param[out] drivertype  Can driver type, \ref canDRIVER_xxx)
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref canGetBusOutputControl()
 */
canStatus CANLIBAPI canSetBusOutputControl (const CanHandle hnd,
                                            const unsigned int drivertype);

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canGetBusOutputControl(int handle, out int drivertype);</b>
 *
 * \source_delphi   <b>function canGetBusOutputControl(handle: canHandle; var drivertype: Cardinal): canStatus;  </b>
 * \source_end
 *
 * This function retrieves the current CAN controller driver type.
 * This corresponds loosely to the bus output control register in the
 * CAN controller, hence the name of this function. CANLIB does not
 * allow for direct manipulation of the bus output control register;
 * instead, symbolic constants are used to select the desired driver
 * type.
 *
 * \note Don't confuse the CAN controller driver type with the bus driver
 *       type. The CAN controller is not connected directly to the CAN bus;
 *       instead, it is connected to a bus transceiver circuit which interfaces
 *       directly to the bus. The "CAN controller driver type" we are talking
 *       about here refers to the mode which the CAN controller uses to drive
 *       the bus transceiver circuit.
 *
 * \note Silent Mode is not supported by all CAN controllers.
 *
 * \param[in] hnd         An open handle to a CAN circuit.
 * \param[in] drivertype  A pointer to an unsigned int which receives the
 *                        current driver type. The driver type can be either
 *                        \ref canDRIVER_NORMAL or \ref canDRIVER_SILENT.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref canSetBusOutputControl()
 */
canStatus CANLIBAPI canGetBusOutputControl (const CanHandle hnd,
                                            unsigned int *drivertype);

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canAccept(int handle, int envelope, int flag);</b>
 *
 * \source_delphi   <b>function canAccept(handle: canHandle; envelope: Longint; flag: Cardinal): canStatus;     </b>
 * \source_end
 *
 * This routine sets the message acceptance filters on a CAN channel.
 *
 * On some boards the acceptance filtering is done by the CAN hardware; on
 * other boards (typically those with an embedded CPU,) the acceptance
 * filtering is done by software. \ref canAccept() behaves in the same way for all
 * boards, however.
 *
 * \win_start \ref canSetAcceptanceFilter() and \win_end \ref canAccept() both serve the same purpose but the
 * former can set the code and mask in just one call.
 *
 * If you want to remove a filter, call \ref canAccept() with the mask set to 0.
 *
 * \note You can set the extended code and mask only on CAN boards that support
 *       extended identifiers.
 *
 * \note Not all CAN boards support different masks for standard and extended
 *       CAN identifiers.
 *
 * \param[in]  hnd       An open handle to a CAN circuit.
 * \param[in]  envelope  The mask or code to set.
 * \param[in]  flag      Any of \ref canFILTER_SET_CODE_STD,
 *                       \ref canFILTER_SET_MASK_STD,
 *                       \ref canFILTER_SET_CODE_EXT or
 *                       \ref canFILTER_SET_MASK_EXT
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_user_guide_misc_code_and_mask,
 *     \ref page_user_guide_send_recv_filters,
 *     \ref page_code_snippets_examples
 */
canStatus CANLIBAPI canAccept (const CanHandle hnd,
                               const long envelope,
                               const unsigned int flag);

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canReadStatus(int handle, out long flags);</b>
 *
 * \source_delphi   <b>function canReadStatus(handle: canHandle; var flags: Longint): canStatus;     </b>
 * \source_end
 *
 * Returns the status for the specified circuit. flags points to a longword
 * which receives a combination of the \ref canSTAT_xxx flags.
 *
 * \note \ref canReadStatus() returns the latest known status of the specified
 *       circuit. If a status change happens precisely when \ref canReadStatus() is
 *       called, it may not be reflected in the returned result.
 *
 * \param[in]  hnd    A handle to an open circuit.
 * \param[out] flags  Pointer to a \c DWORD which receives the status flags;
 *                    this is a combination of any of the \ref canSTAT_xxx.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 */
canStatus CANLIBAPI canReadStatus (const CanHandle hnd,
                                   unsigned long *const flags);

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canReadErrorCounters(int handle, out int txErr, out int rxErr, out int ovErr);</b>
 *
 * \source_delphi   <b>function canReadErrorCounters(handle: canHandle; var txErr, rxErr, ovErr: Cardinal): canStatus;     </b>
 * \source_end
 *
 * Reads the error counters of the CAN controller.
 *
 * \ref canReadErrorCounters() returns the latest known values of the error counters
 * in the specified circuit. If the error counters change values precisely when
 * \ref canReadErrorCounters() is called, it may not be reflected in the returned
 * result.
 *
 * It is allowed to pass \c NULL as the value of the \a txErr, \a rxErr, and \a
 * ovErr parameters.
 *
 * Use \ref canIoCtl() to clear the counters.
 *
 * \note Not all CAN controllers provide access to the error counters;
 *       in this case, an educated guess is returned.
 *
 * \param[in]  hnd    A handle to an open circuit.
 * \param[out] txErr  A pointer to a \c DWORD which receives the transmit error
 *                    counter.
 * \param[out] rxErr  A pointer to a \c DWORD which receives the receive error
 *                    counter.
 * \param[out] ovErr  A pointer to a \c DWORD which receives the number of
 *                    overrun errors.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref canIoCtl()
 */
canStatus CANLIBAPI canReadErrorCounters (const CanHandle hnd,
                                          unsigned int *txErr,
                                          unsigned int *rxErr,
                                          unsigned int *ovErr);

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canWrite(int handle, int id, byte[] msg, int dlc, int flag);</b>
 *
 * \source_delphi   <b>function canWrite(handle: canHandle; id: Longint; msg: Pointer; dlc: Cardinal; flag: Cardinal): canStatus;     </b>
 * \source_end
 *
 * This function sends a CAN message. The call returns immediately after queuing
 * the message to the driver.
 *
 * If you are using the same channel via multiple handles, note that the
 * default behaviour is that the different handles will "hear" each other just as
 * if each handle referred to a channel of its own. If you open, say, channel 0
 * from thread A and thread B and then send a message from thread A, it will be
 * "received" by thread B.
 * This behaviour can be changed using \ref canIOCTL_SET_LOCAL_TXECHO.
 *
 * \note The message has been queued for transmission when this calls return.
 *       It has not necessarily been sent.
 *
 * \param[in]  hnd       A handle to an open CAN circuit.
 * \param[in]  id        The identifier of the CAN message to send.
 * \param[in]  msg       A pointer to the message data, or \c NULL.
 * \param[in]  dlc       The length of the message in bytes.<br>
                         For Classic CAN dlc can be at most 8, unless \ref canOPEN_ACCEPT_LARGE_DLC is used.<br>
                         For CAN FD dlc can be one of the following 0-8, 12, 16, 20, 24, 32, 48, 64.
 * \param[in]  flag      A combination of message flags, \ref canMSG_xxx.
 *                       Use this parameter to send extended (29-bit) frames
 *                       and/or remote frames. Use \ref canMSG_EXT and/or
 *                       \ref canMSG_RTR for this purpose.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_user_guide_send_recv_sending, \ref page_code_snippets_examples
 * \sa \ref canWriteSync(), \ref canWriteWait()
 *
 */
canStatus CANLIBAPI canWrite (const CanHandle hnd,
                              long id,
                              void *msg,
                              unsigned int dlc,
                              unsigned int flag);

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canWriteSync(int handle, long timeout);</b>
 *
 * \source_delphi   <b>function canWriteSync(handle: canHandle; timeout: Longint): canStatus;     </b>
 * \source_end
 *
 * Waits until all CAN messages for the specified handle are sent, or the
 * timeout period expires.
 *
 * \param[in]  hnd       A handle to an open CAN circuit.
 * \param[in]  timeout   The timeout in milliseconds. 0xFFFFFFFF gives an
 *                       infinite timeout.
 *
 * \return \ref canOK (zero) if the queue emptied before the timeout period came to
 *         its end.
 * \return \ref canERR_TIMEOUT (negative) not all messages were transmitted when
 *         the timeout occurred.
 * \return \ref canERR_PARAM (negative) This could be caused by an erroneous
 *         parameter, or if you have turned TXACKs off (by using \ref canIoCtl())
 *         because if you do you can't use this call. The driver simply doesn't
 *         know when all the messages are sent!
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref canWrite(), \ref canWriteWait()
 */
canStatus CANLIBAPI canWriteSync (const CanHandle hnd, unsigned long timeout);

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canRead(int handle, out int id, byte[] msg, out int dlc, out int flag, out long time);</b>
 *
 * \source_delphi   <b>function canRead(handle: canHandle; var id: Longint; msg: Pointer; var dlc: Cardinal; var flag: Cardinal; var time: Longint): canStatus;     </b>
 * \source_end
 *
 * Reads a message from the receive buffer. If no message is available, the
 * function returns immediately with return code \ref canERR_NOMSG.
 *
 * If you are using the same channel via multiple handles, note that the
 * default behaviour is that the different handles will "hear" each other just as
 * if each handle referred to a channel of its own. If you open, say, channel 0
 * from thread A and thread B and then send a message from thread A, it will be
 * "received" by thread B.
 * This behaviour can be changed using \ref canIOCTL_SET_LOCAL_TXECHO.
 *
 * It is allowed to pass \c NULL as the value of \a id, \a msg, \a dlc, \a
 * flag, and \a time.
 *
 * \param[in]  hnd   A handle to an open circuit.
 * \param[out] id    Pointer to a buffer which receives the CAN identifier.
 *                   This buffer will only get the identifier. To determine
 *                   whether this identifier was standard (11-bit) or extended
 *                   (29-bit), and/or whether it was remote or not, or if it
 *                   was an error frame, examine the contents of the flag
 *                   argument.
 * \param[out] msg   Pointer to the buffer which receives the message data.
 *                   This buffer must be large enough (i.e. 8 bytes.) Only the
 *                   message data is copied; the rest of the buffer is left
 *                   as-is.
 * \param[out] dlc   Pointer to a buffer which receives the message length.
 * \param[out] flag  Pointer to a buffer which receives the message flags,
 *                   which is a combination of the \ref canMSG_xxx and
 *                   \ref canMSGERR_xxx values.
 * \param[out] time  Pointer to a buffer which receives the message time stamp.
 *
 * \return \ref canOK (zero) if a message was read.
 * \return \ref canERR_NOMSG (negative) if there was no message available.
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_user_guide_send_recv_reading, \ref
 * page_user_guide_send_recv_mailboxes, \ref page_code_snippets_examples,
 * \ref page_user_guide_time_accuracy_and_resolution
 * \sa \win_start \ref canReadSpecific(), \ref canReadSpecificSkip(),\win_end \ref canReadSync(),
 *     \win_start \ref canReadSyncSpecific(),\win_end \ref canReadWait()
 *
 */
canStatus CANLIBAPI canRead (const CanHandle hnd,
                             long *id,
                             void *msg,
                             unsigned int *dlc,
                             unsigned int *flag,
                             unsigned long *time);

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canReadWait(int handle, out int id, byte[] msg, out int dlc, out int flag, out long time, long timeout);</b>
 *
 * \source_delphi   <b>function canReadWait(handle: canHandle; var id: Longint; msg: Pointer; var dlc: Cardinal; var flag: Cardinal; var time: Longint; timeout: Longint): canStatus;     </b>
 * \source_end
 *
 * Reads a message from the receive buffer. If no message is available, the
 * function waits until a message arrives or a timeout occurs.
 *
 * If you are using the same channel via multiple handles, note that the
 * default behaviour is that the different handles will "hear" each other just as
 * if each handle referred to a channel of its own. If you open, say, channel 0
 * from thread A and thread B and then send a message from thread A, it will be
 * "received" by thread B.
 * This behaviour can be changed using \ref canIOCTL_SET_LOCAL_TXECHO.
 *
 * It is allowed to pass \c NULL as the value of \a id, \a msg, \a dlc, \a
 * flag, and \a time.
 *
 * \param[in]   hnd    A handle to an open circuit.
 * \param[out]  id     Pointer to a buffer which receives the CAN identifier.
 *                     This buffer will only get the identifier. To determine
 *                     whether this identifier was standard (11-bit) or extended
 *                     (29-bit), and/or whether it was remote or not, or if it
 *                     was an error frame, examine the contents of the flag
 *                     argument.
 * \param[out]  msg    Pointer to the buffer which receives the message data.
 *                     This buffer must be large enough (i.e. 8 bytes.).
 * \param[out]  dlc    Pointer to a buffer which receives the message length.
 * \param[out]  flag   Pointer to a buffer which receives the message flags,
 *                     which is a combination of the \ref canMSG_xxx and
 *                     \ref canMSGERR_xxx values.
 * \param[out] time    Pointer to a buffer which receives the message time stamp.
 * \param[in]  timeout If no message is immediately available, this parameter
 *                     gives the number of milliseconds to wait for a message
 *                     before returning. 0xFFFFFFFF gives an infinite timeout.
 *
 * \return \ref canOK (zero) if a message was read.
 * \return \ref canERR_NOMSG (negative) if there was no message available.
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref canRead(),\win_start \ref canReadSpecific(), \ref canReadSpecificSkip(),
 *  \ref canReadSyncSpecific(),\win_end \ref canReadSync()
 *
 * \sa \ref page_user_guide_time_accuracy_and_resolution
 */
canStatus CANLIBAPI canReadWait (const CanHandle hnd,
                                 long *id,
                                 void *msg,
                                 unsigned int  *dlc,
                                 unsigned int  *flag,
                                 unsigned long *time,
                                 unsigned long timeout);

#if defined(CANLIB_DECLARE_ALL)
/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canReadSync(int handle, long timeout);</b>
 *
 * \source_delphi   <b>function canReadSync(handle: canHandle; timeout: Longint): canStatus;     </b>
 * \source_end
 *
 * Waits until the receive buffer contains at least one message or a timeout
 * occurs.
 *
 * If you are using the same channel via multiple handles, note that the
 * default behaviour is that the different handles will "hear" each other just as
 * if each handle referred to a channel of its own. If you open, say, channel 0
 * from thread A and thread B and then send a message from thread A, it will be
 * "received" by thread B.
 * This behaviour can be changed using \ref canIOCTL_SET_LOCAL_TXECHO.
 *
 * \param[in]  hnd      A handle to an open circuit.
 * \param[in]  timeout  The timeout in milliseconds. 0xFFFFFFFF gives an
 *                      infinite timeout.
 *
 * \return \ref canOK (zero) if the queue contains the desired message.
 * \return \ref canERR_TIMEOUT (negative) if a timeout occurs before a message
 *         arrived.
 * \return \ref canERR_xxx (negative) if the call fails.
 *
 * \sa \ref canRead(), \win_start \ref canReadSpecific(), \ref canReadSpecificSkip(),
 * \ref canReadSyncSpecific(),\win_end \ref canReadWait()
 */
canStatus CANLIBAPI canReadSync (const CanHandle hnd, unsigned long timeout);

#endif

/**
 * \ingroup CAN
 *
 * \todo Rewrite, this is just guessing from windows version
 *
 * \source_cs       <b>static Canlib.canStatus canSetNotify(int handle, IntPtr win_handle, int aNotifyFlags);</b>
 *
 * \source_delphi   <b>function canSetNotify(handle: canHandle; aHWnd: HWND; aNotifyFlags: Cardinal): canStatus;     </b>
 * \source_end
 *
 * This function associates a callback function with the CAN circuit.
 *
 * \param[in] hnd          A handle to an open CAN circuit.
 * \param[in] callback     Handle to callback routine.
 * \param[in] notifyFlags  The events specified with \ref canNOTIFY_xxx, for
 *                         which callback should be called.
 * \param[in] tag          Pointer to user defined data. Passed to callback in the \ref canNotifyData struct.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 */
canStatus CANLIBAPI canSetNotify (const CanHandle hnd,
                                  void (*callback)(canNotifyData *),
                                  unsigned int notifyFlags,
                                  void *tag);

/**
 * \ingroup CAN
 *
 * Returns raw handle/file descriptor for use in system calls.
 * \note Use this function with caution.
 *
 * \param[in]  hnd   CanHandle
 * \param[out] pvFd  Pointer to raw can data.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 */
canStatus CANLIBAPI canGetRawHandle (const CanHandle hnd, void *pvFd);

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canTranslateBaud(ref int freq, out int tseg1, out int tseg2, out int sjw, out int nosamp, out int syncMode);</b>
 *
 * \source_delphi   <b>function canTranslateBaud(var freq: longint; var tseg1, tseg2, sjw, noSamp, syncMode: Cardinal): canStatus;     </b>
 * \source_end
 *
 * This function translates the \ref canBITRATE_xxx constants to their corresponding
 * bus parameter values. At return, this \a freq contains the actual bit rate
 * (in bits per second). \a TSeg1 is the number of quanta (less one) in a bit
 * before the sampling point. \a TSeg2 is the number of quanta after the
 * sampling point.
 *
 * \param[in]  freq      A pointer to a \c DWORD which contains the \ref canBITRATE_xxx
 *                       constant to translate
 * \param[in]  tseg1     A pointer to a buffer which receives the Time segment 1,
 *                       that is, the number of quanta from (but not including)
 *                       the Sync Segment to the sampling point.
 * \param[in]  tseg2     A pointer to a buffer which receives the Time segment 2,
 *                       that is, the number of quanta from the sampling point to
 *                       the end of the bit.
 * \param[in]  sjw       A pointer to a buffer which receives the Synchronization
 *                       Jump Width; can be 1,2,3, or 4.
 * \param[in]  nosamp    A pointer to a buffer which receives the number of
 *                       sampling points; can be 1 or 3.
 * \param[in]  syncMode  Unsupported, always read as zero.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref canSetBusParams()
 */
canStatus CANLIBAPI canTranslateBaud (long *const freq,
                                      unsigned int *const tseg1,
                                      unsigned int *const tseg2,
                                      unsigned int *const sjw,
                                      unsigned int *const nosamp,
                                      unsigned int *const syncMode);

/**
 * \ingroup General
 *
 * \source_cs       <b>static Canlib.canStatus canGetErrorText(Canlib.canStatus err, out string buf_str);</b>
 *
 * \source_delphi   <b>function canGetErrorText(err: canStatus; buf: PChar; bufsiz: Cardinal): canStatus;     </b>
 * \source_end
 *
 * This function translates an error code (\ref canERR_xxx)
 * to a human-readable, English text.
 *
 * \param[in]     err     The error code.
 * \param[in,out] buf     The buffer which is to receive the text, which is a
 *                        zero-terminated string (provided the buffer is large enough.)
 * \param[in]     bufsiz  The length of the input buffer.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_code_snippets_examples
 *
 */
canStatus CANLIBAPI canGetErrorText (canStatus err, char *buf, unsigned int bufsiz);

/**
 * \ingroup  General
 *
 * \source_cs       <b>static short canGetVersion();</b>
 *
 * \source_delphi   <b>function canGetVersion: Word;     </b>
 * \source_end
 *
 * \win_start
 * This API call returns the version of the CANLIB API DLL (canlib32.dll).  The
 * most significant byte is the major version number and the least significant
 * byte is the minor version number.
 *
 * The actual version of the different driver files can be obtained by studying
 * the version resources in each of the files.
 *
 * \note The version number of the canlib32.dll file is not related to the
 *       product version of CANLIB you are using. CANLIB consists of several
 *       driver and DLL files. To obtain the product version, use
 *       \ref canGetVersionEx().
 *
 * \return version number of canlib32.dll
 *
 * \sa \ref page_user_guide_build_driver_version
 * \sa \ref canGetVersionEx(), \ref canProbeVersion()
 *
 * \win_end
 *
 * \linux_start
 * This API call returns the version of the CANLIB API library (libcanlib.so.x.y). The
 * most significant byte is the major version number and the least significant
 * byte is the minor version number.
 *
 * \return version number of libcanlib.so.x.y
 *
 * \linux_end
 *
 *
 */
unsigned short CANLIBAPI canGetVersion (void);

/**
 * \ingroup General
 *
 * \source_cs       <b>static Canlib.canStatus canIoCtl(int handle, int func, int val);<br>
      static Canlib.canStatus canIoCtl(int handle, int func, out int val);<br>
      static Canlib.canStatus canIoCtl(int handle, int func, out string str_buf);<br>
      static Canlib.canStatus canIoCtl(int handle, int func, ref object obj_buf);</b>
 *
 * \source_delphi   <b>function canIoCtl(handle: canHandle; func: Cardinal; buf: Pointer; buflen: Cardinal): canStatus;     </b>
 * \source_end
 *
 * This API call performs several different functions; these are described
 * below. The functions are handle-specific unless otherwise noted; this means
 * that they affect only the handle you pass to \ref canIoCtl(), whereas other open
 * handles will remain unaffected.  The contents of \a buf after the call is
 * dependent on the function code you specified.
 *
 * \param[in]     hnd     A handle to an open circuit.
 * \param[in]     func    A \ref canIOCTL_xxx function code
 * \param[in,out] buf     Pointer to a buffer containing function-dependent data;
                          or a \c NULL pointer for certain function codes. The
                          buffer can be used for both input and output
                          depending on the function code. See \ref canIOCTL_xxx.
 * \param[in]     buflen  The length of the buffer.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 */
canStatus CANLIBAPI canIoCtl (const CanHandle hnd,
                              unsigned int func,
                              void *buf,
                              unsigned int buflen);

/* Note the difference from the windows version */

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static canStatus canReadTimer(int hnd, long time);</b>
 *
 * \source_delphi   <b>function canReadTimer(handle: canHandle; time: longint): canStatus;     </b>
 * \source_end
 *
 * Reads the current time from the clock used to timestamp the
 * messages for the indicated circuit.
 *
 * This API may return \ref canERR_INVHANDLE and/or \ref canERR_NOTINITIALIZED!
 * This happens if \a hnd is invalid, or if the library was not initialized.
 *
 * \note The clock used to timestamp the messages may not be available for
 * direct reading on all platforms. In such cases, the PC's clock is used
 * to return an approximation of the current time. Note that clock drift might
 * occur in this case.
 *
 * \param[in]  hnd   A handle to an open circuit.
 * \param[out] time  The current time, with the prevailing time resolution.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_user_guide_time_accuracy_and_resolution
 * \win_start
 * \sa \ref kvReadTimer()
 * \win_end
 */
canStatus CANLIBAPI canReadTimer (const CanHandle hnd, unsigned long *time);

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static int canOpenChannel(int channel, int flags);</b>
 *
 * \source_delphi   <b>function canOpenChannel(channel: Integer; flags: Integer): canHandle;     </b>
 * \source_end
 *
 * Opens a CAN channel (circuit) and returns a handle which is used
 * in subsequent calls to CANLIB.
 *
 *
 * Channel numbering is dependent on the installed hardware. The first channel
 * always has number 0.
 *
 * For example,
 *
 * \li If you have a single LAPcan, the channels are numbered 0 and 1.
 *
 * \li If you have a USBcan Professional, the channels are numbered 0-1
 *     according to the labels on the cables.
 *
 * \li The virtual channels come after all physical channels.
 *
 * If you are using multiple threads, note that the returned handle is usable
 * only in the context of the thread that created it. That is, you must call
 * \ref canOpenChannel() in each of the threads in your application that uses the
 * CAN bus. You can open the same channel from multiple threads, but you must
 * call \ref canOpenChannel() once per thread.
 *
 * If you are using the same channel via multiple handles, note that the
 * default behaviour is that the different handles will "hear" each other just as
 * if each handle referred to a channel of its own. If you open, say, channel 0
 * from thread A and thread B and then send a message from thread A, it will be
 * "received" by thread B.
 * This behaviour can be changed using \ref canIOCTL_SET_LOCAL_TXECHO.
 *
 * \note The handle returned may be zero which is perfectly valid.
 *
 * \param[in]  channel  The number of the channel. Channel numbering is hardware
 *                      dependent.
 * \param[in]  flags    A combination of \ref canOPEN_xxx flags
 *
 * \return Returns a handle to the opened circuit, or \ref canERR_xxx
 *         (negative) if the call failed.
 *
 * \sa \ref page_code_snippets_examples,  \ref page_user_guide_virtual_info
 * \sa \ref canGetNumberOfChannels(), \ref canGetChannelData(), \ref canIoCtl()
 *
 */
CanHandle CANLIBAPI canOpenChannel (int channel, int flags);

/**
 * \ingroup General
 *
 * \source_cs       <b>static Canlib.canStatus canGetNumberOfChannels(out int channelCount);</b>
 *
 * \source_delphi   <b>function canGetNumberOfChannels(var channelCount: Integer): canStatus;     </b>
 * \source_end
 *
 * This function returns the number of available CAN channels in the
 * computer. The virtual channels are included in this number.
 *
 * \param[out] channelCount  A pointer to a \c DWORD which will receive the current
 *                           number of channels.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_code_snippets_examples, \ref page_user_guide_virtual_info
 * \sa \ref canGetChannelData()
 */
canStatus CANLIBAPI canGetNumberOfChannels (int *channelCount);

/**
 * \ingroup General
 *
 * \source_cs       <b>static Canlib.canStatus canGetChannelData(int channel, int item, out object buffer);</b>
 *
 * \source_delphi   <b>function canGetChannelData(channel, item: Integer; var buffer; bufsize: Cardinal): canStatus;     </b>
 * \source_end
 *
 * This function can be used to retrieve certain pieces of information about a channel.
 *
 * \note You must pass a channel number and not a channel handle.
 *
 * \param[in]  channel  The number of the channel you are interested in. Channel
 *                        numbers are integers in the interval beginning at 0
 *                        (zero) and ending at the value returned by
 *                        \ref canGetNumberOfChannels() minus 1.
 * \param[in]  item  This parameter specifies what data to obtain for the
 *                        specified channel. The value is one of the constants
 *                        \ref canCHANNELDATA_xxx.
 * \param[in,out] buffer  The address of a buffer which is to receive the data.
 * \param[in]  bufsize    The size of the buffer to which the buffer parameter
 *                        points.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_code_snippets_examples
 * \sa \ref canGetNumberOfChannels()
 */
canStatus CANLIBAPI canGetChannelData (int channel,
                                       int item,
                                       void *buffer,
                                       size_t bufsize);

/**
 * \ingroup General
 * \anchor canCHANNELDATA_xxx
 * \name canCHANNELDATA_xxx
 *
 * These defines are used in \ref canGetChannelData().
 *
 *  @{
 */

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a 32-bit unsigned integer that receives the
   * capabilities of the CAN controller; this is a combination of the \ref
   * canCHANNEL_CAP_xxx flags.
   */
#define canCHANNELDATA_CHANNEL_CAP                1

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a 32-bit unsigned integer that receives the
   * capabilities of the CAN transceiver; this is a combination of the
   * \ref canDRIVER_CAP_xxx flags.
   */
#define canCHANNELDATA_TRANS_CAP                  2

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \note Currently not implemented.
   */
#define canCHANNELDATA_CHANNEL_FLAGS              3   // available, etc

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a 32-bit unsigned integer that receives the hardware
   * type of the card. This value is any one of the \ref canHWTYPE_xxx
   * constants.
   */
#define canCHANNELDATA_CARD_TYPE                  4

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a 32-bit unsigned integer that receives the card's
   * number in the computer. Each card type is numbered separately. For
   * example, the first LAPcan card in a machine will have number 0, the second
   * LAPcan number 1, etc.
   */
#define canCHANNELDATA_CARD_NUMBER                5

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a 32-bit unsigned integer which receives the channel
   * number on the card.
   */
#define canCHANNELDATA_CHAN_NO_ON_CARD            6

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a 64-bit (8 bytes) area which receives the serial
   * number of the card. If the card doesn't have a serial number, 0 is
   * returned. The serial number is an 8-byte unsigned integer. Currently, no
   * products are using all 8 bytes; at most 4 bytes are used.
   */
#define canCHANNELDATA_CARD_SERIAL_NO             7

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a 64-bit (8 bytes) area which receives the serial
   * number of the transceiver. The serial number is an 8-byte unsigned
   * integer. If the transceiver doesn't have a serial number, 0 is returned.
   */
#define canCHANNELDATA_TRANS_SERIAL_NO            8

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a 64-bit (8 bytes) area which receives the firmware
   * revision number on the card. This number consists of four 16-bit words:
   * the major revision, the minor revision, the release number and the build
   * number, listed in order from the most significant to the least
   * significant.
   */
#define canCHANNELDATA_CARD_FIRMWARE_REV          9

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a 64-bit (8 bytes) area which receives the hardware
   * revision number on the card. This number consists of four 16-bit words;
   * the two most significant are always 0, and the two least significant are
   * the major revision and the minor revision, listed in order from the most
   * significant to the least significant.
   */
#define canCHANNELDATA_CARD_HARDWARE_REV          10

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a 8-byte area which receives the UPC (EAN) number for
   * the card. If there is no UPC number, the buffer is filled with zeros. The
   * UPC (EAN) number is coded as a BCD string with the LSB first, so
   * e.g. 733-0130-00122-0 is coded as 0x30001220 0x00073301.
   */
#define canCHANNELDATA_CARD_UPC_NO                11

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a 8-byte area which receives the UPC (EAN) number for
   * the transceiver. If there is no UPC number, the buffer is filled with
   * zeros. The UPC (EAN) number is coded as a BCD string with the LSB first,
   * so e.g. 733-0130-00122-0 is coded as 0x30001220 0x00073301.
   */
#define canCHANNELDATA_TRANS_UPC_NO               12

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to an area which receives a zero-terminated string with a
   * clear-text name of the channel.
   *
   * \note Use of this item code is no longer recommended. The returned
   * channel name doesn't contain the exact hardware type (it just contains
   * the device family) and uses zero-based channel numbering, which is not
   * user friendly.  Instead, use e.g. \ref canCHANNELDATA_DEVDESCR_ASCII and
   * \ref canCHANNELDATA_CHAN_NO_ON_CARD to build your own channel name.
   *
   *\win_start
   * \sa \ref canCHANNELDATA_DEVNAME_ASCII
   *\win_end
   */
#define canCHANNELDATA_CHANNEL_NAME               13
#if defined(CANLIB_DECLARE_ALL)

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to an array of 4 16-bit unsigned integers which receives
   * the file version number of the second-level DLL driver file, i.e. the DLL
   * that interfaces between CANLIB32.DLL and the driver proper.
   *
   * Contents depening on index:
   *
   * \li 0: 0
   * \li 1: The build number
   * \li 2: The minor revision number
   * \li 3: The major revision number
   */
# define canCHANNELDATA_DLL_FILE_VERSION          14

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to an array of 4 16-bit unsigned integers which receives
   * the product version number of the second-level DLL driver file, i.e. the
   * DLL that interfaces between CANLIB32.DLL and the driver proper.
   *
   * Contents depening on index:
   *
   * \li 0: 0
   * \li 1: 1
   * \li 2: The minor revision number
   * \li 3: The major revision number
   */
# define canCHANNELDATA_DLL_PRODUCT_VERSION       15

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a 32-bit unsigned integer which receives a number that
   * identifies the second-level DLL driver file, i.e. the DLL that interfaces
   * between CANLIB32.DLL and the driver proper.
   *
   * Values:
   *
   * \li 1: kvalapw.dll - used with CANLIB up to 2.29.
   *
   * \li 2: kvalapw2.dll - used with CANLIB from 3.0 and on.
   */
# define canCHANNELDATA_DLL_FILETYPE              16

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a 32-bit unsigned integer which receives the CAN
   * transceiver type of the specified channel.  This value is one of the
   * \ref canTRANSCEIVER_TYPE_xxx
   */
# define canCHANNELDATA_TRANS_TYPE                17

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a 32-bit unsigned integer which receives an address
   * indicating where the device is located on its underlying bus. The
   * interpretation of this number is bus-specific. If the address is unknown
   * or the bus driver does not support an address, the bus driver leaves this
   * member at its default value of 0xFFFFFFFF.
   *
   * The following list describes the information certain bus drivers store in
   * the Address field for their child devices:
   *
   * \li ISA: Does not supply an address. Defaults to 0xFFFFFFFF.
   *
   * \li PC Card (PCMCIA): The socket number (typically 0x00 or 0x40)
   *
   * \li PCI: The device number in the high word and the function number in the
   *          low word.
   *
   * \li USB: The port number.
   */
# define canCHANNELDATA_DEVICE_PHYSICAL_POSITION  18

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a 32-bit unsigned integer which receives a number
   * associated with the device that can be displayed in the user
   * interface. This number is typically a user-perceived slot number, such as
   * a number printed next to the slot on the board, or some other number that
   * makes locating the physical device easier for the user. For buses with no
   * such convention, or when the UI number is unknown, 0xFFFFFFFF is returned.
   */
# define canCHANNELDATA_UI_NUMBER                 19

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a 32-bit unsigned integer which is set to 0, if the
   * legacy time synchronization is not currently enabled for the specified
   * channel, and 1, if the legacy time synchronization is currently enabled
   * for the specified channel.
   *
   * Legacy time synchronization is a mechanism that will keep the PC and CAN
   * channel clocks in sync. The synchronization is done in the driver, which
   * periodically calculates the difference between the PC clock and the CAN
   * device clock and compensates for the clock drift by recalculating the CAN
   * message time stamps. You need to enable clock synchronization in the
   * Control Panel using the Kvaser Hardware applet.
   *
   * \note Legacy time synchronization is implemented only on LAPcan and LAPcan
   * II. It is not related to Kvaser MagiSync&tm; which is implemented in the
   * high-end members of the Kvaser Leaf family. Kvaser MagiSync&tm; is always
   * enabled and allows for much more accurate time synchronization.
   *
   */
# define canCHANNELDATA_TIMESYNC_ENABLED          20

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to an array of four 16-bit unsigned integers which
   * receives the file version number of the kernel-mode driver.
   *
   * Contents depening on index:
   *
   * \li 0: The build number
   * \li 1: 0
   * \li 2: The minor revision number
   * \li 3: The major revision number
   */
# define canCHANNELDATA_DRIVER_FILE_VERSION       21

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   *  \a buffer points to an array of four 16-bit unsigned integers which
   *  receives the product version number of the kernel-mode driver.
   *
   * Contents depening on index:
   *
   * \li 0: 0
   * \li 1: 0
   * \li 2: The minor revision number
   * \li 3: The major revision number
   */
# define canCHANNELDATA_DRIVER_PRODUCT_VERSION    22

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a buffer which receives the device manufacturer's name
   * as a zero-terminated Unicode string.
   */
# define canCHANNELDATA_MFGNAME_UNICODE           23

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a buffer which receives the device manufacturer's name
   * as a zero-terminated ASCII string.
   */
# define canCHANNELDATA_MFGNAME_ASCII             24

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a buffer which receives the product name of the device
   * as a zero-terminated Unicode string.
   */
# define canCHANNELDATA_DEVDESCR_UNICODE          25

  /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a buffer which receives the product name of the device
   * as a zero-terminated ASCII string.
   */
# define canCHANNELDATA_DEVDESCR_ASCII            26

 /**
   * This define is used in \ref canGetChannelData(), \a buffer
   * mentioned below refers to this functions argument.
   *
   * \a buffer points to a buffer which receives the name of the device
   * driver (e.g. "kcans") as a zero-terminated ASCII string.
   *
   * \note The device driver names have no special meanings and may change
   * from a release to another.
   */
# define canCHANNELDATA_DRIVER_NAME               27

#endif

 /** @} */


/**
 * \name canCHANNEL_IS_xxx
 * \anchor canCHANNEL_IS_xxx
 *
 * These channelFlags are used in \ref canGetChannelData() and in conjunction with \ref
 * canCHANNELDATA_CHANNEL_FLAGS.
 *  @{
 */
/** Used with \ref canCHANNELDATA_CHANNEL_FLAGS, indicates that the channel is
    opened exclusively. */
#define canCHANNEL_IS_EXCLUSIVE         0x0001
/** Used with \ref canCHANNELDATA_CHANNEL_FLAGS, indicates that the channel is
    opened. */
#define canCHANNEL_IS_OPEN              0x0002

/** Used with \ref canCHANNELDATA_CHANNEL_FLAGS, indicates that the channel is
 *  opened as CAN FD.  */

#define canCHANNEL_IS_CANFD             0x0004

//#define canCHANNEL_IS_CANFD_NON_ISO          0x0008 Reserved for when needed

 /** @} */


/**
 * \name canHWTYPE_xxx
 * \anchor canHWTYPE_xxx
 *
 * The following constants can be returned from \ref canGetChannelData(), using the
 * \ref canCHANNELDATA_CARD_TYPE item code. They identify the hardware type for
 * the channel specified in the call to \ref canGetChannelData().
 *
 * \note They indicate a hardware type, but not necessarily a specific
 * product. For example, \ref canHWTYPE_LAPCAN is returned both for LAPcan and
 * LAPcan II. (You can use \ref canGetChannelData() to obtain the UPC/EAN code for
 * the device. This number uniquely identifies the product.)
 *
 *  @{
 */
#define canHWTYPE_NONE                0  ///< Unknown or undefined
#define canHWTYPE_VIRTUAL             1  ///< The virtual CAN bus
#define canHWTYPE_LAPCAN              2  ///< LAPcan Family
#define canHWTYPE_CANPARI             3  ///< CANpari (obsolete).
#define canHWTYPE_PCCAN               8  ///< PCcan Family
#define canHWTYPE_PCICAN              9  ///< PCIcan Family
#define canHWTYPE_USBCAN             11  ///< USBcan (obsolete).
#define canHWTYPE_PCICAN_II          40  ///< PCIcan II family
#define canHWTYPE_USBCAN_II          42  ///< USBcan II, USBcan Rugged, Kvaser Memorator
#define canHWTYPE_SIMULATED          44  ///< Simulated CAN bus for Kvaser Creator (obsolete).
#define canHWTYPE_ACQUISITOR         46  ///< Kvaser Acquisitor (obsolete).
#define canHWTYPE_LEAF               48  ///< Kvaser Leaf Family
#define canHWTYPE_PC104_PLUS         50  ///< Kvaser PC104+
#define canHWTYPE_PCICANX_II         52  ///< Kvaser PCIcanx II
#define canHWTYPE_MEMORATOR_II       54  ///< Kvaser Memorator Professional family
#define canHWTYPE_MEMORATOR_PRO      54  ///< Kvaser Memorator Professional family
#define canHWTYPE_USBCAN_PRO         56  ///< Kvaser USBcan Professional
#define canHWTYPE_IRIS               58  ///< Obsolete name, use canHWTYPE_BLACKBIRD instead
#define canHWTYPE_BLACKBIRD          58  ///< Kvaser BlackBird
#define canHWTYPE_MEMORATOR_LIGHT    60  ///< Kvaser Memorator Light
#define canHWTYPE_MINIHYDRA          62  ///< Obsolete name, use canHWTYPE_EAGLE instead
#define canHWTYPE_EAGLE              62  ///< Kvaser Eagle family
#define canHWTYPE_BAGEL              64  ///< Obsolete name, use canHWTYPE_BLACKBIRD_V2 instead
#define canHWTYPE_BLACKBIRD_V2       64  ///< Kvaser BlackBird v2
#define canHWTYPE_MINIPCIE           66  ///< Kvaser Mini PCI Express
#define canHWTYPE_USBCAN_KLINE       68  ///< USBcan Pro HS/K-Line
#define canHWTYPE_ETHERCAN           70  ///< Kvaser Ethercan
#define canHWTYPE_USBCAN_LIGHT       72  ///< Kvaser USBcan Light
#define canHWTYPE_USBCAN_PRO2        74  ///< Kvaser USBcan Pro 5xHS and variants
#define canHWTYPE_PCIE_V2            76  ///< Kvaser PCIEcan 4xHS and variants
#define canHWTYPE_MEMORATOR_PRO2     78  ///< Kvaser Memorator Pro 5xHS and variants
#define canHWTYPE_LEAF2              80  ///< Kvaser Leaf Pro HS v2 and variants
#define canHWTYPE_MEMORATOR_V2       82  ///< Kvaser Memorator (2nd generation)


/** @} */

/**
 * \name canCHANNEL_CAP_xxx
 * \anchor canCHANNEL_CAP_xxx
 *
 * Channel capabilities.
 */
#define canCHANNEL_CAP_EXTENDED_CAN      0x00000001L ///< Can use extended identifiers
#define canCHANNEL_CAP_BUS_STATISTICS    0x00000002L ///< Can report busload etc
#define canCHANNEL_CAP_ERROR_COUNTERS    0x00000004L ///< Can return error counters
#define canCHANNEL_CAP_CAN_DIAGNOSTICS   0x00000008L ///< Can report CAN diagnostics
#define canCHANNEL_CAP_GENERATE_ERROR    0x00000010L ///< Can send error frames
#define canCHANNEL_CAP_GENERATE_OVERLOAD 0x00000020L ///< Can send CAN overload frame
#define canCHANNEL_CAP_TXREQUEST         0x00000040L ///< Can report when a CAN messsage transmission is initiated
#define canCHANNEL_CAP_TXACKNOWLEDGE     0x00000080L ///< Can report when a CAN messages has been transmitted
#define canCHANNEL_CAP_VIRTUAL           0x00010000L ///< Virtual CAN channel
#define canCHANNEL_CAP_SIMULATED         0x00020000L ///< Simulated CAN channel
#define canCHANNEL_CAP_REMOTE            0x00040000L ///< Remote CAN channel (e.g. BlackBird).
#define canCHANNEL_CAP_CAN_FD            0x00080000L ///< CAN-FD channel
#define canCHANNEL_CAP_CAN_FD_NONISO     0x00100000L ///< Supports Non-ISO CAN-FD channel

/** @} */

/**
 *  \name canDRIVER_CAP_xxx
 *  \anchor canDRIVER_CAP_xxx
 *
 *  Driver (transceiver) capabilities.
 *  @{
 */
/** Used with \ref canCHANNELDATA_TRANS_CAP */
#define canDRIVER_CAP_HIGHSPEED             0x00000001L
/** @} */

/**
 * \ingroup General
 * \name canIOCTL_xxx
 * \anchor canIOCTL_xxx
 *
 * These defines are used in \ref canIoCtl().
 *
 * @{
 */


  /**
   * This define is used in \ref canIoCtl(), \a buf and \a buflen refers to this
   * functions arguments.
   *
   * Tells CANLIB to "prefer" extended identifiers; that is, if you send a
   * message with \ref canWrite() and don't specify \ref canMSG_EXT nor \ref canMSG_STD,
   * \ref canMSG_EXT will be assumed. The contents of \a buf and \a buflen are
   * ignored. \ref canRead() et al will set \ref canMSG_EXT and/or \ref canMSG_STD as usual
   * and are not affected by this call.
   */
#define canIOCTL_PREFER_EXT             1

  /**
   * This define is used in \ref canIoCtl(), \a buf and \a buflen refers to this
   * functions arguments.
   *
   * Tells CANLIB to "prefer" standard identifiers; that is, if you send a
   * message with \ref canWrite() and don't specify \ref canMSG_EXT nor \ref canMSG_STD,
   * \ref canMSG_STD will be assumed. The contents of \a buf and \a buflen are
   * ignored. \ref canRead() et al will set \ref canMSG_EXT and/or \ref canMSG_STD as usual
   * and are not affected by this call.
   */
#define canIOCTL_PREFER_STD             2
// 3,4 reserved.

  /**
   * This define is used in \ref canIoCtl(), \a buf and \a buflen refers to this
   * functions arguments.
   *
   * Tells CANLIB to clear the CAN error counters. The contents of \a buf and \a
   * buflen are ignored.
   *
   * \note Not all CAN controllers support this operation (and if they don't,
   * nothing will happen.)
   */
#define canIOCTL_CLEAR_ERROR_COUNTERS   5

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * \a buf points to a DWORD which contains the desired time-stamp clock
   * resolution in microseconds. The default value is 1000 microseconds, i.e.
   * one millisecond.
   *
   * \note The accuracy of the clock isn't affected.
   */
#define canIOCTL_SET_TIMER_SCALE        6

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * \a buf points to a DWORD which contains
   *
   * \li 0: to turn Transmit Acknowledges off.
   * \li 1: to turn Transmit Acknowledges on.
   * \li 2: to turn Transmit Acknowledges off, even for the driver's internal
   * usage. This might enhance performance but will cause some other APIs to
   * stop working (for example, the current size of the transmit queue can not
   * be read when this mode is active.)
   *
   * The default value is 0, Transmit Acknowledge is off.
   */
#define canIOCTL_SET_TXACK              7

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * \a buf points at a \c DWORD which receives the current RX queue level. The
   * returned value is approximative (this is because not all hardware supports
   * retrieving the queue levels. In that case a best-effort guess is
   * returned. Also note that a device with embedded CPU will report its queue
   * levels to the host computer after a short delay that depends on the bus
   * traffic intensity, and consequently the value returned by the call to
   * \ref canIoCtl() might be a few milliseconds old.)
   */
#define canIOCTL_GET_RX_BUFFER_LEVEL              8

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * \a buf points at a \c DWORD which receives the current TX queue level. The
   * returned value is approximative (this is because not all hardware supports
   * retrieving the queue levels. In that case a best-effort guess is
   * returned. Also note that a device with embedded CPU will report its queue
   * levels to the host computer after a short delay that depends on the bus
   * traffic intensity, and consequently the value returned by the call to
   * \ref canIoCtl() might be a few milliseconds old.)
   */
#define canIOCTL_GET_TX_BUFFER_LEVEL              9

  /**
   * This define is used in \ref canIoCtl(), \a buf and \a buflen refers to this
   * functions arguments.
   *
   * Discard the current contents of the RX queue. The values of \a buf and \a
   * buflen are ignored.
   *
   * \note This is the same thing as calling \ref canFlushReceiveQueue()
   */
#define canIOCTL_FLUSH_RX_BUFFER                  10

  /**
   * This define is used in \ref canIoCtl(), \a buf and \a buflen refers to this
   * functions arguments.
   *
   * Discard the current contents of the TX queue. The values of \a buf and \a
   * buflen are ignored.
   *
   * \note This is the same thing as calling \ref canFlushTransmitQueue().
   */
#define canIOCTL_FLUSH_TX_BUFFER                  11

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * \a buf points to a \c DWORD which contains the desired time-stamp clock
   * resolution in microseconds. Note that the accuracy of the clock isn't
   * affected. The default value is 1000 microseconds, i.e. one millisecond.
   */
#define canIOCTL_GET_TIMER_SCALE                  12

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * \a buf points to a \c DWORD which contains
   *
   * \li \c 0 to turn Transmit Requests off.
   * \li \c 1 to turn Transmit Requests on.
   *
   * Default value is \c 0, Transmit Requests off.
   */
#define canIOCTL_SET_TXRQ                         13

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * \a buf points at a \c DWORD which receives a Windows Event handle which can
   * be passed to the Win32 API \c WaitForSingleObject. The event is signaled
   * when "something" (typically that a CAN message has been received or
   * transmitted) happens in the driver.
   *
   * \note There is no more information available as to what happened when this
   * call returns. The call may return on an "internal" event in CANLIB and your
   * application must be prepared to handle this (i.e. go to sleep again.)
   *
   * \win_start
   * \note If \ref canWaitForEvent() returns with success status (\ref canOK), you must call
   * \ref canRead() repeatedly until it returns \ref canERR_NOMSG, before calling
   * \ref canWaitForEvent() again. This will flush the driver's internal event queues.
   * Failure to call \ref canRead() can cause \ref canWaitForEvent() to get stuck in a state
   * where it always sleeps for the specified timeout and then returns with
   * \ref canERR_TIMEOUT.
   *
   * \sa \ref canWaitForEvent()
   * \win_end
   *
   * \note You must not set, reset, nor close this handle.  Waiting on it is
   *       the only supported operation.
   */
#define canIOCTL_GET_EVENTHANDLE                  14

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * \note Not yet implemented.
   */
#define canIOCTL_SET_BYPASS_MODE                  15

  /**
   * This define is used in \ref canIoCtl().
   *
   * This is only intended for internal use.
   */
#define canIOCTL_SET_WAKEUP                       16

#if defined(CANLIB_DECLARE_ALL)

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * \a buf points to a HANDLE which receives the Windows handle related to the
   * CANLIB handle.
   */
# define canIOCTL_GET_DRIVERHANDLE                17

  /**
   * This define is used in \ref canIoCtl().
   *
   * This is only intended for internal use.
   */
# define canIOCTL_MAP_RXQUEUE                     18

  /**
   * This define is used in \ref canIoCtl().
   *
   *  This is only intended for internal use.
   */
# define canIOCTL_GET_WAKEUP                      19

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * \a buf points to a BYTE which contains
   *
   * \li \c 0 to turn access error reporting off, and
   * \li \c 1 to turn access error reporting on.
   *
   * Default value is \c 0, access error reporting off.
   */
# define canIOCTL_SET_REPORT_ACCESS_ERRORS        20

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * \a buf points to a BYTE which receives the current setting of the access
   * error reporting (0 or 1.)
   */
# define canIOCTL_GET_REPORT_ACCESS_ERRORS        21

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * Connects the handle to the virtual bus number (0..31) which the \a buf
   * points to.
   */
# define canIOCTL_CONNECT_TO_VIRTUAL_BUS          22

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * Disonnects the handle from the virtual bus number (0..31) which the \a buf
   * points to.
   */
# define canIOCTL_DISCONNECT_FROM_VIRTUAL_BUS     23

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * \a buf points to a \ref canUserIoPortData struct that contains a port number
   * and a port value to set. This is used by special hardware only.
   */
# define canIOCTL_SET_USER_IOPORT                 24

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * \a buf points to a \ref canUserIoPortData struct that contains a port
   *  number. After the call, the struct will contain the current value of the
   *  I/O port. This is used by special hardware only.
   */
# define canIOCTL_GET_USER_IOPORT                 25

  /**
   * This define is used in \ref canIoCtl().
   *
   *  This is only intended for internal use.
   */
# define canIOCTL_SET_BUFFER_WRAPAROUND_MODE      26

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * Use this function code to set the size of the receive buffer for a
   * specific handle. \a buf points to an unsigned integer which contains the
   * new size (number of messages) of the receive buffer.
   *
   * \note The receive buffer consumes system nonpaged pool memory, which is a
   *       limited resource. Do not increase the receive buffer size unless you
   *       have good reasons to do so.
   *
   * \note You can't use this function code when the channel is on bus.
   */
# define canIOCTL_SET_RX_QUEUE_SIZE               27

  /**
   * This define is used in \ref canIoCtl().
   *
   *  This is only intended for internal use.
   */
# define canIOCTL_SET_USB_THROTTLE                28

  /**
   * This define is used in \ref canIoCtl().
   *
   *  This is only intended for internal use.
   */
# define canIOCTL_GET_USB_THROTTLE                29

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * \a buf points to a DWORD. If the value is zero, the CAN clock will not be
   * reset at buson for the handle. Otherwise, the CAN clock will be reset at
   * buson.
   *
   * Default value is \c 1, the CAN clock will be reset at buson.
   */
# define canIOCTL_SET_BUSON_TIME_AUTO_RESET       30

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * Returns the state of the Transmit Acknowledge as a DWORD in \a buf:
   *
   * \li 0: Transmit Acknowledges is turned off.
   * \li 1: Transmit Acknowledges is turned on.
   * \li 2: Transmit Acknowledges is turned off, even for the driver's internal
   *        usage.
   */
# define canIOCTL_GET_TXACK                       31

  /**
   * This define is used in \ref canIoCtl(), \a buf mentioned below refers to this
   * functions argument.
   *
   * \a buf points to an unsigned byte. If the value is zero, the local transmit
   * echo is turned off for the handle. Otherwise, local transmit echo is turned
   * on.
   *
   * Local transmit echo is turned on by default on all handles.  This means
   * that if two handles are open on the same channel, and a message is
   * transmitted on the first handle, it will be received as a normal message
   * on the second handle. Use the \ref canIOCTL_SET_LOCAL_TXECHO function code to
   * turn this function off, if it is not desired on a certain handle.
   */
# define canIOCTL_SET_LOCAL_TXECHO                32

#endif
 /** @} */

#if defined(CANLIB_DECLARE_ALL)
/** Used in \ref canIOCTL_SET_USER_IOPORT and \ref canIOCTL_GET_USER_IOPORT. */
typedef struct {
  unsigned int portNo;     ///< Port number used in e.g. \ref canIOCTL_SET_USER_IOPORT
  unsigned int portValue;  ///< Port value used in e.g. \ref canIOCTL_SET_USER_IOPORT
} canUserIoPortData;

#endif

#if defined(CANLIB_DECLARE_ALL)
#endif

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canSetBusParamsC200(int hnd, byte btr0, byte btr1);</b>
 *
 * \source_delphi   <b>function canSetBusParamsC200(hnd: canHandle; btr0, btr1: byte): canStatus;     </b>
 * \source_end
 *
 * This function sets the bus timing parameters using the same
 * convention as the 82c200 CAN controller (which is the same as many
 * other CAN controllers, for example, the 82527.)
 *
 * To calculate the bit timing parameters, you can use the bit timing
 * calculator that is included with CANLIB SDK. Look in the BIN directory.
 *
 * 82c200 Bit Timing
 *
 * \li \a btr0 [b7..b6]: SJW - 1
 * \li \a btr0 [b5..b0]: Prescaler -1
 * \li \a btr1 [b7]: \c 1: 3 samples, \c 0: 1 samples
 * \li \a btr1 [b6..b4]: tseg2 - 1
 * \li \a btr1 [b3..b0]: tseg1 - 2
 *
 * \note CANLIB will always behave as if the clock frequency is 16 MHz. It does
 * not matter if the device has a different physical clock, since this will be
 * compensated for by the driver.
 *
 * \param[in] hnd   A handle to an open CAN circuit.
 * \param[in] btr0  The desired bit timing, formatted as the contents of the
 *                  BTR0 register in the 82c200.
 * \param[in] btr1  The desired bit timing, formatted as the contents of the
 *                  BTR1 register in the 82c200.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 *
 * \sa \ref page_code_snippets_bit_rate, \ref page_user_guide_misc_bitrate
 * \sa \ref canSetBusParams()
 */
canStatus CANLIBAPI canSetBusParamsC200 (const CanHandle hnd, BYTE btr0, BYTE btr1);

#if defined(CANLIB_DECLARE_ALL)
#endif


#if defined(CANLIB_DECLARE_ALL)
/**
 * \ingroup ObjectBuffers
 *
 * \source_cs       <b>static Canlib.canStatus canObjBufFreeAll(int handle);</b>
 *
 * \source_delphi   <b>function canObjBufFreeAll(handle: canHandle): canStatus;     </b>
 * \source_end
 *
 * Deallocates all object buffers on the specified handle. The
 * buffers cannot be referenced after this operation.
 *
 * \param[in]  hnd  An open handle to a CAN circuit.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_user_guide_send_recv_obj_buf
 * \sa \ref canObjBufFree(), \ref canObjBufAllocate()
 */
canStatus CANLIBAPI canObjBufFreeAll (const CanHandle hnd);

/**
 * \ingroup ObjectBuffers
 *
 * \source_cs       <b>static Canlib.canStatus canObjBufAllocate(int handle, int type);</b>
 *
 * \source_delphi   <b>function canObjBufAllocate(handle: canHandle; tp: Integer): canStatus;     </b>
 * \source_end
 *
 * Allocates an object buffer associated with a handle to a CAN
 * circuit.
 *
 * \param[in] hnd   An open handle to a CAN circuit.
 * \param[in] type  The type of the buffer. Must be one of \ref canOBJBUF_TYPE_xxx
 *
 * \return A buffer index (zero or positive) if success.
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_user_guide_send_recv_obj_buf
 * \sa \ref canObjBufFree(), \ref canObjBufFreeAll()
 */
canStatus CANLIBAPI canObjBufAllocate (const CanHandle hnd, int type);

 /**
  * \name canOBJBUF_TYPE_xxx
  * \anchor canOBJBUF_TYPE_xxx
  *
  * Used in \ref canObjBufAllocate().
  *
  * @{
  */
#define canOBJBUF_TYPE_AUTO_RESPONSE            0x01 ///< The buffer is an auto-response buffer.
#define canOBJBUF_TYPE_PERIODIC_TX              0x02 ///< The buffer is an auto-transmit buffer.
 /** @} */

/**
 * \ingroup ObjectBuffers
 *
 * \source_cs       <b>static Canlib.canStatus canObjBufFree(int handle, int idx);</b>
 *
 * \source_delphi   <b>function canObjBufFree(handle: canHandle; idx: Integer): canStatus;      </b>
 * \source_end
 *
 * Deallocates the object buffer with the specified index. The buffer
 * can not be referenced after this operation.
 *
 * \param[in] hnd  An open handle to a CAN circuit.
 * \param[in] idx  The object buffer to deallocate.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_user_guide_send_recv_obj_buf
 * \sa \ref canObjBufFreeAll(), \ref canObjBufAllocate(),
 */
canStatus CANLIBAPI canObjBufFree (const CanHandle hnd, int idx);

// Writes CAN data to the object buffer with the specified index.

/**
 * \ingroup ObjectBuffers
 *
 * \source_cs       <b>static Canlib.canStatus canObjBufWrite(int handle, int idx, int id, byte[] msg, int dlc, int flags);</b>
 *
 * \source_delphi   <b>function canObjBufWrite(handle: canHandle; idx, id: Integer; var msg; dlc, flags: cardinal): canStatus;     </b>
 * \source_end
 *
 * Defines the contents of a specific object buffer.
 *
 * \param[in] hnd   An open handle to a CAN circuit.
 * \param[in] idx   The index of the object buffer whose contents is to be
 *                  defined.
 * \param[in] id    The CAN identifier of the message.
 * \param[in] msg   Points to the contents of the message.
 * \param[in] dlc   The length of the message in bytes.<br>
                    For Classic CAN dlc can be at most 8, unless \ref canOPEN_ACCEPT_LARGE_DLC is used.<br>
                    For CAN FD dlc can be one of the following 0-8, 12, 16, 20, 24, 32, 48, 64.
 * \param[in] flags Message flags; a combination of the \ref canMSG_xxx flags.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_user_guide_send_recv_obj_buf
 */
canStatus CANLIBAPI canObjBufWrite (const CanHandle hnd,
                                    int idx,
                                    int id,
                                    void* msg,
                                    unsigned int dlc,
                                    unsigned int flags);

/**
 * \ingroup ObjectBuffers
 *
 * \source_cs       <b>static Canlib.canStatus canObjBufSetFilter(int handle, int idx, int code, int mask);</b>
 *
 * \source_delphi   <b>function canObjBufSetFilter(handle: canHandle; idx: Integer; code, mask: Cardinal): canStatus;      </b>
 * \source_end
 *
 * Defines a message reception filter on the specified object buffer.
 * Messages not matching the filter are discarded.
 *
 * \note For an auto response buffer, set the code and mask that together define
 * the identifier(s) that trigger(s) the automatic response.
 *
 * \param[in] hnd   An open handle to a CAN circuit.
 * \param[in] idx   The index of the object buffer on which the filter is to be
 *                  set.
 * \param[in] code  The acceptance code in the filter.
 * \param[in] mask  The acceptance mask in the filter.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_user_guide_misc_code_and_mask,
 *     \ref page_user_guide_send_recv_obj_buf
 */
canStatus CANLIBAPI canObjBufSetFilter (const CanHandle hnd,
                                        int idx,
                                        unsigned int code,
                                        unsigned int mask);

/**
 * \ingroup ObjectBuffers
 *
 * \source_cs       <b>static Canlib.canStatus canObjBufSetFlags(int handle, int idx, int flags);</b>
 *
 * \source_delphi   <b>function canObjBufSetFlags(handle: canHandle; idx: Integer; flags: Cardinal): canStatus;     </b>
 * \source_end
 *
 * Sets object buffer flags on a specified object buffer.
 *
 * \param[in] hnd    An open handle to a CAN circuit.
 * \param[in] idx    The buffer on which the flags are to be set.
 * \param[in] flags  Specifies a combination of zero or more of the
 *                   \ref canOBJBUF_AUTO_RESPONSE_xxx flag values
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_user_guide_send_recv_obj_buf
 */
canStatus CANLIBAPI canObjBufSetFlags (const CanHandle hnd,
                                       int idx,
                                       unsigned int flags);

/**
 * \name canOBJBUF_AUTO_RESPONSE_xxx
 * \anchor canOBJBUF_AUTO_RESPONSE_xxx
 *
 * These defines are used in \ref canObjBufSetFlags().
 *
 * @{
 */
 /**
  * This define is used in \ref canObjBufSetFlags().
  *
  * For auto-response buffers only. When this flag is in effect, the buffer
  * will auto-respond to remote requests only.  If this flag is not in effect,
  * the buffer will auto-respond to both remote requests and ordinary data
  * frames.
  *
  */
# define canOBJBUF_AUTO_RESPONSE_RTR_ONLY        0x01
 /** @} */

/**
 * \ingroup ObjectBuffers
 *
 * \source_cs       <b>static Canlib.canStatus canObjBufSetPeriod(int hnd, int idx, int period);</b>
 *
 * \source_delphi   <b>function canObjBufSetPeriod(handle: canHandle; idx: Integer; period: Cardinal): canStatus;     </b>
 * \source_end
 *
 * The \ref canObjBufSetPeriod function sets the transmission period for an auto
 * transmission object buffer.
 *
 * \param[in] hnd     An open handle to a CAN channel.
 * \param[in] idx     The index of a CAN object buffer.
 * \param[in] period  The transmission interval, in microseconds.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_user_guide_send_recv_obj_buf
 */
canStatus CANLIBAPI canObjBufSetPeriod (const CanHandle hnd,
                                        int idx,
                                        unsigned int period);

/**
 * \ingroup ObjectBuffers
 *
 * \source_cs       <b>static Canlib.canStatus canObjBufSetMsgCount(int hnd, int idx, int count);</b>
 *
 * \source_delphi   <b>function canObjBufSetMsgCount(handle: canHandle; idx: Integer; count: Cardinal): canStatus;     </b>
 * \source_end
 *
 * The \ref canObjBufSetMsgCount function sets the message count for an auto
 * transmit object buffer.
 *
 * \param[in] hnd    An open handle to a CAN channel.
 * \param[in] idx    The index of a CAN object buffer.
 * \param[in] count  The message count.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_user_guide_send_recv_obj_buf
 */
canStatus CANLIBAPI canObjBufSetMsgCount (const CanHandle hnd,
                                          int idx,
                                          unsigned int count);

/**
 * \ingroup ObjectBuffers
 *
 * \source_cs       <b>static Canlib.canStatus canObjBufEnable(int handle, int idx);</b>
 *
 * \source_delphi   <b>function canObjBufEnable(handle: canHandle; idx: Integer): canStatus;     </b>
 * \source_end
 *
 * Enables the object buffer with the specified index.
 *
 * \param[in] hnd  An open handle to a CAN circuit.
 * \param[in] idx  The index of the object buffer to enable.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_user_guide_send_recv_obj_buf
 * \sa \ref canObjBufDisable()
 */
canStatus CANLIBAPI canObjBufEnable (const CanHandle hnd, int idx);

/**
 * \ingroup ObjectBuffers
 *
 * \source_cs       <b>static Canlib.canStatus canObjBufDisable(int handle, int idx);</b>
 *
 * \source_delphi   <b>function canObjBufDisable(handle: canHandle; idx: Integer): canStatus;     </b>
 * \source_end
 *
 * Disables the object buffer with the specified index.
 *
 * \param[in] hnd  An open handle to a CAN circuit.
 * \param[in] idx  The index of the buffer.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_user_guide_send_recv_obj_buf
 * \sa \ref canObjBufEnable()
 */
canStatus CANLIBAPI canObjBufDisable (const CanHandle hnd, int idx);

/**
 * \ingroup ObjectBuffers
 *
 * \source_cs       <b>static Canlib.canStatus canObjBufSendBurst(int hnd, int idx, int burstlen);</b>
 *
 * \source_delphi   <b>function canObjBufSendBurst(handle: canHandle; idx: Integer; burstLen: Cardinal): canStatus;      </b>
 * \source_end
 *
 * The canObjBufSendBurst function sends a burst of CAN messages. You have to
 * set up an object buffer first with the message to send. The messages will be
 * sent as fast as possible from the hardware.
 *
 * This function is inteneded for certain diagnostic applications.
 *
 * \param[in] hnd       An open handle to a CAN channel.
 * \param[in] idx       The index of a CAN object buffer.
 * \param[in] burstlen  The number of messages to send.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref page_user_guide_send_recv_obj_buf
 */
canStatus CANLIBAPI canObjBufSendBurst (const CanHandle hnd,
                                        int idx,
                                        unsigned int burstlen);

#endif

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canResetBus(int handle);</b>
 *
 * \source_delphi   <b>function canResetBus(handle: canHandle): canStatus;     </b>
 * \source_end
 *
 * This function tries to reset a CAN bus controller by taking the channel off
 * bus and then on bus again (if it was on bus before the call to \ref canResetBus().)
 *
 * This function will affect the hardware (and cause a real reset of the CAN
 * chip) only if \a hnd is the only handle open on the channel. If there
 * are other open handles, this operation will not affect the hardware.
 *
 * \param[in] hnd  A handle to an open circuit.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref canBusOn(), \ref canBusOff()
 */
canStatus CANLIBAPI canResetBus (const CanHandle hnd);

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canWriteWait(int handle, int id, byte[] msg, int dlc, int flag, long timeout);</b>
 *
 * \source_delphi   <b>function canWriteWait(handle: canHandle; id: longint; var msg; dlc, flag, timeout : Cardinal): canStatus;     </b>
 * \source_end
 *
 * This function sends a CAN message. It returns when the message is sent, or
 * the timeout expires.
 *
 * This is a convenience function that combines \ref canWrite() and \ref canWriteSync().
 *
 * If you are using the same channel via multiple handles, note that the
 * default behaviour is that the different handles will "hear" each other just as
 * if each handle referred to a channel of its own. If you open, say, channel 0
 * from thread A and thread B and then send a message from thread A, it will be
 * "received" by thread B.
 * This behaviour can be changed using \ref canIOCTL_SET_LOCAL_TXECHO.
 *
 * \param[in]  hnd       A handle to an open CAN circuit.
 * \param[in]  id        The identifier of the CAN message to send.
 * \param[in]  msg       A pointer to the message data, or \c NULL.
 * \param[in]  dlc       The length of the message in bytes.<br>
                         For Classic CAN dlc can be at most 8, unless \ref canOPEN_ACCEPT_LARGE_DLC is used.<br>
                         For CAN FD dlc can be one of the following 0-8, 12, 16, 20, 24, 32, 48, 64.
 * \param[in]  flag      A combination of message flags, \ref canMSG_xxx.
 *                       Use this parameter to send extended (29-bit) frames
 *                       and/or remote frames. Use \ref canMSG_EXT and/or
 *                       \ref canMSG_RTR for this purpose.
 * \param[in] timeout    The timeout, in milliseconds. 0xFFFFFFFF gives an
 *                       infinite timeout.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 */
canStatus CANLIBAPI canWriteWait (const CanHandle hnd,
                                  long id,
                                  void *msg,
                                  unsigned int dlc,
                                  unsigned int flag,
                                  unsigned long timeout);


#if defined(CANLIB_DECLARE_ALL)

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canFlushReceiveQueue(int hnd);</b>
 *
 * \source_delphi   <b>function canFlushReceiveQueue(handle: canHandle): canStatus;     </b>
 * \source_end
 *
 * This function removes all received messages from the handle's receive queue.
 * Other handles open to the same channel are not affcted by this
 * operation. That is, only the messages belonging to the handle you are
 * passing to \ref canFlushReceiveQueue are discarded.
 *
 * \note This call has the same effect as calling \ref canIoCtl() with a function
 * code of \ref canIOCTL_FLUSH_RX_BUFFER.
 *
 * \param[in] hnd  A handle to an open circuit.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref canFlushTransmitQueue()
 */
canStatus CANLIBAPI canFlushReceiveQueue (const CanHandle hnd);

/**
 * \ingroup CAN
 *
 * \source_cs       <b>static Canlib.canStatus canFlushTransmitQueue(int hnd);</b>
 *
 * \source_delphi   <b>function canFlushTransmitQueue(handle: canHandle): canStatus;     </b>
 * \source_end
 *
 * This function removes all messages pending transmission from the
 * transmit queue of the circuit.
 *
 * \note If there are other handles open to the same circuit, they are also
 * flushed.
 *
 * \note This call has the same effect as calling \ref canIoCtl() with a function
 * code of \ref canIOCTL_FLUSH_TX_BUFFER.
 *
 * \param[in] hnd  A handle to an open circuit.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref canFlushReceiveQueue()
 */
canStatus CANLIBAPI canFlushTransmitQueue (const CanHandle hnd);


/** Contains status codes according to \ref canSTAT_xxx. */
typedef canStatus kvStatus;

/**
 * \name kvCallback_t
 * \anchor kvCallback_t
 * \ref kvCallback_t is used by the function \ref kvSetNotifyCallback()
 *
 * The callback function is called with the following arguments:
 * \li hnd - the handle of the CAN channel where the event happened.
 * \li context - the context pointer you passed to \ref kvSetNotifyCallback().
 * \li notifyEvent - one of the \ref canNOTIFY_xxx notification codes.
 *
 * \note It is really the \ref canNOTIFY_xxx codes, and not the
 *  \ref canEVENT_xxx codes that the \ref canSetNotify() API is using.
 *
 * \param[in] hnd          An open handle to a CAN channel.
 * \param[in] context      Arbitrary user-defined context data which
 *                         is passed to the callback function.
 * \param[in] notifyEvent  One or more of the \ref canEVENT_xxx flags.
 *
 */
typedef void (CANLIBAPI *kvCallback_t) (CanHandle hnd, void* context, unsigned int notifyEvent);

/**
 * \ingroup General
 *
 * \source_cs       <b>static Canlib.canStatus kvSetNotifyCallback(int hnd, Canlib.kvCallbackDelegate callback, IntPtr context, uint notifyFlags);</b>
 *
 * \source_delphi   <b>function kvSetNotifyCallback(handle: canHandle; callback: kvCallback_t; context: Pointer; notifyFlags: Cardinal): canStatus;     </b>
 * \source_end
 *
 * The \ref kvSetNotifyCallback() function registers a callback function which is
 * called when certain events occur.
 *
 * You can register at most one callback function per handle at any time.
 *
 * To remove the callback, call \ref kvSetNotifyCallback() with a \c NULL pointer in
 * the callback argument.
 *
 * \note The callback function is called in the context of a high-priority
 * thread created by CANLIB. You should take precaution not to do any time
 * consuming tasks in the callback.  You must also arrange the synchronization
 * between the callback and your other threads yourself.
 *
 * \param[in] hnd          An open handle to a CAN channel.
 * \param[in] callback     A pointer to a callback function of type
 *                         \ref kvCallback_t
 * \param[in] context      A pointer to arbitrary user-defined context data which
 *                         is passed to the callback function.
 * \param[in] notifyFlags  One or more of the \ref canNOTIFY_xxx flags.
 *
 * \return \ref canOK (zero) if success
 * \return \ref canERR_xxx (negative) if failure
 *
 * \sa \ref canSetNotify()
 */
kvStatus CANLIBAPI kvSetNotifyCallback (const CanHandle hnd,
                                        kvCallback_t callback,
                                        void* context,
                                        unsigned int notifyFlags);

/** @} */
#endif

#ifdef __cplusplus
}
#endif



#endif
