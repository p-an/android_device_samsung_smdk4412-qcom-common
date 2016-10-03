#ifndef ANDROID_INCLUDE_HARDWARE_GPS_H
#define ANDROID_INCLUDE_HARDWARE_GPS_H

#include <stdint.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include <pthread.h>
#include <sys/socket.h>
#include <stdbool.h>

#include <hardware/hardware.h>

__BEGIN_DECLS

/**
  * The id of this module
*/
#define GPS_HARDWARE_MODULE_ID "gps"

#define GNSS_MAX_SVS 64

#define GNSS_MAX_MEASUREMENT   64

typedef uint8_t                         GnssConstellationType;
#define GNSS_CONSTELLATION_UNKNOWN      0
#define GNSS_CONSTELLATION_GPS          1
#define GNSS_CONSTELLATION_SBAS         2
#define GNSS_CONSTELLATION_GLONASS      3
#define GNSS_CONSTELLATION_QZSS         4
#define GNSS_CONSTELLATION_BEIDOU       5
#define GNSS_CONSTELLATION_GALILEO      6


typedef uint8_t                                 GnssSvFlags;
#define GNSS_SV_FLAGS_NONE                      0
#define GNSS_SV_FLAGS_HAS_EPHEMERIS_DATA        (1 << 0)
#define GNSS_SV_FLAGS_HAS_ALMANAC_DATA          (1 << 1)
#define GNSS_SV_FLAGS_USED_IN_FIX               (1 << 2)

typedef uint16_t GnssClockFlags;
/** A valid 'leap second' is stored in the data structure. */
#define GNSS_CLOCK_HAS_LEAP_SECOND               (1<<0)
/** A valid 'time uncertainty' is stored in the data structure. */
#define GNSS_CLOCK_HAS_TIME_UNCERTAINTY          (1<<1)
/** A valid 'full bias' is stored in the data structure. */
#define GNSS_CLOCK_HAS_FULL_BIAS                 (1<<2)
/** A valid 'bias' is stored in the data structure. */
#define GNSS_CLOCK_HAS_BIAS                      (1<<3)
/** A valid 'bias uncertainty' is stored in the data structure. */
#define GNSS_CLOCK_HAS_BIAS_UNCERTAINTY          (1<<4)
/** A valid 'drift' is stored in the data structure. */
#define GNSS_CLOCK_HAS_DRIFT                     (1<<5)
/** A valid 'drift uncertainty' is stored in the data structure. */
#define GNSS_CLOCK_HAS_DRIFT_UNCERTAINTY         (1<<6)

typedef uint32_t GnssMeasurementFlags;
/** A valid 'snr' is stored in the data structure. */
#define GNSS_MEASUREMENT_HAS_SNR                               (1<<0)
/** A valid 'carrier frequency' is stored in the data structure. */
#define GNSS_MEASUREMENT_HAS_CARRIER_FREQUENCY                 (1<<9)
/** A valid 'carrier cycles' is stored in the data structure. */
#define GNSS_MEASUREMENT_HAS_CARRIER_CYCLES                    (1<<10)
/** A valid 'carrier phase' is stored in the data structure. */
#define GNSS_MEASUREMENT_HAS_CARRIER_PHASE                     (1<<11)
/** A valid 'carrier phase uncertainty' is stored in the data structure. */
#define GNSS_MEASUREMENT_HAS_CARRIER_PHASE_UNCERTAINTY         (1<<12)


typedef uint32_t GnssMeasurementState;
#define GNSS_MEASUREMENT_STATE_UNKNOWN                   0
#define GNSS_MEASUREMENT_STATE_CODE_LOCK             (1<<0)
#define GNSS_MEASUREMENT_STATE_BIT_SYNC              (1<<1)
#define GNSS_MEASUREMENT_STATE_SUBFRAME_SYNC         (1<<2)
#define GNSS_MEASUREMENT_STATE_TOW_DECODED           (1<<3)
#define GNSS_MEASUREMENT_STATE_MSEC_AMBIGUOUS        (1<<4)
#define GNSS_MEASUREMENT_STATE_SYMBOL_SYNC           (1<<5)
#define GNSS_MEASUREMENT_STATE_GLO_STRING_SYNC       (1<<6)
#define GNSS_MEASUREMENT_STATE_GLO_TOD_DECODED       (1<<7)
#define GNSS_MEASUREMENT_STATE_BDS_D2_BIT_SYNC       (1<<8)
#define GNSS_MEASUREMENT_STATE_BDS_D2_SUBFRAME_SYNC  (1<<9)
#define GNSS_MEASUREMENT_STATE_GAL_E1BC_CODE_LOCK    (1<<10)
#define GNSS_MEASUREMENT_STATE_GAL_E1C_2ND_CODE_LOCK (1<<11)
#define GNSS_MEASUREMENT_STATE_GAL_E1B_PAGE_SYNC     (1<<12)
#define GNSS_MEASUREMENT_STATE_SBAS_SYNC             (1<<13)


typedef uint16_t GnssAccumulatedDeltaRangeState;
#define GNSS_ADR_STATE_UNKNOWN                       0
#define GNSS_ADR_STATE_VALID                     (1<<0)
#define GNSS_ADR_STATE_RESET                     (1<<1)
#define GNSS_ADR_STATE_CYCLE_SLIP                (1<<2)

typedef uint8_t GnssMultipathIndicator;
/** The indicator is not available or unknown. */
#define GNSS_MULTIPATH_INDICATOR_UNKNOWN                 0
/** The measurement is indicated to be affected by multipath. */
#define GNSS_MULTIPATH_INDICATOR_PRESENT                 1
/** The measurement is indicated to be not affected by multipath. */
#define GNSS_MULTIPATH_INDICATOR_NOT_PRESENT             2

typedef int16_t GnssNavigationMessageType;

#define GNSS_NAVIGATION_MESSAGE_TYPE_UNKNOWN       0
/** GPS L1 C/A message contained in the structure.  */
#define GNSS_NAVIGATION_MESSAGE_TYPE_GPS_L1CA      0x0101
/** GPS L2-CNAV message contained in the structure. */
#define GNSS_NAVIGATION_MESSAGE_TYPE_GPS_L2CNAV    0x0102
/** GPS L5-CNAV message contained in the structure. */
#define GNSS_NAVIGATION_MESSAGE_TYPE_GPS_L5CNAV    0x0103
/** GPS CNAV-2 message contained in the structure. */
#define GNSS_NAVIGATION_MESSAGE_TYPE_GPS_CNAV2     0x0104
/** Glonass L1 CA message contained in the structure. */
#define GNSS_NAVIGATION_MESSAGE_TYPE_GLO_L1CA      0x0301
/** Beidou D1 message contained in the structure. */
#define GNSS_NAVIGATION_MESSAGE_TYPE_BDS_D1        0x0501
/** Beidou D2 message contained in the structure. */
#define GNSS_NAVIGATION_MESSAGE_TYPE_BDS_D2        0x0502
/** Galileo I/NAV message contained in the structure. */
#define GNSS_NAVIGATION_MESSAGE_TYPE_GAL_I         0x0601
/** Galileo F/NAV message contained in the structure. */
#define GNSS_NAVIGATION_MESSAGE_TYPE_GAL_F         0x0602

typedef uint16_t NavigationMessageStatus;
#define NAV_MESSAGE_STATUS_UNKNOWN              0
#define NAV_MESSAGE_STATUS_PARITY_PASSED   (1<<0)
#define NAV_MESSAGE_STATUS_PARITY_REBUILT  (1<<1)

/* This constant is deprecated, and will be removed in the next release. */
#define NAV_MESSAGE_STATUS_UNKONW              0



typedef struct {
    /** set to sizeof(GnssSvInfo) */
    size_t size;

    /**
     * Pseudo-random number for the SV, or FCN/OSN number for Glonass. The
     * distinction is made by looking at constellation field. Values should be
     * in the range of:
     *
     * - GPS:     1-32
     * - SBAS:    120-151, 183-192
     * - GLONASS: 1-24, the orbital slot number (OSN), if known.  Or, if not:
     *            93-106, the frequency channel number (FCN) (-7 to +6) offset by + 100
     *            i.e. report an FCN of -7 as 93, FCN of 0 as 100, and FCN of +6 as 106.
     * - QZSS:    193-200
     * - Galileo: 1-36
     * - Beidou:  1-37
     */
    int16_t svid;

    /**
     * Defines the constellation of the given SV. Value should be one of those
     * GNSS_CONSTELLATION_* constants
     */
    GnssConstellationType constellation;

    /**
     * Carrier-to-noise density in dB-Hz, typically in the range [0, 63].
     * It contains the measured C/N0 value for the signal at the antenna port.
     *
     * This is a mandatory value.
     */
    float c_n0_dbhz;

    /** Elevation of SV in degrees. */
    float elevation;

    /** Azimuth of SV in degrees. */
    float azimuth;

    /**
     * Contains additional data about the given SV. Value should be one of those
     * GNSS_SV_FLAGS_* constants
     */
    GnssSvFlags flags;

} GnssSvInfo;

typedef struct {
    /** set to sizeof(GnssSvStatus) */
    size_t size;

    /** Number of GPS SVs currently visible, refers to the SVs stored in sv_list */
    int num_svs;
    /**
     * Pointer to an array of SVs information for all GNSS constellations,
     * except GPS, which is reported using sv_list
     */
    GnssSvInfo gnss_sv_list[GNSS_MAX_SVS];

} GnssSvStatus;

typedef struct {
    /** Set to sizeof(GnssSystemInfo) */
    size_t   size;
    /* year in which the last update was made to the underlying hardware/firmware
     * used to capture GNSS signals, e.g. 2016 */
    uint16_t year_of_hw;
} GnssSystemInfo;


typedef struct {
    /** set to sizeof(GnssClock) */
    size_t size;

    /**
     * A set of flags indicating the validity of the fields in this data
     * structure.
     */
    GnssClockFlags flags;

    /**
     * Leap second data.
     * The sign of the value is defined by the following equation:
     *      utc_time_ns = time_ns - (full_bias_ns + bias_ns) - leap_second *
     *      1,000,000,000
     *
     * If the data is available 'flags' must contain GNSS_CLOCK_HAS_LEAP_SECOND.
     */
    int16_t leap_second;

    /**
     * The GNSS receiver internal clock value. This is the local hardware clock
     * value.
     *
     * For local hardware clock, this value is expected to be monotonically
     * increasing while the hardware clock remains power on. (For the case of a
     * HW clock that is not continuously on, see the
     * hw_clock_discontinuity_count field). The receiver's estimate of GPS time
     * can be derived by substracting the sum of full_bias_ns and bias_ns (when
     * available) from this value.
     *
     * This GPS time is expected to be the best estimate of current GPS time
     * that GNSS receiver can achieve.
     *
     * Sub-nanosecond accuracy can be provided by means of the 'bias_ns' field.
     * The value contains the 'time uncertainty' in it.
     *
     * This field is mandatory.
     */
    int64_t time_ns;

    /**
     * 1-Sigma uncertainty associated with the clock's time in nanoseconds.
     * The uncertainty is represented as an absolute (single sided) value.
     *
     * If the data is available, 'flags' must contain
     * GNSS_CLOCK_HAS_TIME_UNCERTAINTY. This value is effectively zero (it is
     * the reference local clock, by which all other times and time
     * uncertainties are measured.)  (And thus this field can be not provided,
     * per GNSS_CLOCK_HAS_TIME_UNCERTAINTY flag, or provided & set to 0.)
     */
    double time_uncertainty_ns;
    /**
     * The difference between hardware clock ('time' field) inside GPS receiver
     * and the true GPS time since 0000Z, January 6, 1980, in nanoseconds.
     *
     * The sign of the value is defined by the following equation:
     *      local estimate of GPS time = time_ns - (full_bias_ns + bias_ns)
     *
     * This value is mandatory if the receiver has estimated GPS time. If the
     * computed time is for a non-GPS constellation, the time offset of that
     * constellation to GPS has to be applied to fill this value. The error
     * estimate for the sum of this and the bias_ns is the bias_uncertainty_ns,
     * and the caller is responsible for using this uncertainty (it can be very
     * large before the GPS time has been solved for.) If the data is available
     * 'flags' must contain GNSS_CLOCK_HAS_FULL_BIAS.
     */
    int64_t full_bias_ns;

    /**
     * Sub-nanosecond bias.
     * The error estimate for the sum of this and the full_bias_ns is the
     * bias_uncertainty_ns
     *
     * If the data is available 'flags' must contain GNSS_CLOCK_HAS_BIAS. If GPS
     * has computed a position fix. This value is mandatory if the receiver has
     * estimated GPS time.
     */
    double bias_ns;

    /**
     * 1-Sigma uncertainty associated with the local estimate of GPS time (clock
     * bias) in nanoseconds. The uncertainty is represented as an absolute
     * (single sided) value.
     *
     * If the data is available 'flags' must contain
     * GNSS_CLOCK_HAS_BIAS_UNCERTAINTY. This value is mandatory if the receiver
     * has estimated GPS time.
     */
    double bias_uncertainty_ns;

    /**
     * The clock's drift in nanoseconds (per second).
     *
     * A positive value means that the frequency is higher than the nominal
     * frequency, and that the (full_bias_ns + bias_ns) is growing more positive
     * over time.
     *
     * The value contains the 'drift uncertainty' in it.
     * If the data is available 'flags' must contain GNSS_CLOCK_HAS_DRIFT.
     *
     * This value is mandatory if the receiver has estimated GNSS time
     */
    double drift_nsps;

    /**
     * 1-Sigma uncertainty associated with the clock's drift in nanoseconds (per second).
     * The uncertainty is represented as an absolute (single sided) value.
     *
     * If the data is available 'flags' must contain
     * GNSS_CLOCK_HAS_DRIFT_UNCERTAINTY. If GPS has computed a position fix this
     * field is mandatory and must be populated.
     */
    double drift_uncertainty_nsps;
   /**
     * When there are any discontinuities in the HW clock, this field is
     * mandatory.
     *
     * A "discontinuity" is meant to cover the case of a switch from one source
     * of clock to another.  A single free-running crystal oscillator (XO)
     * should generally not have any discontinuities, and this can be set and
     * left at 0.
     *
     * If, however, the time_ns value (HW clock) is derived from a composite of
     * sources, that is not as smooth as a typical XO, or is otherwise stopped &
     * restarted, then this value shall be incremented each time a discontinuity
     * occurs.  (E.g. this value may start at zero at device boot-up and
     * increment each time there is a change in clock continuity. In the
     * unlikely event that this value reaches full scale, rollover (not
     * clamping) is required, such that this value continues to change, during
     * subsequent discontinuity events.)
     *
     * While this number stays the same, between GnssClock reports, it can be
     * safely assumed that the time_ns value has been running continuously, e.g.
     * derived from a single, high quality clock (XO like, or better, that's
     * typically used during continuous GNSS signal sampling.)
     *
     * It is expected, esp. during periods where there are few GNSS signals
     * available, that the HW clock be discontinuity-free as long as possible,
     * as this avoids the need to use (waste) a GNSS measurement to fully
     * re-solve for the GPS clock bias and drift, when using the accompanying
     * measurements, from consecutive GnssData reports.
     */
    uint32_t hw_clock_discontinuity_count;

} GnssClock;

typedef struct {
    /** set to sizeof(GpsMeasurement) */
    size_t size;

    /** A set of flags indicating the validity of the fields in this data structure. */
    GnssMeasurementFlags flags;

    /**
     * Satellite vehicle ID number, as defined in GnssSvInfo::svid
     * This is a mandatory value.
     */
    int16_t svid;

    /**
     * Defines the constellation of the given SV. Value should be one of those
     * GNSS_CONSTELLATION_* constants
     */
    GnssConstellationType constellation;

    /**
     * Time offset at which the measurement was taken in nanoseconds.
     * The reference receiver's time is specified by GpsData::clock::time_ns and should be
     * interpreted in the same way as indicated by GpsClock::type.
     *
     * The sign of time_offset_ns is given by the following equation:
     *      measurement time = GpsClock::time_ns + time_offset_ns
     *
     * It provides an individual time-stamp for the measurement, and allows sub-nanosecond accuracy.
     * This is a mandatory value.
     */
    double time_offset_ns;

    /**
     * Per satellite sync state. It represents the current sync state for the associated satellite.
     * Based on the sync state, the 'received GPS tow' field should be interpreted accordingly.
     *
     * This is a mandatory value.
     */
    GnssMeasurementState state;

    int64_t received_sv_time_in_ns;

    /**
     * 1-Sigma uncertainty of the Received GPS Time-of-Week in nanoseconds.
     *
     * This value must be populated if 'state' != GPS_MEASUREMENT_STATE_UNKNOWN.
     */
    int64_t received_sv_time_uncertainty_in_ns;

    /**
     * Carrier-to-noise density in dB-Hz, typically in the range [0, 63].
     * It contains the measured C/N0 value for the signal at the antenna port.
     *
     * This is a mandatory value.
     */
    double c_n0_dbhz;

    /**
     * Pseudorange rate at the timestamp in m/s. The correction of a given
     * Pseudorange Rate value includes corrections for receiver and satellite
     * clock frequency errors. Ensure that this field is independent (see
     * comment at top of GnssMeasurement struct.)
     *
     * It is mandatory to provide the 'uncorrected' 'pseudorange rate', and provide GpsClock's
     * 'drift' field as well (When providing the uncorrected pseudorange rate, do not apply the
     * corrections described above.)
     *
     * The value includes the 'pseudorange rate uncertainty' in it.
     * A positive 'uncorrected' value indicates that the SV is moving away from the receiver.
     *
     * The sign of the 'uncorrected' 'pseudorange rate' and its relation to the sign of 'doppler
     * shift' is given by the equation:
     *      pseudorange rate = -k * doppler shift   (where k is a constant)
     *
     * This should be the most accurate pseudorange rate available, based on
     * fresh signal measurements from this channel.
     *
     * It is mandatory that this value be provided at typical carrier phase PRR
     * quality (few cm/sec per second of uncertainty, or better) - when signals
     * are sufficiently strong & stable, e.g. signals from a GPS simulator at >=
     * 35 dB-Hz.
     */
    double pseudorange_rate_mps;

    /**
     * 1-Sigma uncertainty of the pseudorange_rate_mps.
     * The uncertainty is represented as an absolute (single sided) value.
     *
     * This is a mandatory value.
     */
    double pseudorange_rate_uncertainty_mps;

    /**
     * Accumulated delta range's state. It indicates whether ADR is reset or there is a cycle slip
     * (indicating loss of lock).
     *
     * This is a mandatory value.
     */
    GnssAccumulatedDeltaRangeState accumulated_delta_range_state;

    double accumulated_delta_range_m;

    /**
     * 1-Sigma uncertainty of the accumulated delta range in meters.
     * This value must be populated if 'accumulated delta range state' != GPS_ADR_STATE_UNKNOWN.
     */
    double accumulated_delta_range_uncertainty_m;

    /**
     * Carrier frequency at which codes and messages are modulated, it can be L1 or L2.
     * If the field is not set, the carrier frequency is assumed to be L1.
     *
     * If the data is available, 'flags' must contain
     * GNSS_MEASUREMENT_HAS_CARRIER_FREQUENCY.
     */
    float carrier_frequency_hz;

    /**
     * The number of full carrier cycles between the satellite and the receiver.
     * The reference frequency is given by the field 'carrier_frequency_hz'.
     * Indications of possible cycle slips and resets in the accumulation of
     * this value can be inferred from the accumulated_delta_range_state flags.
     *
     * If the data is available, 'flags' must contain
     * GNSS_MEASUREMENT_HAS_CARRIER_CYCLES.
     */
    int64_t carrier_cycles;

    /**
     * The RF phase detected by the receiver, in the range [0.0, 1.0].
     * This is usually the fractional part of the complete carrier phase measurement.
     *
     * The reference frequency is given by the field 'carrier_frequency_hz'.
     * The value contains the 'carrier-phase uncertainty' in it.
     *
     * If the data is available, 'flags' must contain
     * GNSS_MEASUREMENT_HAS_CARRIER_PHASE.
     */
    double carrier_phase;

    /**
     * 1-Sigma uncertainty of the carrier-phase.
     * If the data is available, 'flags' must contain
     * GNSS_MEASUREMENT_HAS_CARRIER_PHASE_UNCERTAINTY.
     */
    double carrier_phase_uncertainty;
  GnssMultipathIndicator multipath_indicator;

    /**
     * Signal-to-noise ratio at correlator output in dB.
     * If the data is available, 'flags' must contain GNSS_MEASUREMENT_HAS_SNR.
     * This is the power ratio of the "correlation peak height above the
     * observed noise floor" to "the noise RMS".
     */
    double snr_db;
} GnssMeasurement;

typedef struct {
    /** set to sizeof(GnssData) */
    size_t size;

    /** Number of measurements. */
    size_t measurement_count;

    /** The array of measurements. */
    GnssMeasurement measurements[GNSS_MAX_MEASUREMENT];

    /** The GPS clock time reading. */
    GnssClock clock;
} GnssData;


typedef struct {
    /** set to sizeof(GnssNavigationMessage) */
    size_t size;

    /**
     * Satellite vehicle ID number, as defined in GnssSvInfo::svid
     * This is a mandatory value.
     */
    int16_t svid;

    /**
     * The type of message contained in the structure.
     * This is a mandatory value.
     */
    GnssNavigationMessageType type;

    /**
     * The status of the received navigation message.
     * No need to send any navigation message that contains words with parity error and cannot be
     * corrected.
     */
    NavigationMessageStatus status;

    /**
     * Message identifier. It provides an index so the complete Navigation
     * Message can be assembled.
     *
     * - For GPS L1 C/A subframe 4 and 5, this value corresponds to the 'frame
     *   id' of the navigation message, in the range of 1-25 (Subframe 1, 2, 3
     *   does not contain a 'frame id' and this value can be set to -1.)
     *
     * - For Glonass L1 C/A, this refers to the frame ID, in the range of 1-5.
     *
     * - For BeiDou D1, this refers to the frame number in the range of 1-24
     *
     * - For Beidou D2, this refers to the frame number, in the range of 1-120
     *
     * - For Galileo F/NAV nominal frame structure, this refers to the subframe
     *   number, in the range of 1-12
     *
     * - For Galileo I/NAV nominal frame structure, this refers to the subframe
     *   number in the range of 1-24
     */
    int16_t message_id;

    /**
     * Sub-message identifier. If required by the message 'type', this value
     * contains a sub-index within the current message (or frame) that is being
     * transmitted.
     *
     * - For GPS L1 C/A, BeiDou D1 & BeiDou D2, the submessage id corresponds to
     *   the subframe number of the navigation message, in the range of 1-5.
     *
     * - For Glonass L1 C/A, this refers to the String number, in the range from
     *   1-15
     *
     * - For Galileo F/NAV, this refers to the page type in the range 1-6
     *
     * - For Galileo I/NAV, this refers to the word type in the range 1-10+
     */
    int16_t submessage_id;
    /**
     * The length of the data (in bytes) contained in the current message.
     * If this value is different from zero, 'data' must point to an array of the same size.
     * e.g. for L1 C/A the size of the sub-frame will be 40 bytes (10 words, 30 bits/word).
     *
     * This is a mandatory value.
     */
    size_t data_length;

    /**
     * The data of the reported GPS message. The bytes (or words) specified
     * using big endian format (MSB first).
     *
     * - For GPS L1 C/A, Beidou D1 & Beidou D2, each subframe contains 10 30-bit
     *   words. Each word (30 bits) should be fit into the last 30 bits in a
     *   4-byte word (skip B31 and B32), with MSB first, for a total of 40
     *   bytes, covering a time period of 6, 6, and 0.6 seconds, respectively.
     *
     * - For Glonass L1 C/A, each string contains 85 data bits, including the
     *   checksum.  These bits should be fit into 11 bytes, with MSB first (skip
     *   B86-B88), covering a time period of 2 seconds.
     *
     * - For Galileo F/NAV, each word consists of 238-bit (sync & tail symbols
     *   excluded). Each word should be fit into 30-bytes, with MSB first (skip
     *   B239, B240), covering a time period of 10 seconds.
     *
     * - For Galileo I/NAV, each page contains 2 page parts, even and odd, with
     *   a total of 2x114 = 228 bits, (sync & tail excluded) that should be fit
     *   into 29 bytes, with MSB first (skip B229-B232).
     */
    uint8_t* data;

} GnssNavigationMessage;





__END_DECLS

#endif /* ANDROID_INCLUDE_HARDWARE_GPS_H */

