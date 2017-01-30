#include <time.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#include <libdaemon/dlog.h>

#include "rtl-sdr.h"

#include "Acurite5N1RTL433.hpp"

// Helper macros
#ifndef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#endif
#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif

// From draythomp/Desert-home-rtl_433
// matches acu-link internet bridge values
// The mapping isn't circular, it jumps around.
const char* acurite_5n1_winddirection_str[] =
{
    "NW",  // 0  315
    "WSW", // 1  247.5
    "WNW", // 2  292.5
    "W",   // 3  270
    "NNW", // 4  337.5
    "SW",  // 5  225
    "N",   // 6  0
    "SSW", // 7  202.5
    "ENE", // 8  67.5
    "SE",  // 9  135
    "E",   // 10 90
    "ESE", // 11 112.5
    "NE",  // 12 45
    "SSE", // 13 157.5
    "NNE", // 14 22.5
    "S"  // 15 180
};

const float acurite_5n1_winddirections[] =
{
    315.0, // 0 - NW
    247.5, // 1 - WSW
    292.5, // 2 - WNW
    270.0, // 3 - W
    337.5, // 4 - NNW
    225.0, // 5 - SW
    0.0,   // 6 - N
    202.5, // 7 - SSW
    67.5,  // 8 - ENE
    135.0, // 9 - SE
    90.0,  // a - E
    112.5, // b - 112.5
    45.0,  // c - NE
    157.5, // d - SSE
    22.5,  // e - NNE
    180.0, // f - S
};

// If we make this an inline function instead of a macro, it means we don't
// have to worry about using bit numbers with side-effects (bit++).
static inline int bit(const uint8_t *bytes, unsigned bit)
{
	return bytes[bit >> 3] >> (7 - (bit & 7)) & 1;
}

RTL433BitBuffer::RTL433BitBuffer()
{
    clear();
}

RTL433BitBuffer::~RTL433BitBuffer()
{

}

uint16_t 
RTL433BitBuffer::getActiveRows()
{
    return num_rows;
}

uint8_t* 
RTL433BitBuffer::getRowPtr( unsigned rowIndex, uint16_t &activeBits )
{
    activeBits = 0;
   
    if( rowIndex >= num_rows )
        return NULL;

    activeBits = bits_per_row[ rowIndex ];

    return bb[ rowIndex ];
}

void 
RTL433BitBuffer::clear() 
{
    num_rows = 0;
    memset( bits_per_row, 0, BITBUF_ROWS*2 );
    memset( bb, 0, BITBUF_ROWS * BITBUF_COLS );
}


void 
RTL433BitBuffer::add_bit( int bit ) 
{
    if( num_rows == 0 )
        num_rows++;  // Add first row automatically

    uint16_t col_index = bits_per_row[ (num_rows-1) ] / 8;
    uint16_t bit_index = bits_per_row[ (num_rows-1) ] % 8;

    if( ( col_index < BITBUF_COLS ) && ( num_rows <= BITBUF_ROWS ) )
    {
        bb[ (num_rows-1) ][ col_index ] |= ( bit << (7-bit_index) );
        bits_per_row[ (num_rows-1) ]++;
    }
    else
    {
        daemon_log( LOG_ERR,  "ERROR: bitbuffer:: Could not add more columns\n" );  // Some decoders may add many columns...
    }
}

void 
RTL433BitBuffer::add_row()
{
    if( num_rows == 0 )
        num_rows++;  // Add first row automatically

    if( num_rows < BITBUF_ROWS )
    {
        num_rows++;
    }
    else
    {
        daemon_log( LOG_ERR,  "ERROR: bitbuffer:: Could not add more rows\n" );  // Some decoders may add many rows...
    }
}

void 
RTL433BitBuffer::invert()
{
    for( unsigned row = 0; row < num_rows; ++row )
    {
        if( bits_per_row[ row ] > 0 )
        {
            const unsigned last_col  = ( bits_per_row[ row ]-1 ) / 8;
            const unsigned last_bits = ( ( bits_per_row[ row ]-1 ) % 8 ) +1;

            for( unsigned col = 0; col <= last_col; ++col )
            {
                bb[ row ][ col ] = ~bb[ row ][ col ];  // Invert
            }

            bb[ row ][ last_col ] ^= 0xFF >> last_bits;	// Re-invert unused bits in last byte
        }
    }
}

void 
RTL433BitBuffer::extract_bytes( unsigned row, unsigned pos, uint8_t *out, unsigned len )
{
    uint8_t *bits = bb[ row ];

    if( ( pos & 7 ) == 0 )
    {
        memcpy( out, bits + (pos / 8), (len + 7) / 8 );
    }
    else
    {
        unsigned shift = 8 - (pos & 7);
        uint16_t word;

        pos >>= 3; // Convert to bytes
        len >>= 3;

        word = bits[ pos ];

        while( len-- )
        {
            word <<= 8;
            word |= bits[ ++pos ];
            *(out++) = word >> shift;
        }
    }
}

unsigned 
RTL433BitBuffer::search( unsigned row, unsigned start, const uint8_t *pattern, unsigned pattern_bits_len )
{
    uint8_t *bits = bb[ row ];
    unsigned len = bits_per_row[ row ];
    unsigned ipos = start;
    unsigned ppos = 0;  // cursor on init pattern

    while( ipos < len && ppos < pattern_bits_len )
    {
        if( bit( bits, ipos ) == bit( pattern, ppos ) )
        {
            ppos++;
            ipos++;
            if( ppos == pattern_bits_len )
                return ipos - pattern_bits_len;
        }
        else
        {
            ipos += -ppos + 1;
            ppos = 0;
        }
    }

    // Not found
    return len;
}

unsigned 
RTL433BitBuffer::manchester_decode( unsigned row, unsigned start, RTL433BitBuffer &outbuf, unsigned max )
{
    uint8_t *bits = bb[ row ];
    unsigned int len = bits_per_row[ row ];
    unsigned int ipos = start;

    if( max && len > start + (max * 2) )
        len = start + (max * 2);

    while( ipos < len )
    {
        uint8_t bit1, bit2;

        bit1 = bit( bits, ipos++ );
        bit2 = bit( bits, ipos++ );

        if( bit1 == bit2 )
            break;

        outbuf.add_bit( bit2 );
    }

    return ipos;
}


void 
RTL433BitBuffer::print()
{
    daemon_log( LOG_ERR,  "bitbuffer:: Number of rows: %d \n", num_rows );

    for( uint16_t row = 0; row < num_rows; ++row )
    {
        daemon_log( LOG_ERR,  "[%02d] {%d} ", row, bits_per_row[row] );

		for( uint16_t col = 0; col < (bits_per_row[row]+7)/8; ++col)
        {
            daemon_log( LOG_ERR,  "%02x ", bb[row][col] );
        }

        // Print binary values also?
        if( bits_per_row[row] <= BITBUF_MAX_PRINT_BITS )
        {
            daemon_log( LOG_ERR,  ": " );
            for( uint16_t bit = 0; bit < bits_per_row[row]; ++bit )
            {
                if( bb[row][bit/8] & ( 0x80 >> (bit % 8) ) )
                {
                    daemon_log( LOG_ERR,  "1" );
                } 
                else
                {
                    daemon_log( LOG_ERR,  "0" );
                }
                if( (bit % 8) == 7 ) 
                {
                    daemon_log( LOG_ERR,  " " );
                } // Add byte separators
            }
        }
        daemon_log( LOG_ERR,  "\n" );
    }
}

int 
RTL433BitBuffer::compare_rows( unsigned row_a, unsigned row_b )
{
    return ( bits_per_row[row_a] == bits_per_row[row_b] && !memcmp( bb[row_a], bb[row_b], (bits_per_row[row_a] + 7) / 8 ) );
}

unsigned 
RTL433BitBuffer::count_repeats( unsigned row )
{
    unsigned cnt = 0;
    for( int i = 0; i < num_rows; ++i )
    {
        if( compare_rows( row, i ) ) 
        {
            ++cnt;
        }
    }
    return cnt;
}

int 
RTL433BitBuffer::find_repeated_row( unsigned min_repeats, unsigned min_bits )
{
    for( int i = 0; i < num_rows; ++i )
    {
        if( bits_per_row[i] >= min_bits && count_repeats( i ) >= min_repeats ) 
        {
            return i;
        }
    }

    return -1;
}


RTL433FilterState::RTL433FilterState()
{

}

RTL433FilterState::~RTL433FilterState()
{

}

RTL433DemodFMState::RTL433DemodFMState()
{

}

RTL433DemodFMState::~RTL433DemodFMState()
{

}

RTL433PulseData::RTL433PulseData()
{
    clear();
}

RTL433PulseData::~RTL433PulseData()
{

}

void
RTL433PulseData::clear()
{
    num_pulses         = 0;
    ook_low_estimate   = 0;
    ook_high_estimate  = 0;
    fsk_f1_est         = 0;
    fsk_f2_est         = 0;
}

RTL433Demodulator::RTL433Demodulator()
{
    // Setup the base band filters
    for( int i = 0; i < 256; i++ )
        scaledSquares[i] = (127 - i) * (127 - i);

    // [b,a] = butter(1, 0.05) -> 3x tau (95%) ~20 samples
    a[0] = FIX(1.00000);
    a[1] = FIX(0.85408);

    b[0] = FIX(0.07296);
    b[1] = FIX(0.07296);

    // [b,a] = butter(1, 0.2) -> 3x tau (95%) ~5 samples
    alp[0] = FIX(1.00000);
    alp[1] = FIX(0.50953);

    blp[0] = FIX(0.24524);
    blp[1] = FIX(0.24524);

    // Setup pulse detector

    ook_state = PD_OOK_STATE_IDLE;
    curPulse = NULL;
    pulse_length = 0;    // Counter for internal pulse detection
    max_pulse = 0;       // Size of biggest pulse detected

    data_counter = 0;    // Counter for how much of data chunck is processed
    lead_in_counter = 0; // Counter for allowing initial noise estimate to settle

    ook_low_estimate = 0;   // Estimate for the OOK low level (base noise level) in the envelope data
    ook_high_estimate = 0;  // Estimate for the OOK high level

    // Init other variables
    do_exit = 0;
    samp_rate = DEFAULT_SAMPLE_RATE;
    dev = NULL;
    //rtlThread = NULL;

    am_buf = (int16_t *) malloc( (sizeof( int16_t ) * MAXIMAL_BUF_LENGTH) );	
    fm_buf = (int16_t *) malloc( (sizeof( int16_t ) * MAXIMAL_BUF_LENGTH) ); 
    temp_buf = (uint16_t *) malloc( (sizeof( uint16_t ) * MAXIMAL_BUF_LENGTH) ); 

    acurite_5n1raincounter = 0;

    notifyCB = NULL;

    measurementIndex = 0;
}

RTL433Demodulator::~RTL433Demodulator()
{
    free( am_buf );
    free( fm_buf );
    free( temp_buf );
}

void 
RTL433Demodulator::clearNotify()
{
    notifyCB = NULL;
}

void 
RTL433Demodulator::setNotify( RTL433DemodNotify *cbObj )
{
    notifyCB = cbObj;
}

int32_t 
RTL433Demodulator::getDetectionLimit()
{
    return level_limit;
}

/// Demodulate On/Off Keying (OOK) and Frequency Shift Keying (FSK) from an envelope signal
int 
RTL433Demodulator::identifyPulses( const int16_t *envelope_data, const int16_t *fm_data, int len, int16_t level_limit, uint32_t samp_rate ) 
{
	const int samples_per_ms = samp_rate / 1000;
	ook_high_estimate = max( ook_high_estimate, OOK_MIN_HIGH_LEVEL );	// Be sure to set initial minimum level

    if( curPulse == NULL )
    {
        curPulse = new RTL433PulseData;
    }

	// Process all new samples
	while( data_counter < len ) 
    {
		// Calculate OOK detection threshold and hysteresis
		const int16_t am_n = envelope_data[data_counter];
		int16_t ook_threshold = ook_low_estimate + ( ook_high_estimate - ook_low_estimate ) / 2;
		if( level_limit != 0 ) 
            ook_threshold = level_limit; // Manual override
		const int16_t ook_hysteresis = ook_threshold / 8; // ±12%

		// OOK State machine
		switch( ook_state ) 
        {
            case PD_OOK_STATE_IDLE:
                // Above threshold? Lead in counter to stabilize noise estimate
                if( am_n > ( ook_threshold + ook_hysteresis ) && ( lead_in_counter > OOK_EST_LOW_RATIO ) ) 
                {
                    // Initialize all data
                    curPulse->clear(); //pulse_data_clear(pulses);
                    pulse_length = 0;
                    max_pulse = 0;
                    ook_state = PD_OOK_STATE_PULSE;
                 }
                 else
                 {   
                     // We are still idle..
                     // Estimate low (noise) level
                     const int ook_low_delta = am_n - ook_low_estimate;
                     ook_low_estimate += ook_low_delta / OOK_EST_LOW_RATIO;
                     ook_low_estimate += ( ( ook_low_delta > 0 ) ? 1 : -1 );	// Hack to compensate for lack of fixed-point scaling

                     // Calculate default OOK high level estimate
                     ook_high_estimate = OOK_HIGH_LOW_RATIO * ook_low_estimate;	// Default is a ratio of low level
                     ook_high_estimate = max( ook_high_estimate, OOK_MIN_HIGH_LEVEL );
                     ook_high_estimate = min( ook_high_estimate, OOK_MAX_HIGH_LEVEL );
                     if( lead_in_counter <= OOK_EST_LOW_RATIO ) 
                         lead_in_counter++; // Allow inital estimate to settle
				}
            break;

			case PD_OOK_STATE_PULSE:
				pulse_length++;
				// End of pulse detected? // Gap?
				if( am_n  < ( ook_threshold - ook_hysteresis ) ) 
                { 
                    // Check for spurious short pulses
					if( pulse_length < PD_MIN_PULSE_SAMPLES ) 
                    {
                        ook_state = PD_OOK_STATE_IDLE;
					} 
                    else
                    {
						// Continue with OOK decoding
						curPulse->pulse[ curPulse->num_pulses ] = pulse_length;	// Store pulse width
						max_pulse = max(pulse_length, max_pulse);	// Find largest pulse
						pulse_length = 0;
						ook_state = PD_OOK_STATE_GAP_START;
                    }
                } 
                else
                {
				    // Still pulse
                    // Calculate OOK high level estimate
                    ook_high_estimate += am_n / OOK_EST_HIGH_RATIO - ook_high_estimate / OOK_EST_HIGH_RATIO;
                    ook_high_estimate = max(ook_high_estimate, OOK_MIN_HIGH_LEVEL);
                    ook_high_estimate = min(ook_high_estimate, OOK_MAX_HIGH_LEVEL);
                    // Estimate pulse carrier frequency
                    curPulse->fsk_f1_est += fm_data[data_counter] / OOK_EST_HIGH_RATIO - curPulse->fsk_f1_est / OOK_EST_HIGH_RATIO;
				}
            break;

			case PD_OOK_STATE_GAP_START:	// Beginning of gap - it might be a spurious gap
                pulse_length++;
				
                // Pulse detected again already? (This is a spurious short gap)
                if( am_n  > ( ook_threshold + ook_hysteresis ) )
                { // New pulse?
				    pulse_length += curPulse->pulse[curPulse->num_pulses];	// Restore counter
                    ook_state = PD_OOK_STATE_PULSE;
                    // Or this gap is for real?
                } 
                else if( pulse_length >= PD_MIN_PULSE_SAMPLES ) 
                {
                    ook_state = PD_OOK_STATE_GAP;
				}
            break;

            case PD_OOK_STATE_GAP:
                pulse_length++;
                // New pulse detected?
                if( am_n  > ( ook_threshold + ook_hysteresis ) ) 
                { 
                    curPulse->gap[ curPulse->num_pulses ] = pulse_length; // Store gap width
                    curPulse->num_pulses++; // Next pulse

                    // EOP if too many pulses
                    if( curPulse->num_pulses >= PD_MAX_PULSES )
                    {
                        ook_state = PD_OOK_STATE_IDLE;

                        // Store estimates
                        curPulse->ook_low_estimate = ook_low_estimate;
                        curPulse->ook_high_estimate = ook_high_estimate;

                        // Pulse Data ready for decode
                        pulseQueue.push_back( curPulse );
                        curPulse = new RTL433PulseData;

                        return 1; // End Of Package!!
                    }

                    pulse_length = 0;
                    ook_state = PD_OOK_STATE_PULSE;
                }

                // EOP if gap is too long
                // gap/pulse ratio exceeded // Minimum gap exceeded // maximum gap exceeded
                if( ( ( pulse_length > ( PD_MAX_GAP_RATIO * max_pulse ) ) && ( pulse_length > ( PD_MIN_GAP_MS * samples_per_ms ) ) ) || ( pulse_length > ( PD_MAX_GAP_MS * samples_per_ms ) ) )
                {
                    curPulse->gap[ curPulse->num_pulses ] = pulse_length;	// Store gap width
                    curPulse->num_pulses++;	// Store last pulse
                    ook_state = PD_OOK_STATE_IDLE;

                    // Store estimates
                    curPulse->ook_low_estimate = ook_low_estimate;
                    curPulse->ook_high_estimate = ook_high_estimate;

                    // Pulse Data ready for decode
                    pulseQueue.push_back( curPulse );
                    curPulse = new RTL433PulseData;

                    return 1; // End Of Package!!
                }
            break;

            default:
                daemon_log( LOG_ERR, "demod_OOK(): Unknown state!!\n");
                ook_state = PD_OOK_STATE_IDLE;
            break;

        }

        data_counter++;

    }

    // Done
    data_counter = 0;
    return 0;
}

int 
RTL433Demodulator::demodPWM( RTL433PulseData *pulseData ) 
{
    int events = 0;
    int start_bit_detected = 0;
    int start_bit = 0; //device->demod_arg;
    RTL433BitBuffer *bits = NULL;

    for( unsigned n = 0; n < pulseData->num_pulses; ++n ) 
    {
        if( bits == NULL )
        {
            bits = new RTL433BitBuffer;
        }

        // Should we disregard startbit?
        if( start_bit == 1 && start_bit_detected == 0 )
        {
            start_bit_detected = 1;
        }
        else
        {
            // Detect pulse width
            if( pulseData->pulse[n] <= 70 ) //device->short_limit )
            {
                bits->add_bit( 1 );
            }
            else
            {
                bits->add_bit( 0 );
            }
        }

		// End of Message? // No more pulses (FSK) // Long silence (OOK)
        if( n == pulseData->num_pulses - 1 || pulseData->gap[n] > 200 ) // device->reset_limit )
        {  
            bitQueue.push_back( bits );
            bits = NULL;

            start_bit_detected = 0;
            // Check for new packet in multipacket
        }
        else if( pulseData->gap[n] > 130 ) //device->long_limit )
        {
            bits->add_row();
            start_bit_detected = 0;
        }
    }
    return events;
}

/** This will give a noisy envelope of OOK/ASK signals
 *  Subtract the bias (-128) and get an envelope estimation
 *  The output will be written in the input buffer
 *  @returns   pointer to the input buffer
 */
void 
RTL433Demodulator::envelope_detect(const uint8_t *iq_buf, uint16_t *y_buf, uint32_t len) 
{
    unsigned int i;
    for( i = 0; i < len; i++ ) 
    {
        y_buf[i] = scaledSquares[ iq_buf[ (2 * i) ] ] + scaledSquares[ iq_buf[ ((2 * i) + 1) ] ];
    }
}


/** Something that might look like a IIR lowpass filter
 *
 *  [b,a] = butter(1, Wc) # low pass filter with cutoff pi*Wc radians
 *  Q1.15*Q15.0 = Q16.15
 *  Q16.15>>1 = Q15.14
 *  Q15.14 + Q15.14 + Q15.14 could possibly overflow to 17.14
 *  but the b coeffs are small so it wont happen
 *  Q15.14>>14 = Q15.0 \o/
 */
void 
RTL433Demodulator::low_pass_filter( const uint16_t *x_buf, int16_t *y_buf, uint32_t len ) 
{
    unsigned int i;
    // Fixme: Will Segmentation Fault if len < FILTERORDER

    /* Calculate first sample */
    y_buf[0] = ((a[1] * amFilterState.y[0] >> 1) + (b[0] * x_buf[0] >> 1) + (b[1] * amFilterState.x[0] >> 1)) >> (F_SCALE - 1);
    for (i = 1; i < len; i++) 
    {
        y_buf[i] = ((a[1] * y_buf[i - 1] >> 1) + (b[0] * x_buf[i] >> 1) + (b[1] * x_buf[i - 1] >> 1)) >> (F_SCALE - 1);
    }

    /* Save last samples */
    memcpy(amFilterState.x, &x_buf[len - FILTER_ORDER], FILTER_ORDER * sizeof (int16_t));
    memcpy(amFilterState.y, &y_buf[len - FILTER_ORDER], FILTER_ORDER * sizeof (int16_t));
}


/// Integer implementation of atan2() with int16_t normalized output
///
/// Returns arc tangent of y/x across all quadrants in radians
/// Reference: http://dspguru.com/dsp/tricks/fixed-point-atan2-with-self-normalization
/// @param y: Numerator (imaginary value of complex vector)
/// @param x: Denominator (real value of complex vector)
/// @return angle in radians (Pi equals INT16_MAX)
int16_t 
RTL433Demodulator::atan2_int16( int16_t y, int16_t x ) 
{
	static const int32_t I_PI_4 = INT16_MAX/4;		// M_PI/4
	static const int32_t I_3_PI_4 = 3*INT16_MAX/4;	// 3*M_PI/4
	const int32_t abs_y = abs(y);
	int32_t r, angle;

	if (x >= 0) 
    {	// Quadrant I and IV
		int32_t denom = (abs_y + x);
		if (denom == 0) denom = 1;	// Prevent divide by zero
		r = ((x - abs_y) << 16) / denom;
		angle = I_PI_4;
	} 
    else 
    {		// Quadrant II and III
		int32_t denom = (abs_y - x);
		if (denom == 0) denom = 1;	// Prevent divide by zero
		r = ((x + abs_y) << 16) / denom;
		angle = I_3_PI_4;
	}
	angle -= (I_PI_4 * r) >> 16;	// Error max 0.07 radians
	if (y < 0) angle = -angle;	// Negate if in III or IV
	return angle;
}

void 
RTL433Demodulator::demod_FM( const uint8_t *x_buf, int16_t *y_buf, unsigned num_samples ) 
{
	int16_t ar, ai;		// New IQ sample: x[n]
	int16_t br, bi;		// Old IQ sample: x[n-1]
	int32_t pr, pi;		// Phase difference vector
	int16_t angle;		// Phase difference angle
	int16_t xlp, ylp, xlp_old, ylp_old;	// Low Pass filter variables

	// Pre-feed old sample
	ar = fmFilterState.br; ai = fmFilterState.bi;
	xlp_old = fmFilterState.xlp; ylp_old = fmFilterState.ylp;

	for( unsigned n = 0; n < num_samples; n++ ) 
    {
		// delay old sample 
		br = ar;
		bi = ai;
		// get new sample
		ar = x_buf[2*n]-128;
		ai = x_buf[2*n+1]-128;
		// Calculate phase difference vector: x[n] * conj(x[n-1])
		pr = ar*br+ai*bi;	// May exactly overflow an int16_t (-128*-128 + -128*-128)
		pi = ai*br-ar*bi; 
		xlp = atan2_int16(pi, pr);	// Integer implementation
		// Low pass filter
		ylp = ((alp[1] * ylp_old >> 1) + (blp[0] * xlp >> 1) + (blp[1] * xlp_old >> 1)) >> (F_SCALE - 1);
		ylp_old = ylp; xlp_old = xlp;
		y_buf[n] = ylp;
	}

	// Store newest sample for next run
	fmFilterState.br = ar; fmFilterState.bi = ai;
	fmFilterState.xlp = xlp_old; fmFilterState.ylp = ylp_old;
}

void 
RTL433Demodulator::rtlsdr_callback(unsigned char *iq_buf, uint32_t len, void *ctx) 
{
    RTL433Demodulator *demod = (RTL433Demodulator *) ctx;

    demod->processRtlsdrData( iq_buf, len );
}

void 
RTL433Demodulator::processRtlsdrData( unsigned char *iq_buf, uint32_t len )
{
    int i;

    if( do_exit )
        return;

	// AM demodulation
	envelope_detect( iq_buf, temp_buf, len/2 );
	low_pass_filter( temp_buf, am_buf, len/2 );

	// FM demodulation
    demod_FM( iq_buf, fm_buf, len/2 );

    // Identify pulses
    identifyPulses( am_buf, fm_buf, len/2, level_limit, samp_rate );

    // Decode pulses to bit stream
    for( std::list< RTL433PulseData* >::iterator it = pulseQueue.begin(); it != pulseQueue.end(); it++ )
    {
        // Decode device data from bit stream
        demodPWM( *it );

        delete *it;
    }

    pulseQueue.clear();

    // Decode the bit stream into Acurite5n1 data fields.
    for( std::list< RTL433BitBuffer* >::iterator it = bitQueue.begin(); it != bitQueue.end(); it++ )
    {
        // Decode device data from bit stream
        extractAcurite5n1Data( *it );

        delete *it;
    }

    bitQueue.clear();

    time_t rawtime;
    time( &rawtime );

}

void
RTL433Demodulator::init()
{
    int ppm_error = 0;
    int r = 0;
    uint32_t dev_index = 0;

    do_exit = false;

    level_limit = DEFAULT_LEVEL_LIMIT;

	r = rtlsdr_open( &dev, dev_index );
	if( r < 0 ) 
    {
	    daemon_log( LOG_ERR, "Failed to open rtlsdr device #%d.\n", dev_index);
	    return;
	}

	/* Set the sample rate */
	r = rtlsdr_set_sample_rate(dev, samp_rate);
	if (r < 0)
	    daemon_log( LOG_ERR, "WARNING: Failed to set sample rate.\n");
	else
	    daemon_log( LOG_ERR, "Sample rate set to %d.\n", rtlsdr_get_sample_rate(dev)); // Unfortunately, doesn't return real rate

    /* Enable automatic gain */
    r = rtlsdr_set_tuner_gain_mode( dev, 0 );
    if( r < 0 )
        daemon_log( LOG_ERR,  "WARNING: Failed to enable automatic gain.\n" );
    else
        daemon_log( LOG_ERR,  "Tuner gain set to Auto.\n" );

	daemon_log( LOG_ERR,  "Bit detection level set to %d%s.\n", getDetectionLimit(), (getDetectionLimit() ? "" : " (Auto)") );

	r = rtlsdr_set_freq_correction( dev, ppm_error );

    /* Reset endpoint before we start reading from it (mandatory) */
    r = rtlsdr_reset_buffer( dev );
    if( r < 0 )
        daemon_log( LOG_ERR,  "WARNING: Failed to reset buffers.\n" );

    // Set the frequency
    r = rtlsdr_set_center_freq( dev, DEFAULT_FREQUENCY );

    if (r < 0)
        daemon_log( LOG_ERR, "WARNING: Failed to set center freq.\n");
    else
        daemon_log( LOG_ERR, "Tuned to %u Hz.\n", rtlsdr_get_center_freq(dev));

}

void
RTL433Demodulator::cleanup()
{


}

void 
RTL433Demodulator::processSample()
{
    int r = 0;
    uint32_t out_block_size = DEFAULT_BUF_LENGTH;
    int n_read = 0;

#if 0
    r = rtlsdr_read_async(dev, RTL433Demodulator::rtlsdr_callback, (void *) this, DEFAULT_ASYNC_BUF_NUMBER, out_block_size);
#endif

    uint8_t *buffer = (uint8_t*) malloc( out_block_size * sizeof (uint8_t) );

    r = rtlsdr_read_sync( dev, buffer, out_block_size, &n_read );
    if( r < 0 )
    {
        daemon_log( LOG_ERR, "sync read failed.\n" );
        return;
    }

    //daemon_log( LOG_ERR, "rtlsdr_read_sync - read: %d\n", n_read );

    processRtlsdrData( buffer, n_read );

    free( buffer );
}

int
RTL433Demodulator::acurite_checksum( uint8_t row[BITBUF_COLS], int cols ) 
{
    // sum of first n-1 bytes modulo 256 should equal nth byte
    // also disregard a row of all zeros
    int i;
    int sum = 0;
    for ( i=0; i < cols; i++ )
        sum += row[i];

    if( sum != 0 && ( sum % 256 == row[cols] ) )
        return 1;
    else
        return 0;
}

int 
RTL433Demodulator::acurite_detect( uint8_t *pRow )
{
    int i;
    if( pRow[0] != 0x00 )
    {
        // invert bits due to wierd issue
        for (i = 0; i < 8; i++)
            pRow[i] = ~pRow[i] & 0xFF;
        pRow[0] |= pRow[8];  // fix first byte that has mashed leading bit

        if (acurite_checksum(pRow, 7))
            return 1;  // valid checksum
    }
    return 0;
}

// 5n1 keep state for how much rain has been seen so far
//static int acurite_5n1raincounter = 0;  // for 5n1 decoder
//static int acurite_5n1t_raincounter = 0;  // for combined 5n1/TXR decoder

// Temperature encoding for 5-n-1 sensor and possibly others
float 
RTL433Demodulator::acurite_getTemp( uint8_t highbyte, uint8_t lowbyte )
{
    // range -40 to 158 F
    int highbits = ( highbyte & 0x0F ) << 7 ;
    int lowbits = lowbyte & 0x7F;
    int rawtemp = highbits | lowbits;
    float temp = ( rawtemp - 400 ) / 10.0;
    return temp;
}

int 
RTL433Demodulator::acurite_getWindSpeed( uint8_t highbyte, uint8_t lowbyte )
{
    // range: 0 to 159 kph
	// TODO: sensor does not seem to be in kph, e.g.,
	// a value of 49 here was registered as 41 kph on base unit
	// value could be rpm, etc which may need (polynomial) scaling factor??
	int highbits = ( highbyte & 0x1F ) << 3;
    int lowbits = ( lowbyte & 0x70 ) >> 4;
    int speed = highbits | lowbits;
    return speed;
}

// For the 5n1 based on a linear/circular encoding.
float 
RTL433Demodulator::acurite_getWindDirection( uint8_t byte ) 
{
    // 16 compass points, ccw from (NNW) to 15 (N)
    int direction = byte & 0x0F;
    return acurite_5n1_winddirections[ direction ];
}

int 
RTL433Demodulator::acurite_getHumidity( uint8_t byte ) 
{
    // range: 1 to 99 %RH
    int humidity = byte & 0x7F;
    return humidity;
}

int 
RTL433Demodulator::acurite_getRainfallCounter( uint8_t hibyte, uint8_t lobyte )
{
    // range: 0 to 99.99 in, 0.01 in incr., rolling counter?
    int raincounter = ( ( hibyte & 0x7f ) << 7 ) | ( lobyte & 0x7F );
    return raincounter;
}

void 
RTL433Demodulator::sendReading( uint32_t sensorIndex, HNSM_TYPE_T type, HNSM_UNITS_T units, double reading, struct timeval &timestamp )
{
    HNodeSensorMeasurement record;

    if( notifyCB == NULL )
        return;

    record.setType( type );
    record.setUnits( units );
    record.setCount( measurementIndex );
    record.setReading( reading );
    record.setTimestamp( timestamp );

    notifyCB->notifyNewMeasurement( sensorIndex, record );

    measurementIndex += 1;
}

int 
RTL433Demodulator::extractAcurite5n1Data( RTL433BitBuffer *bits )
{
    // acurite 5n1 weather sensor decoding for rtl_433
    // Jens Jensen 2014
    int i;
    uint8_t *buf = NULL;
    struct timeval tstamp;

    gettimeofday( &tstamp, NULL );

    // run through rows til we find one with good checksum (brute force)
    for( i=0; i < bits->getActiveRows(); i++ )
    {
        uint16_t activeBits;
        uint8_t *rowPtr = bits->getRowPtr( i, activeBits );

        if( acurite_detect( rowPtr ) ) 
        {
            buf = rowPtr;
            break; // done
        }
    }

    if( buf )
    {
        if( ( buf[2] & 0x0F ) == 1 )
        {
            // wind speed, wind direction, rainfall
            float rainfall = 0.00;
            int raincounter = acurite_getRainfallCounter( buf[5], buf[6] );
            if( acurite_5n1raincounter > 0 ) 
            {
                // track rainfall difference after first run
                rainfall = ( raincounter - acurite_5n1raincounter ) * 0.01;
            }
            else
            {
                // capture starting counter
                acurite_5n1raincounter = raincounter;
            }

            sendReading( 1, HNSM_TYPE_WIND_SPEED, HNSM_UNITS_KPH, acurite_getWindSpeed( buf[3], buf[4] ), tstamp );
            sendReading( 2, HNSM_TYPE_WIND_DIRECTION, HNSM_UNITS_DEGREES, acurite_getWindDirection( buf[4] ), tstamp );
            sendReading( 3, HNSM_TYPE_RAINFALL, HNSM_UNITS_INCHES, rainfall, tstamp );

            daemon_log( LOG_ERR,  "wind speed: %d kph, ", acurite_getWindSpeed( buf[3], buf[4] ) );
            daemon_log( LOG_ERR,  "wind direction: %0.1f°, ", acurite_getWindDirection( buf[4] ) );
            daemon_log( LOG_ERR,  "rain gauge: %0.2f in.\n", rainfall );
        }
        else if( ( buf[2] & 0x0F ) == 8 )
        {
            sendReading( 1, HNSM_TYPE_WIND_SPEED, HNSM_UNITS_KPH, acurite_getWindSpeed( buf[3], buf[4] ), tstamp );
            sendReading( 4, HNSM_TYPE_TEMPERATURE, HNSM_UNITS_FAHRENHEIT, acurite_getTemp(buf[4], buf[5]), tstamp );
            sendReading( 5, HNSM_TYPE_RELATIVE_HUMIDITY, HNSM_UNITS_PERCENT, acurite_getHumidity(buf[6]), tstamp );

            // wind speed, temp, RH
            daemon_log( LOG_ERR,  "wind speed: %d kph, ", acurite_getWindSpeed(buf[3], buf[4]) );
            daemon_log( LOG_ERR,  "temp: %2.1f° F, ", acurite_getTemp(buf[4], buf[5]) );
            daemon_log( LOG_ERR,  "humidity: %d%% RH\n", acurite_getHumidity(buf[6]) );
        }
    }
    else
    {
        return 0;
    }

    return 1;
}

