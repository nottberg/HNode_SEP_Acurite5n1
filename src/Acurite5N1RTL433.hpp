#ifndef __ACCURITE5N1RTL433_H__
#define __ACCURITE5N1RTL433_H__

#include <stdint.h>
#include <time.h>

#include <list>
#include <vector>

//#include <boost/thread.hpp>

#include "rtl-sdr.h"

#include "HNodeSensorMeasurement.hpp"

#define FILTER_ORDER 1
#define F_SCALE 15
#define S_CONST (1<<F_SCALE)
#define FIX(x) ((int)(x*S_CONST))

#define DEFAULT_SAMPLE_RATE     250000

#define DEFAULT_HOP_TIME        (60*10)
#define DEFAULT_HOP_EVENTS      2

#define DEFAULT_FREQUENCY       433920000
#define DEFAULT_ASYNC_BUF_NUMBER    32

/*
 * Theoretical high level at I/Q saturation is 128x128 = 16384 (above is ripple)
 * 0 = automatic adaptive level limit, else fixed level limit
 * 8000 = previous fixed default
 */
#define DEFAULT_LEVEL_LIMIT     8000
#define DEFAULT_BUF_LENGTH      (16 * 16384)

// OOK adaptive level estimator constants
#define OOK_HIGH_LOW_RATIO	8			// Default ratio between high and low (noise) level
#define OOK_MIN_HIGH_LEVEL	1000		// Minimum estimate of high level
#define OOK_MAX_HIGH_LEVEL	(128*128)	// Maximum estimate for high level (A unit phasor is 128, anything above is overdrive)
#define OOK_MAX_LOW_LEVEL	(OOK_MAX_HIGH_LEVEL/2)	// Maximum estimate for low level
#define OOK_EST_HIGH_RATIO	64			// Constant for slowness of OOK high level estimator
#define OOK_EST_LOW_RATIO	1024		// Constant for slowness of OOK low level (noise) estimator (very slow)

#define PD_MAX_PULSES 1200			// Maximum number of pulses before forcing End Of Package
#define PD_MIN_PULSES 16			// Minimum number of pulses before declaring a proper package
#define PD_MIN_PULSE_SAMPLES 10		// Minimum number of samples in a pulse for proper detection
#define PD_MIN_GAP_MS 10			// Minimum gap size in milliseconds to exceed to declare End Of Package
#define PD_MAX_GAP_MS 100			// Maximum gap size in milliseconds to exceed to declare End Of Package
#define PD_MAX_GAP_RATIO 10			// Ratio gap/pulse width to exceed to declare End Of Package (heuristic)
#define PD_MAX_PULSE_MS 100			// Pulse width in ms to exceed to declare End Of Package (e.g. for non OOK packages)

#define BITBUF_COLS		80		// Number of bytes in a column
#define BITBUF_ROWS		25
#define BITBUF_MAX_PRINT_BITS	50	// Maximum number of bits to print (in addition to hex values)

#define MINIMAL_BUF_LENGTH      512
#define MAXIMAL_BUF_LENGTH      (256 * 16384)

typedef enum RTL433PulseDecoderStateEnum 
{
    PD_OOK_STATE_IDLE		= 0,
    PD_OOK_STATE_PULSE		= 1,
    PD_OOK_STATE_GAP_START	= 2,
    PD_OOK_STATE_GAP		= 3
}PD_OOK_STATE_T;

class RTL433FilterState
{
    private:

    public:
       int16_t y[FILTER_ORDER];
       int16_t x[FILTER_ORDER];

       RTL433FilterState();
      ~RTL433FilterState(); 
};

class RTL433DemodFMState
{
    private:

    public:
        int16_t br;  // Last I/Q sample
        int16_t bi;
        int16_t xlp; // Low-pass filter state
        int16_t ylp;

        RTL433DemodFMState();
       ~RTL433DemodFMState();
};

class RTL433PulseData
{
    private:

    public:
        unsigned int num_pulses;
        int pulse[PD_MAX_PULSES];	// Contains width of a pulse	(high)
        int gap[PD_MAX_PULSES];		// Width of gaps between pulses (low)
        int ook_low_estimate;		// Estimate for the OOK low level (base noise level) at beginning of package
        int ook_high_estimate;		// Estimate for the OOK high level at end of package
        int fsk_f1_est;				// Estimate for the F1 frequency for FSK
        int fsk_f2_est;				// Estimate for the F2 frequency for FSK

        RTL433PulseData();
       ~RTL433PulseData();

        void clear();
};

class RTL433BitBuffer
{
    private:
        uint16_t   num_rows; // Number of active rows
        uint16_t   bits_per_row[BITBUF_ROWS]; // Number of active bits per row
        uint8_t    bb[BITBUF_COLS][BITBUF_ROWS]; // The actual bits buffer

    public:
        RTL433BitBuffer();
       ~RTL433BitBuffer();

        void clear();

        void add_bit( int bit );

        void add_row();

        void invert();

        void extract_bytes( unsigned row, unsigned pos, uint8_t *out, unsigned len );

        unsigned search( unsigned row, unsigned start, const uint8_t *pattern, unsigned pattern_bits_len );

        unsigned manchester_decode( unsigned row, unsigned start, RTL433BitBuffer &outbuf, unsigned max );

        void print();

        int compare_rows( unsigned row_a, unsigned row_b );

        unsigned count_repeats( unsigned row );

        int find_repeated_row( unsigned min_repeats, unsigned min_bits );

        uint16_t getActiveRows();
        uint8_t* getRowPtr( unsigned rowIndex, uint16_t &activeBits );
};

class RTL433DemodNotify
{
    private:

    public:
        
        virtual void notifyNewMeasurement( uint32_t sensorIndex, HNodeSensorMeasurement &reading ) = 0;
        virtual void signalError( std::string errMsg ) = 0;
        virtual void signalRunning() = 0;
};


class RTL433Demodulator
{
    private:
        int do_exit;
        time_t rawtime_old;
        time_t stop_time;
        uint32_t samp_rate;
        rtlsdr_dev_t *dev;

        int32_t level_limit;
        int16_t *am_buf; //[MINIMAL_BUF_LENGTH];	// AM demodulated signal (for OOK decoding)

        // These buffers aren't used at the same time, so let's use a union to save some memory
        int16_t *fm_buf; //[MINIMAL_BUF_LENGTH];	// FM demodulated signal (for FSK decoding)
        uint16_t *temp_buf; //[MINIMAL_BUF_LENGTH];	// Temporary buffer (to be optimized out..)

        uint16_t scaledSquares[256];

        int a[FILTER_ORDER + 1];
        int b[FILTER_ORDER + 1];

        int alp[2];
        int blp[2];

        RTL433FilterState  amFilterState;
        RTL433DemodFMState fmFilterState;

        // Pulse State variables
        PD_OOK_STATE_T ook_state;

        int pulse_length;    // Counter for internal pulse detection
        int max_pulse;       // Size of biggest pulse detected

        int data_counter;    // Counter for how much of data chunck is processed
        int lead_in_counter; // Counter for allowing initial noise estimate to settle

        int ook_low_estimate;   // Estimate for the OOK low level (base noise level) in the envelope data
        int ook_high_estimate;  // Estimate for the OOK high level

        RTL433PulseData  *curPulse;

        std::list< RTL433PulseData* > pulseQueue;

        int identifyPulses( const int16_t *envelope_data, const int16_t *fm_data, int len, int16_t level_limit, uint32_t samp_rate );

        int demodPWM( RTL433PulseData *pulseData );

        std::list< RTL433BitBuffer* > bitQueue;

        //std::list< RTL433WeatherReading* > readingList;

        int acurite_5n1raincounter;  // for 5n1 decoder
      
        RTL433DemodNotify *notifyCB;

        uint32_t measurementIndex;

        // Integer implementation of atan2() with int16_t normalized output
        int16_t atan2_int16( int16_t y, int16_t x );

        // This will give a noisy envelope of OOK/ASK signals
        void envelope_detect( const uint8_t *iq_buf, uint16_t *y_buf, uint32_t len );

        // Something that might look like a IIR lowpass filter
        void low_pass_filter( const uint16_t *x_buf, int16_t *y_buf, uint32_t len );

        // 
        void demod_FM( const uint8_t *x_buf, int16_t *y_buf, unsigned num_samples );

        int acurite_checksum( uint8_t row[BITBUF_COLS], int cols );
        int acurite_detect( uint8_t *pRow );

        float acurite_getTemp( uint8_t highbyte, uint8_t lowbyte );
        int acurite_getWindSpeed( uint8_t highbyte, uint8_t lowbyte );
        float acurite_getWindDirection( uint8_t byte );
        int acurite_getHumidity( uint8_t byte );
        int acurite_getRainfallCounter( uint8_t hibyte, uint8_t lobyte );
        int extractAcurite5n1Data( RTL433BitBuffer *bits );

        void processRtlsdrData( unsigned char *iq_buf, uint32_t len );

        void sendReading( uint32_t sensorIndex, HNSM_TYPE_T type, HNSM_UNITS_T units, double reading, struct timeval &timestamp );

        //void trimReadingList();

        //boost::thread *rtlThread;

        //boost::mutex readingListMutex;


    public:
        RTL433Demodulator();
       ~RTL433Demodulator();

        int32_t getDetectionLimit();

        uint32_t getMeasurementCount();

        void clearNotify();
        void setNotify( RTL433DemodNotify *cbOBJ );

        void init();
        
        void processSample();

        void cleanup();

//        void start();

//        void stop();

        //void getReadingListEntries( std::vector< RTL433WeatherReading > &rList );

        static void rtlsdr_callback(unsigned char *iq_buf, uint32_t len, void *ctx);
};

#endif // __ACCURITE5N1RTL433_H__
