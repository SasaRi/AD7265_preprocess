
/*
 * Test_roboyagi.xc
 *
 *  Created on: Apr 27, 2016
 *      Author: sasa
 *      AD7265 has two output lines (miso) with sampling frequency of 1 MSPS
 */

#include <adc_service.h>
#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include <stdint.h>

#include <adc_7265.h>
#include <quadflashlib.h>

#include <xscope.h>

#include <power_metering.h>
#include <lib_dsp.h>

#define FS 41000     // sampling frequency of AD7265 / 5 (fs = 205 kHz with f_slck = 8 MHz)

#define HYSTERESIS_SIZE 5

#define MIN_FREQ 42.5               // max. and min. value of input signal's frequencies
#define MAX_FREQ 69

#define CHECK_FREQ_MIN(f) (((f) < (MIN_FREQ)) ? (MIN_FREQ) : (f))
#define CHECK_FREQ_MAX(f) (((f) > (MAX_FREQ)) ? (MAX_FREQ) : (f))

#define REFERENCE_INPUT 3.3

//// Ports for QuadSPI access on RoboYagi board.
//fl_QSPIPorts ports = {
//  XS1_PORT_1B,
//  XS1_PORT_1C,
//  XS1_PORT_4B,
//  on tile[0]: XS1_CLKBLK_1
//};
//
//// List of QuadSPI devices that are supported by default.
//fl_QuadDeviceSpec deviceSpecs[] =
//{
//  FL_QUADDEVICE_SPANSION_S25FL116K,
//  FL_QUADDEVICE_SPANSION_S25FL132K,
//  FL_QUADDEVICE_SPANSION_S25FL164K,
//  FL_QUADDEVICE_ISSI_IS25LQ080B,
//  FL_QUADDEVICE_ISSI_IS25LQ016B,
//  FL_QUADDEVICE_ISSI_IS25LQ032B,
//};

/**< [[Nullable]] Array of 32-bit buffered ADC data ports. */
/**< [[Nullable]] Internal XMOS clock. */
/**< [[Nullable]] Port connecting to external ADC serial clock. */
/**< [[Nullable]] Port used to as ready signal for p32_adc_data ports and ADC chip. */
/**< [[Nullable]] 4-bit Port used to control multiplexor on ADC chip. */

ADCPorts adc_ports = { { null, null, null, null },
        {{XS1_PORT_1D, XS1_PORT_1C}, XS1_CLKBLK_1, XS1_PORT_1E, XS1_PORT_1F, XS1_PORT_4D },
                        {1, -1, 50}};

/* DC offset measuring function
* result = (maximum value + minimum value) / 2
* warning: reference clock on board is 250 MHz
* time monitored for dc offset calculation is 0.25 s
*/

int dc_offset_measurement(client interface ADCInterface adc_if)
{
    timer t;
    unsigned ts, delay_offset;

    int sample_a, sample_b;
    int max_sample = 0, min_sample = 0, dc_offset = 0, flag_done = 0;

    delay_offset = 62500000;  // 0.25 s
    t :> ts;

    while(!flag_done)
    {
        {sample_a, sample_b} = adc_if.get_temperature();

        select
        {
        case t when timerafter(ts + delay_offset) :> void:
                dc_offset = (max_sample + min_sample) / 2;
                flag_done = 1;
                break;
        default:
            if (sample_b > max_sample)
                max_sample = sample_b;
            if (sample_b < min_sample)
                min_sample = sample_b;
            break;
        }
    }

    return dc_offset;
}

/* Frequency measurement function
* measurement is based on the dc offset crossing detection (from lower to higher values comparing to dc offset)
* period of the signal is measured, after 1 second mean value of measured periods is calculated
* number of periods is in the variable
* hysteresis around dc_offset is used (size of hysteresis is 5 samples)
* sampling frequency in this measurement is reduced from 500 kHz to 5 KHz -> better accuracy of freq. results
* frequency is passed over a channel to the task that collects the data
*/

void frequency_measurement(client interface ADCInterface adc_if, chanend c)
{
    int dc_offset, sample_a, sample_b, hysteresis[HYSTERESIS_SIZE];

    timer t;
    float frequency;

    unsigned ts, time_crossing,  time_last_crossing, measurement_time = 250000000;                // measurement time = 1 second
    unsigned flag_exit = 0, sum_periods = 0, num_periods = 0, num_samples = 0;

    // dc offset information
    dc_offset =  dc_offset_measurement(adc_if);

    // measurement begins
    t :> ts;

    while(!flag_exit)
    {
        {sample_a, sample_b} = adc_if.get_temperature();

        select
        {
            // measurement ends
            case t when timerafter (ts + measurement_time) :> void:
                    flag_exit = 1;
                    break;

            // measurement
            default:
                if (num_samples % 100 == 0)
                {
                    if (num_samples > ((HYSTERESIS_SIZE-1)*100) && (sample_b > dc_offset) && (hysteresis[0] > dc_offset) && (hysteresis[1] > dc_offset) &&
                            (hysteresis[2] < dc_offset) && (hysteresis[3] < dc_offset) && hysteresis[4] < dc_offset)
                    {
                        t :> time_crossing;
                        if (num_periods > 0)
                        {
                            sum_periods += time_crossing - time_last_crossing;
                        }

                        // timestamp of last dc offset crossing and number of periods of a signal
                        time_last_crossing = time_crossing;
                        num_periods++;
                    }

                    // saving last HYSTERESIS_SIZE values of a signal
                    hysteresis[4] = hysteresis[3];
                    hysteresis[3] = hysteresis[2];
                    hysteresis[2] = hysteresis[1];
                    hysteresis[1] = hysteresis[0];
                    hysteresis[0] = sample_b;
                }
                break;
        }

        num_samples++;
    }

    // period of the signal as a mean value (float)sum_periods/(num_periods-1)
    // frequency = measured time / period
    frequency = measurement_time/((float)sum_periods/(num_periods-1));

    // frequency must be inside specified interval
    CHECK_FREQ_MIN(frequency);
    CHECK_FREQ_MAX(frequency);

    c <: frequency;
//    printf("Frequency = %.5f Hz, DC offset = %d Num of periods = %d\n", frequency, dc_offset, num_periods-1);
}

/* Client for three phases
 * Handles sampling and preprocessing of the data
 * Reads input frequency over the channel
 * Samples the signal with 50 kHz sampling frequency
 * Since there is only one analog input at the disposal,
 * it's simulated that data sequentially are being read from the same channel (input by input)
 */

void adc_client(client interface ADCInterface adc_if, client interface SwapBuffer swap_if, chanend c, int buffer_data[])
{
    int sample_a, sample_b, ticks = 0, additional = 0;
    int num_samples = 0, counter = 0;
    int * movable p = &buffer_data[0];
    int signals_sampled = 0;
    int flag = 0;

    select
    {
    case c :> float frequency:
        num_samples = FS/frequency;
        printf("Frequency = %f, number of samples = %d\n", frequency, num_samples);
        break;
    }

    while (1)
    {
        {sample_a, sample_b} = adc_if.get_temperature();

        ticks++;
//        xscope_int(SAMPLE_B, sample_b);

        if(sample_b == 0)
            flag = 1;

        // sampling frequency from 250 KHz to 50 kHz
        if (flag == 1 && ticks % 5 == 0)
        {
            /* Preprocessing idea
             * Based on the frequency, expected number of samples per one period is calculated
             * data is put into the buffer by applying the decimation algorithm
             * buffer's size is N_FFT_POINTS
             * If frequency changes during the process,
             * system needs to be restarted so that new number of samples
             * per one period could be calculated
             *
             * Decimation algorithm:
             *
             * if (number_of_samples + 1) / N_FFT_POINTS >= 2,
             *      take every second sample until you fill the N_FFT_POINTS
             *
             * else,
             *      N_FFT_POINTS - [(number_of_samples + 1) / 2] is the number of additional samples needed
             *      till 2 * additional_samples - 1 take samples from buffer
             *      after that number, take every second sample
             */

            if ((num_samples+1) / N_FFT_POINTS >= 2)
            {
                if (ticks % 2 == 0 && counter < N_FFT_POINTS)
                {
                    p[signals_sampled*N_FFT_POINTS + counter++] = Q16((float)sample_b/MAX_ADC_VALUE*REFERENCE_INPUT);
//                    xscope_int(SAMPLE_B, sample_b);
                }
            }
            else
            {
                int help_a = N_FFT_POINTS - (num_samples +1) / 2;

                if ((additional <= 2 * help_a - 1) || (ticks % 2 == 0 && counter < N_FFT_POINTS))
                {
                    if(additional <= 2 * help_a - 1)
                        additional++;

                    p[signals_sampled*N_FFT_POINTS + counter++] = Q16((float)sample_b/MAX_ADC_VALUE*REFERENCE_INPUT);
//                    xscope_int(SAMPLE_B, sample_b);
                }
            }

            if (counter == N_FFT_POINTS)
            {
                counter = 0;
                additional = 0;
                signals_sampled++;
            }
        }

        // all three phases sampled and buffered
        if(signals_sampled == NUM_CHANNELS)
        {
            swap_if.swap(p);
            signals_sampled = 0;
        }
    }
}

void xscope_user_init(void)
{
  xscope_register(1, XSCOPE_CONTINUOUS, "SAMPLE_B", XSCOPE_INT, "Value",
                  2, XSCOPE_CONTINUOUS, "MAX_SAMPLE", XSCOPE_INT, "Value",
                  3, XSCOPE_CONTINUOUS, "MIN_SAMPLE", XSCOPE_INT, "Value");
}

int main()
{
    interface ADCInterface adc_if[5];
    interface SwapBuffer swap_if;
    chan c;

    int buffer_data[NUM_CHANNELS*N_FFT_POINTS], buffer_calc[NUM_CHANNELS*N_FFT_POINTS];

//    // erasing the whole stuff
//    if( 0 == fl_connectToDevice(ports, deviceSpecs, sizeof(deviceSpecs)/sizeof(fl_QuadDeviceSpec)))
//    {
//        fl_eraseAll();
//        fl_disconnect();
//        return 1;
//    }
//
//    // Disconnect from the QuadSPI device.
//    fl_disconnect();

    par
    {
        frequency_measurement(adc_if[0],c);    // initialization process (detecting the frequency -> basis of preprocessing)

        adc_client(adc_if[1], swap_if, c, buffer_data);  // buffering the data

        adc_service(adc_ports, null, adc_if);     // ADC service

        task_calculations(swap_if, buffer_calc);  // calculations
    }

    return 0;
}
