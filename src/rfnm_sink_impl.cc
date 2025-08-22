/*
 * rfnm_sink_impl.cc
 *
 *  Created on: 03.12.2024
 *      Author: cloid
 */




#include <gnuradio/io_signature.h>
#include "rfnm_sink_impl.h"

#include <unistd.h>
#include "stdio.h"
#include <cstring>
#include <signal.h>
#include <stdlib.h>
#include <pthread.h>
#include <queue>
#include <chrono>
#include <spdlog/spdlog.h>
#include <arm_neon.h>
#include <stdint.h>

#include "librfnm/device.h"
#include "librfnm/constants.h"

pthread_mutex_t th;


#define BYTES_PER_CS16 4
#define WAIT_FOR_EMPTY_BUFFER_MS 2

namespace gr {
  namespace grRFNM {

    rfnm_sink::sptr
	rfnm_sink::make(float samp_rate, float carrier_freq, float scale, uint16_t power) {
    	return gnuradio::get_initial_sptr(new rfnm_sink_impl(samp_rate, carrier_freq, scale, power));
    }

    /*
     * The private constructor
     */
    rfnm_sink_impl::rfnm_sink_impl(float samp_rate, float carrier_freq, float scale, uint16_t power) :
    		gr::sync_block("rfnm_sink",  gr::io_signature::make(1, 1, sizeof(gr_complex)), gr::io_signature::make(0, 0, 0))
    {
		//TODO: Create necessary structures here
    	bufFillIndex = 0;
    	bufRotationIndex = 0;
        samp_rate_= samp_rate;
        dScale = scale;
        packetCounter = 0;
        retryCount = 0;
        pthread_mutex_init(&th, NULL);
        //Magic values from example
        //uint32_t ddrBufSize, uint32_t dataChunkWidth
        //rfnm.TriggerTransmission(4915200, 0x1000);
        //TRANSPORT_LOCAL
        //TRANSPORT_FIND
        rfnmDevice = new rfnm::device(rfnm::TRANSPORT_LOCAL);
        //this->rfnmDevice->set_samp_rate(122880000 / 4); // for now tx frequency is half this, so max is 61 msps
        rfnmDevice->set_samp_rate(samp_rate); // for now tx frequency is half this, so max is 61 msps

        rfnm::ch_helper tx_ch = rfnmDevice->tx_ch_helper(0);

        rfnmDevice->set_tx_channel_status(tx_ch.id, RFNM_CH_RF_ON, RFNM_CH_STREAM_ON);
        rfnmDevice->set_tx_channel_path(tx_ch.id, RFNM_PATH_SMA_A);	//RFNM_PATH_EMBED_ANT
        rfnmDevice->set_tx_channel_rfic_lpf_bw(tx_ch.id, int(2*samp_rate/1e6));
        rfnmDevice->set_tx_channel_power(tx_ch.id, power);
        rfnmDevice->set_tx_channel_freq(tx_ch.id, RFNM_MHZ_TO_HZ((int)(carrier_freq / 1e6)));

        if (auto lret = rfnmDevice->apply(tx_ch.apply)) {
        	spdlog::error("Error applying TX channel configuration: {} (code: {})\n", rfnm::device::failcode_to_string(lret), lret);
			return;
		}



        this->rfnmDevice->set_stream_format(rfnm::STREAM_FORMAT_CS16, &inbufsize, &bytes_per_ele);
        elementsPerBuffer = inbufsize / bytes_per_ele;
        // inbufsize is the size of the buffer containing the CS16 samples
        // Therefore it is NUM_ELEMENTS * 4 bytes for CS16
        spdlog::info("bufsize: {}, {} bytes per element", inbufsize, bytes_per_ele);

        // Allocate and prepare buffers
	    for (int i = 0; i < NBUF; ++i) {
	    	//txbufs are the ring buffers for the PCIe streaming. The struct contains hard coded buffer sizes
		    txbufs[i] = (struct rfnm_tx_usb_buf*)malloc(sizeof(struct rfnm_tx_usb_buf));
		    txbufs[i]->magic = 0x758f4d4a;
		    txbufs[i]->dac_cc = 0;
		    txbufs[i]->dac_id = 0;
		    txbufs[i]->dropped = 0;
		    txbufs[i]->padding[0] = 0;
		    txbufs[i]->phytimer = 0;
		    txbufs[i]->usb_cc = 0;
		    //These buffers are for the incoming data
		    //txBufferTrack[i] = (uint8_t*)malloc(inbufsize); 	//Ensure we can track the allocated buffers
		    //memset(txBufferTrack[i], 0, inbufsize);
	    }
	    dequed = 0;
	    //TX_LATENCY_POLICY_AGGRESSIVE
	    //TX_LATENCY_POLICY_DEFAULT
	    //We queue the buffer directly, no need for the threads
        //rfnmDevice->tx_work_start(rfnm::TX_LATENCY_POLICY_RELAXED); //TX_LATENCY_POLICY_RELAXED
        this->tstart = std::chrono::high_resolution_clock::now();

        spdlog::info("Initialized RFNM Sink with carrier freq {} Hz and sample rate {} and num power: {}", carrier_freq, samp_rate, power);
    }

    /*
     * Our virtual destructor.
     */
    rfnm_sink_impl::~rfnm_sink_impl(){
    	//TODO: Delete created structures here
    	rfnmDevice->tx_work_stop();
    	delete rfnmDevice;
    	for(int i = 0; i < NBUF; i++){
    		free(txBufferTrack[i]);
    	}
    	pthread_mutex_destroy(&th);
    }

    void rfnm_sink_impl::set_freq(float freq){
    	//printf("new frequency: %f\n",freq);
    	pthread_mutex_lock(&th);


    	rfnmDevice->set_tx_channel_freq(0, RFNM_MHZ_TO_HZ((int)(freq / 1e6)));
    	//rfnmDevice->apply(rfnm::APPLY_CH0_TX);

    	pthread_mutex_unlock(&th);
    }

    int rfnm_sink_impl::work(int noutput_items, gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)    {
    	const float* in = (const float*)input_items[0]; // Already CS16: [I0, Q0, I1, Q1, ...]
		uint32_t bytes_copied = 0;
		uint32_t samples_to_copy = noutput_items;
		pthread_mutex_lock(&th);

		//First do some buffer management. Copy the incoming samples in the currently free buffer while checking the fill
		//bufFillIndex is in number of items in the buffer
		do {
			if((bufFillIndex + samples_to_copy) >= elementsPerBuffer) {
				//We will fill this buffer
				convert_and_pack_complex_float_to_int12(in, &txbufs[bufRotationIndex]->buf[bufFillIndex*3], elementsPerBuffer-(bufFillIndex + samples_to_copy), dScale);
				txbufs[bufRotationIndex]->usb_cc = packetCounter++;
				while(rfnmDevice->queueBuffer(txbufs[bufRotationIndex++]) != RFNM_API_OK){
					std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_EMPTY_BUFFER_MS));
					retryCount++;
					if(retryCount == 3){
						spdlog::error("Could not queue buffer to LA9310!");
						retryCount = 0;
						break;
					}
				}
				if(bufRotationIndex >= NBUF){
					bufRotationIndex = 0;
				}
				samples_to_copy -= elementsPerBuffer-(bufFillIndex + samples_to_copy);
				bufFillIndex = 0;

			} else {
				//Not enough elements to fill the buffer, just copy the data
				//We need 3 byte per complex data point
				convert_and_pack_complex_float_to_int12(in, &txbufs[bufRotationIndex]->buf[bufFillIndex*3], noutput_items, dScale);
				bufFillIndex += noutput_items;
				samples_to_copy -= noutput_items;
			}
		}while(samples_to_copy > 0);

		// Optional: Print TX stats
		auto tnow = std::chrono::high_resolution_clock::now();
		auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(tnow - tstart);
		if (ms_int.count() > 1000) {
			float sps = (inbufsize * dequed / 4.0f);
			spdlog::info("TX Rate: {} Msps, {} Mbps, nelements {}, inbufsize {}, bufFillIndex {}", sps / 1e6, (sps * 24) / 1e6, noutput_items, inbufsize, bufFillIndex);
			dequed = 0;
			tstart = tnow;
		}

		pthread_mutex_unlock(&th);

		return noutput_items; // We consumed these many input items
    }

    static inline int16_t float_to_int12_saturate(float x, float scale) {
        int32_t scaled = (int32_t)roundf(x * scale);
        if (scaled > 2047) return 2047;
        if (scaled < -2048) return -2048;
        return (int16_t)scaled;
    }

    //Straight from chatgpt
    //The gnuradio block sues volk
    //https://github.com/gnuradio/gnuradio/blob/main/gr-blocks/lib/complex_to_interleaved_short_impl.cc
    //volk_32f_s32f_convert_16i
    void rfnm_sink_impl::convert_and_pack_complex_float_to_int12(const float* src, uint8_t* dst, int count, float scale) {
        int i = 0;

        for (; i <= count - 4; i += 4) {
            // Load 4 complex values (8 floats)
            float32x4x2_t complex = vld2q_f32(src + 2 * i);

            float32x4_t scaled_real = vmulq_n_f32(complex.val[0], scale);
            float32x4_t scaled_imag = vmulq_n_f32(complex.val[1], scale);

            // Convert to int32
            int32x4_t real_i32 = vcvtq_s32_f32(scaled_real);
            int32x4_t imag_i32 = vcvtq_s32_f32(scaled_imag);

            // Signed saturating extract Narrow, half the width of the integer
            int16x4_t real_i16 = vqmovn_s32(real_i32);
            int16x4_t imag_i16 = vqmovn_s32(imag_i32);

            // Store multiple single-element structures from one, two, three, or four registers.
            int16_t real_vals[4], imag_vals[4];
            vst1_s16(real_vals, real_i16);
            vst1_s16(imag_vals, imag_i16);

            // Scalar packing to 12-bit
            for (int j = 0; j < 4; ++j) {
                int16_t real = real_vals[j];
                int16_t imag = imag_vals[j];

                // Clamp to 12-bit
                //real = real > 2047 ? 2047 : (real < -2048 ? -2048 : real);
                //imag = imag > 2047 ? 2047 : (imag < -2048 ? -2048 : imag);

                // Convert to unsigned 12-bit two's complement
                //uint16_t real_u = (uint16_t)(real & 0x0FFF);
                //uint16_t imag_u = (uint16_t)(imag & 0x0FFF);

                // Pack into 3 bytes
                dst[0] = real & 0xFF;
                dst[1] = ((real >> 8) & 0x0F) | ((imag & 0x0F) << 4);
                dst[2] = (imag >> 4) & 0xFF;

                dst += 3;
            }
        }

        // Tail processing
        for (; i < count; ++i) {
            int16_t real = float_to_int12_saturate(src[2 * i], scale);
            int16_t imag = float_to_int12_saturate(src[2 * i + 1], scale);

            uint16_t r = (uint16_t)(real & 0x0FFF);
            uint16_t im = (uint16_t)(imag & 0x0FFF);

            dst[0] = r & 0xFF;
            dst[1] = ((r >> 8) & 0x0F) | ((im & 0x0F) << 4);
            dst[2] = (im >> 4) & 0xFF;
            dst += 3;
        }
    }
    /*void rfnm_sink_impl::convert_complex_float_to_int16_neon(const float* src, int16_t* dst, int count, float scale) {
        int i = 0;
        float32x4_t scale_vec = vdupq_n_f32(scale);  // Load scale to SIMD

        for (; i <= count - 4; i += 4) {
            // Load 4 complex values = 8 float32
            float32x4x2_t complex = vld2q_f32(src + 2 * i);  // Deinterleaves to real/imag

            // Scale both real and imag
            float32x4_t real_scaled = vmulq_f32(complex.val[0], scale_vec);
            float32x4_t imag_scaled = vmulq_f32(complex.val[1], scale_vec);

            // Convert to int16 with saturation
            int16x4_t real_i16 = vqmovn_s32(vcvtq_s32_f32(real_scaled));
            int16x4_t imag_i16 = vqmovn_s32(vcvtq_s32_f32(imag_scaled));

            // Interleave back
            int16x4x2_t result;
            result.val[0] = real_i16;
            result.val[1] = imag_i16;

            // Store 8 int16_t (4 complex) interleaved
            vst2_s16(dst + 2 * i, result);
        }

        // Fallback for remaining elements (if count not divisible by 4)
        for (; i < count; i++) {
            float real = src[2 * i] * scale;
            float imag = src[2 * i + 1] * scale;

            int16_t r = (int16_t)(real > 32767.0f ? 32767.0f : (real < -32768.0f ? -32768.0f : real));
            int16_t im = (int16_t)(imag > 32767.0f ? 32767.0f : (imag < -32768.0f ? -32768.0f : imag));

            dst[2 * i] = r;
            dst[2 * i + 1] = im;
        }
    }*/
  }
}
