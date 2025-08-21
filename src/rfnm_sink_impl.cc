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

#include "librfnm/device.h"
#include "librfnm/constants.h"

pthread_mutex_t th;


#define BYTES_PER_CS16 4
#define WAIT_FOR_EMPTY_BUFFER_MS 2

namespace gr {
  namespace grRFNM {

    rfnm_sink::sptr
	rfnm_sink::make(float samp_rate, float carrier_freq) {
    	return gnuradio::get_initial_sptr(new rfnm_sink_impl(samp_rate, carrier_freq));
    }

    /*
     * The private constructor
     */
    rfnm_sink_impl::rfnm_sink_impl(float samp_rate, float carrier_freq) :
    		gr::sync_block("rfnm_sink",  gr::io_signature::make(1, 1, sizeof(int16_t) * 2), gr::io_signature::make(0, 0, 0))
    {
		//TODO: Create necessary structures here
    	bufFillIndex = 0;
    	bufRotationIndex = 0;
        samp_rate_=samp_rate;
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
        rfnmDevice->set_tx_channel_power(tx_ch.id, 40);
        rfnmDevice->set_tx_channel_freq(tx_ch.id, RFNM_MHZ_TO_HZ((int)(carrier_freq / 1e6)));

        if (auto lret = rfnmDevice->apply(tx_ch.apply)) {
        	spdlog::error("Error applying TX channel configuration: {} (code: {})\n", rfnm::device::failcode_to_string(lret), lret);
			return;
		}



        this->rfnmDevice->set_stream_format(rfnm::STREAM_FORMAT_CS16, &inbufsize, &bytes_per_ele);
        spdlog::info("bufsize: {}, {} bytes per element", inbufsize, bytes_per_ele);
        this->dequed = 0;
        // Allocate and prepare buffers
	    for (int i = 0; i < NBUF; ++i) {
		    txbuf[i].buf = (uint8_t*)malloc(inbufsize);
		    txBufferTrack[i] = txbuf[i].buf; 	//Ensure we can track the allocated buffers
		    memset(txbuf[i].buf, 0, inbufsize);
		    ltxqueue.push(&txbuf[i]);
	    }
	    //TX_LATENCY_POLICY_AGGRESSIVE
	    //TX_LATENCY_POLICY_DEFAULT
        rfnmDevice->tx_work_start(rfnm::TX_LATENCY_POLICY_RELAXED); //TX_LATENCY_POLICY_RELAXED
        this->tstart = std::chrono::high_resolution_clock::now();

        spdlog::info("Initialized RFNM Sink with carrier freq {} Hz and sample rate {} and num buffers: {}", carrier_freq, samp_rate, ltxqueue.size());
        if (ltxqueue.empty()) {
        	spdlog::warn("For whatever reason ltxqueue is empty!");
        }
    }

    /*
     * Our virtual destructor.
     */
    rfnm_sink_impl::~rfnm_sink_impl(){
    	//TODO: Delete created structures here
    	rfnmDevice->tx_work_stop();
    	delete rfnmDevice;
    	while(!ltxqueue.empty()){
    		auto first_txbuf = ltxqueue.front();
    		ltxqueue.pop();
    	}
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
    	const int16_t* in = (const int16_t*)input_items[0]; // Already CS16: [I0, Q0, I1, Q1, ...]
		uint32_t bytes_copied = 0;
		uint32_t samples_to_copy = noutput_items;
		pthread_mutex_lock(&th);
		struct rfnm::tx_buf* txbuf = nullptr;
		struct rfnm::tx_buf* donebuf = nullptr;
		if (ltxqueue.empty()) {
			//pthread_mutex_unlock(&th);
			//spdlog::error("No available TX buffer, wait for dequeue! In number of elements: {}", noutput_items);
			// Wait for a buffer to complete and retrieve it
			while (rfnmDevice->tx_dqbuf(&donebuf) != RFNM_API_OK) {
				std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_EMPTY_BUFFER_MS));
			}
			donebuf->buf = txBufferTrack[bufRotationIndex++];
			if(bufRotationIndex >= NBUF){
				bufRotationIndex = 0;
			}
			ltxqueue.push(donebuf);
			//We now have at least one free buffer
		}
		auto first_txbuf = ltxqueue.front();

		do {
			if((bufFillIndex + samples_to_copy * BYTES_PER_CS16) >= inbufsize){
				//We will fill this buffer and maybe more
				uint32_t remainder = inbufsize - bufFillIndex;
				//Copy data and send buffer
				memcpy(first_txbuf->buf+bufFillIndex, in+bytes_copied, remainder);
				samples_to_copy -= remainder / BYTES_PER_CS16;	//This counter is in elements, not in bytes
				bytes_copied += remainder;
				if (rfnmDevice->tx_qbuf(first_txbuf) == RFNM_API_OK) {
					dequed++;
				} else {
					//Technically without an exception this code cannot be reached
					ltxqueue.push(first_txbuf); // Requeue on failure
					spdlog::error("Could not queue buffer!");
					return samples_to_copy;
				}
				bufFillIndex = 0;
				ltxqueue.pop();
				if (ltxqueue.empty()) {
					//pthread_mutex_unlock(&th);
					//spdlog::warn("No available TX buffer for data transfer, wait for deque!");
					// Dequeue completed TX buffers
					while (rfnmDevice->tx_dqbuf(&donebuf) != RFNM_API_OK) {
						std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_EMPTY_BUFFER_MS));
					}
					donebuf->buf = txBufferTrack[bufRotationIndex++];
					if(bufRotationIndex >= NBUF){
						bufRotationIndex = 0;
					}
					ltxqueue.push(donebuf);
					//was return
				}
				first_txbuf = ltxqueue.front();
			} else {
				//We wont fill this buffer
				memcpy(first_txbuf->buf+bufFillIndex, in+bytes_copied, samples_to_copy * BYTES_PER_CS16);
				bufFillIndex += samples_to_copy * BYTES_PER_CS16;
				bytes_copied += samples_to_copy * BYTES_PER_CS16;
				samples_to_copy = 0;
			}
		} while(samples_to_copy > 0);

		// Dequeue completed TX buffers
		while(rfnmDevice->tx_dqbuf(&donebuf) == RFNM_API_OK) {
			donebuf->buf = txBufferTrack[bufRotationIndex++];
			if(bufRotationIndex >= NBUF){
				bufRotationIndex = 0;
			}
			ltxqueue.push(donebuf);
		}

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
  }
}
