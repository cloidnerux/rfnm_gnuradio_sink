/*
 * rfnm_sink_impl.h
 *
 *  Created on: 03.12.2024
 *      Author: cloid
 */

#ifndef SRC_RFNM_SINK_IMPL_H_
#define SRC_RFNM_SINK_IMPL_H_

#include "rfnm_sink.h"
#include <gnuradio/types.h>
//#include "rfnmConfig.h"

#define BUILD_RFNM_LOCAL_TRANSPORT

#include "librfnm/device.h"
#include "librfnm/constants.h"
#include <queue>
#include <chrono>

#define NBUF 8 // TX buffer count

namespace gr {
  namespace grRFNM {

    class rfnm_sink_impl : public rfnm_sink
    {
     private:
      //iqdmasync *iqtest;
      float samp_rate_;
      //rfnmConfig rfnm;
      rfnm::device * rfnmDevice;
      int dequed;
      std::chrono::time_point<std::chrono::high_resolution_clock> tstart;
      size_t inbufsize;
	  uint8_t bytes_per_ele;
	  //Ring buffer of tx buffers
	  struct rfnm_tx_usb_buf * txbufs[NBUF];
	  //Ring buffer of data buffers
	  //uint8_t * txBufferTrack[NBUF];

	  uint32_t bufFillIndex;
	  uint32_t bufRotationIndex;
	  uint32_t elementsPerBuffer;
	  float dScale;
	  uint64_t packetCounter;
	  int retryCount;

     public:
      void set_freq(float);
      rfnm_sink_impl(float samp_rate, float carrier_freq, float scale, uint16_t power);
      ~rfnm_sink_impl();

      // Where all the action really happens
      int work(
              int noutput_items,
              gr_vector_const_void_star &input_items,
              gr_vector_void_star &output_items
      );
     protected:
     void convert_and_pack_complex_float_to_int12(const float* src, uint8_t* dst, int count, float scale);
    };

  } // namespace rfnm
} // namespace gr



#endif /* SRC_RFNM_SINK_IMPL_H_ */
