/*
 * rfnm_sink.h
 *
 *  Created on: 03.12.2024
 *      Author: cloid
 */

#ifndef SRC_RFNM_SINK_H_
#define SRC_RFNM_SINK_H_

#include <gnuradio/attributes.h>

#ifdef rfnm_sink_EXPORTS
#define RFNM_API __GR_ATTR_EXPORT
#else
#define RFNM_API __GR_ATTR_IMPORT
#endif

#include <gnuradio/sync_block.h>



namespace gr {
  namespace grRFNM {

    /*!
     * \brief <+description of block+>
     * \ingroup rpitx
     *
     */
    class RFNM_API rfnm_sink : virtual public gr::sync_block
    {
     public:
      typedef std::shared_ptr<rfnm_sink> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of rfnm::rfnm_sink.
       *
       * To avoid accidental use of raw pointers, rfnm::rfnm_sink
       * constructor is in a private implementation
       * class. rfnm::rfnm_sink::make is the public interface for
       * creating new instances.
       */
	static sptr make(float samp_rate, float carrier_freq);
	virtual void set_freq(float)=0;
    };

  }
}




#endif /* SRC_RFNM_SINK_H_ */
