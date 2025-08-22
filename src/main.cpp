

#include <iostream>

#include <gnuradio/analog/random_uniform_source.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/probe_rate.h>
#include <gnuradio/blocks/complex_to_interleaved_short.h>
#include <gnuradio/dtv/dvb_bbheader_bb.h>
#include <gnuradio/dtv/dvb_bbscrambler_bb.h>
#include <gnuradio/dtv/dvb_bch_bb.h>
#include <gnuradio/dtv/dvb_config.h>
#include <gnuradio/dtv/dvb_ldpc_bb.h>
#include <gnuradio/dtv/dvbs2_config.h>
#include <gnuradio/dtv/dvbs2_interleaver_bb.h>
#include <gnuradio/dtv/dvbs2_modulator_bc.h>
#include <gnuradio/dtv/dvbs2_physical_cc.h>
#include <gnuradio/filter/fft_filter_ccf.h>
#include <gnuradio/filter/interp_fir_filter.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/top_block.h>
#include <signal.h>
#include <stdio.h>
#include <vector>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "rfnm_sink_impl.h"

#define SYM_RATE 8e6
#define CARRIER_FREQ 1e9

#define CODE_RATE       gr::dtv::C3_4
#define FEC_FRAME_SIZE  gr::dtv::FECFRAME_NORMAL
//#define FEC_FRAME_SIZE  gr::dtv::FECFRAME_SHORT
//MOD_16QAM
//MOD_8PSK
//MOD_QPSK3_4
//MOD_16APSK
#define CONSTELLATION   gr::dtv::MOD_QPSK

static bool keep_running;

static void signal_handler(int signo)
{
    if (signo == SIGINT)
        fputs("\nCaught SIGINT\n", stderr);
    else if (signo == SIGTERM)
        fputs("\nCaught SIGTERM\n", stderr);
    else if (signo == SIGHUP)
        fputs("\nCaught SIGHUP\n", stderr);
    else if (signo == SIGPIPE)
        fputs("\nReceived SIGPIPE.\n", stderr);
    else
        fprintf(stderr, "\nCaught signal: %d\n", signo);

    keep_running = false;
}

using namespace std;

int main(){
	auto name = "agent_lc";
	auto console = spdlog::stdout_color_mt("console");
	spdlog::get("console")->info("Starting DVB-S2 transmitter");


	gr::top_block_sptr                  tb;
	gr::blocks::probe_rate::sptr        ts_probe;
	gr::blocks::probe_rate::sptr        iq_probe;
	gr::blocks::file_source::sptr       ts_in_file;
	//gr::analog::random_uniform_source_b::sptr  random_source;
	gr::dtv::dvb_bbheader_bb::sptr      bb_header;
	gr::dtv::dvb_bbscrambler_bb::sptr   bb_scrambler;
	gr::dtv::dvb_bch_bb::sptr           bch_enc;
	gr::dtv::dvb_ldpc_bb::sptr          ldpc_enc;
	gr::dtv::dvbs2_interleaver_bb::sptr interleaver;
	gr::dtv::dvbs2_modulator_bc::sptr   modulator;
	gr::dtv::dvbs2_physical_cc::sptr    pl_framer;
	gr::filter::fft_filter_ccf::sptr    filter;
	//gr::filter::interp_fir_filter_ccf::sptr	filter;
	gr::blocks::complex_to_interleaved_short::sptr typeConverter;
	std::vector<float>      filter_taps;
	gr::grRFNM::rfnm_sink::sptr rfnmSink;
	//Some workaround for static initalization issues on aarch64?
	std::ios_base::Init init;

	tb = gr::make_top_block("dvbs2_tx");
	bb_header = gr::dtv::dvb_bbheader_bb::make(gr::dtv::STANDARD_DVBS2,
												FEC_FRAME_SIZE,
											   CODE_RATE,
											   gr::dtv::RO_0_35,
											   gr::dtv::INPUTMODE_NORMAL,
											   gr::dtv::INBAND_OFF,
											   0, 0);
	bb_scrambler = gr::dtv::dvb_bbscrambler_bb::make(gr::dtv::STANDARD_DVBS2,
													FEC_FRAME_SIZE,
													 CODE_RATE);
	bch_enc = gr::dtv::dvb_bch_bb::make(gr::dtv::STANDARD_DVBS2,
										FEC_FRAME_SIZE,
										CODE_RATE);
	ldpc_enc = gr::dtv::dvb_ldpc_bb::make(gr::dtv::STANDARD_DVBS2,
										FEC_FRAME_SIZE,
										CODE_RATE,
										CONSTELLATION);
	interleaver = gr::dtv::dvbs2_interleaver_bb::make(FEC_FRAME_SIZE,
													  CODE_RATE,
													  CONSTELLATION);
	modulator = gr::dtv::dvbs2_modulator_bc::make(FEC_FRAME_SIZE,
												  CODE_RATE,
												  CONSTELLATION,
												  gr::dtv::INTERPOLATION_OFF);
	pl_framer = gr::dtv::dvbs2_physical_cc::make(FEC_FRAME_SIZE,
												 CODE_RATE,
												 CONSTELLATION,
												 gr::dtv::PILOTS_ON,
												 0);

	filter_taps = gr::filter::firdes::root_raised_cosine(1.0,
														 SYM_RATE,
														 SYM_RATE/2,
														 0.35, 31);
	filter = gr::filter::fft_filter_ccf::make(1, filter_taps, 1);
	//filter = gr::filter::interp_fir_filter_ccf::make(2, filter_taps);

	//Convert to 12-bit
	//16383
	//32767
	//typeConverter = gr::blocks::complex_to_interleaved_short::make(true, 20000);

	//double      freq_hz = conf.rf_freq * (1000000.0 - conf.ppm) / 1000000.0;
	//double      rate_hz = (2.0 * conf.sym_rate) * ((1000000.0 - conf.ppm) / 1000000.0);

	try
	{
		rfnmSink = gr::grRFNM::rfnm_sink::make(SYM_RATE, CARRIER_FREQ, 1250, 40);
	}
	catch (std::runtime_error &e)
	{
		fprintf(stderr, "\n*** Error creating rfnm sink: %s\n\n", e.what());
		return 0;
	}


	//random_source = gr::analog::random_uniform_source_b::make(0, 255, 0);
	ts_in_file = gr::blocks::file_source::make(sizeof(char), "/root/qpsk_3_4_4msymbols.ts", true);
	tb->connect(ts_in_file, 0, bb_header, 0);

	//tb->connect(random_source, 0, bb_header, 0);
	tb->connect(bb_header, 0, bb_scrambler, 0);
	tb->connect(bb_scrambler, 0, bch_enc, 0);
	tb->connect(bch_enc, 0, ldpc_enc, 0);
	tb->connect(ldpc_enc, 0, interleaver, 0);
	tb->connect(interleaver, 0, modulator, 0);
	tb->connect(modulator, 0, pl_framer, 0);
	//tb->connect(pl_framer, 0, typeConverter, 0);
	tb->connect(pl_framer, 0, filter, 0);
	//tb->connect(filter, 0, typeConverter, 0);
	tb->connect(filter, 0, rfnmSink, 0);

	/*if (conf.probe)
	{
		fprintf(stderr, "Enabling rate probes\n");
		ts_probe = gr::blocks::probe_rate::make(sizeof(char), 1000, 0.5);
		if (conf.udp_input)
			tb->connect(ts_in_udp, 0, ts_probe, 0);
		else
			tb->connect(ts_in_file, 0, ts_probe, 0);

		iq_probe = gr::blocks::probe_rate::make(sizeof(gr_complex), 1000, 0.5);
		tb->connect(filter, 0, iq_probe, 0);
	}*/

	tb->start();
	//fputs("Flow graph running\n", stderr);
	spdlog::info("Finished flow graph, start demo!");
	keep_running = true;
	while (keep_running)
	{
		sleep(1);

		/*if (ctl_if_poll())
		{
			int pwr = ctl_if_get_tx_power();
			if (pwr < 0)
				conf.gain = 0;
			else if (pwr > 47)
				conf.gain = 47;
			else
				conf.gain = pwr;

			set_gain(iq_sink, conf.gain);
		}

		if (conf.probe)
			fprintf(stderr, "dvbs2_tx:  %.3f kbps in;  %.3f ksps out\n",
					8.0e-3 * ts_probe->rate(), 1.0e-3 * iq_probe->rate());
		*/
	}

	//fputs("Stopping flow graph\n", stderr);
	spdlog::info("Stop demo and shutting down!");
	tb->stop();

	return 0;
}
