# Low Latency Passthrough with volume Pipeline and PCM
#
# Pipeline Endpoints for connection are :-
#
#  host PCM_P --> B0 --> TDFB 0 --> B1 --> sink DAI0

# Include topology builder
include(`utils.m4')
include(`buffer.m4')
include(`pcm.m4')
include(`dai.m4')
include(`bytecontrol.m4')
include(`pipeline.m4')
include(`tdfb.m4')

#
# Controls
#

define(DEF_TDFB_PRIV, concat(`tdfb_priv_', PIPELINE_ID))

# Define filter. A passthrough is set by default.
ifdef(`PIPELINE_FILTER1', , `define(PIPELINE_FILTER1, `tdfb/coef_line2_pass.m4')')
include(PIPELINE_FILTER1)

# TDFB Bytes control with max value of 255
define(DEF_TDFB_BYTES, concat(`tdfb_bytes_', PIPELINE_ID))
C_CONTROLBYTES(DEF_TDFB_BYTES, PIPELINE_ID,
	CONTROLBYTES_OPS(bytes,
		258 binds the mixer control to bytes get/put handlers,
		258, 258),
	CONTROLBYTES_EXTOPS(258 binds the mixer control to bytes get/put handlers,
		258, 258),
	, , ,
	CONTROLBYTES_MAX(, 4096),
	,
	DEF_TDFB_PRIV)

#
# Components and Buffers
#

# Host "TDFB Playback" PCM
# with 2 sink and 0 source periods
W_PCM_PLAYBACK(PCM_ID, TDFB Playback, 2, 0)

# "TDFB 0" has x sink period and 2 source periods
W_TDFB(0, PIPELINE_FORMAT, DAI_PERIODS, 2, SCHEDULE_CORE,
	LIST(`		', "DEF_TDFB_BYTES"))

# Playback Buffers
W_BUFFER(0, COMP_BUFFER_SIZE(2,
	COMP_SAMPLE_SIZE(PIPELINE_FORMAT), PIPELINE_CHANNELS,
	COMP_PERIOD_FRAMES(PCM_MAX_RATE, SCHEDULE_PERIOD)),
	PLATFORM_HOST_MEM_CAP)
W_BUFFER(1, COMP_BUFFER_SIZE(DAI_PERIODS,
	COMP_SAMPLE_SIZE(DAI_FORMAT), PIPELINE_CHANNELS,
	COMP_PERIOD_FRAMES(PCM_MAX_RATE, SCHEDULE_PERIOD)),
	PLATFORM_DAI_MEM_CAP)

#
# Pipeline Graph
#
#  host host PCM_P --> B0 --> TDFB --> B3 --> sink DAI0

P_GRAPH(pipe-tdfb-playback, PIPELINE_ID,
	LIST(`		',
	`dapm(N_BUFFER(0), N_PCMP(PCM_ID))',
	`dapm(N_TDFB(0), N_BUFFER(0))',
	`dapm(N_BUFFER(1), N_TDFB(0))'))

#
# Pipeline Source and Sinks
#
indir(`define', concat(`PIPELINE_SOURCE_', PIPELINE_ID), N_BUFFER(1))
indir(`define', concat(`PIPELINE_PCM_', PIPELINE_ID),
	TDFB Playback PCM_ID)

#
# PCM Configuration

#
PCM_CAPABILITIES(TDFB Playback PCM_ID, `S32_LE,S24_LE,S16_LE',
	PCM_MIN_RATE, PCM_MAX_RATE, 2, PIPELINE_CHANNELS,
	2, 16, 192, 16384, 65536, 65536)

undefine(`DEF_TDFB_PRIV')
undefine(`DEF_TDFB_BYTES')
