#if 0
#else
#include <stdbool.h>
#include "alsa/asoundlib.h"
#include "alsa/error.h"
//#include "avdevice.h"
//#include "libavutil/avassert.h"
//#include "libavutil/channel_layout.h"

#include "alsa.h"

#define DEBUG
#ifdef DEBUG
#undef assert
#define assert(p)\
    do{\
        if (p)\
        {\
            printf("%s %d assert() success\n", __func__, __LINE__);\
        }else{\
            printf("%s %d assert() failed\n", __func__, __LINE__);\
            abort();\
        }\
    }while(0)
#endif

extern DEMO_DBG_LEVEL_e demo_debug_level;
extern bool demo_func_trace;

typedef struct AlsaData {
    snd_pcm_t *h;
    snd_pcm_stream_t mode;
    snd_pcm_format_t format_id;
    unsigned int frame_size;  ///< bytes per sample * channels
    unsigned int period_size; ///< preferred size for reads and writes, in frames
    unsigned int sample_rate; ///< sample rate set by user
    unsigned int channels;    ///< number of channels set by user
    int last_period;
    void (*reorder_func)(const void *, void *, int);
    void *reorder_buf;
    int reorder_buf_size; ///< in frames
    int64_t timestamp; ///< current timestamp, without latency applied.
} AlsaData;

void print_alsaData(AlsaData *s);

static bool pcm_format_is_support(snd_pcm_format_t format_id)
{
    switch(format_id) {
        case SND_PCM_FORMAT_FLOAT64_LE:
        case SND_PCM_FORMAT_FLOAT64_BE:
        case SND_PCM_FORMAT_FLOAT_LE:
        case SND_PCM_FORMAT_FLOAT_BE:
        case SND_PCM_FORMAT_S32_LE:
        case SND_PCM_FORMAT_S32_BE:
        case SND_PCM_FORMAT_U32_LE:
        case SND_PCM_FORMAT_U32_BE:
        case SND_PCM_FORMAT_S24_3LE:
        case SND_PCM_FORMAT_S24_3BE:
        case SND_PCM_FORMAT_U24_3LE:
        case SND_PCM_FORMAT_U24_3BE:
        case SND_PCM_FORMAT_S16_LE:
        case SND_PCM_FORMAT_S16_BE:
        case SND_PCM_FORMAT_U16_LE:
        case SND_PCM_FORMAT_U16_BE:
        case SND_PCM_FORMAT_S8:
        case SND_PCM_FORMAT_U8:
        case SND_PCM_FORMAT_MU_LAW:
        case SND_PCM_FORMAT_A_LAW:
            return true;
        default: //SND_PCM_FORMAT_UNKNOWN;
            return false;
    }
}

static int get_bits_per_sample(snd_pcm_format_t format_id)
{
    switch(format_id) {
        case SND_PCM_FORMAT_FLOAT64_LE:
        case SND_PCM_FORMAT_FLOAT64_BE:
            return 64;
        case SND_PCM_FORMAT_FLOAT_LE:
        case SND_PCM_FORMAT_FLOAT_BE:
        case SND_PCM_FORMAT_S32_LE:
        case SND_PCM_FORMAT_S32_BE:
        case SND_PCM_FORMAT_U32_LE:
        case SND_PCM_FORMAT_U32_BE:
            return 32;
        case SND_PCM_FORMAT_S24_3LE:
        case SND_PCM_FORMAT_S24_3BE:
        case SND_PCM_FORMAT_U24_3LE:
        case SND_PCM_FORMAT_U24_3BE:
            return 24;
        case SND_PCM_FORMAT_S16_LE:
        case SND_PCM_FORMAT_S16_BE:
        case SND_PCM_FORMAT_U16_LE:
        case SND_PCM_FORMAT_U16_BE:
            return 16;
        case SND_PCM_FORMAT_S8:
        case SND_PCM_FORMAT_U8:
        case SND_PCM_FORMAT_MU_LAW:
        case SND_PCM_FORMAT_A_LAW:
            return 8;
        default: //SND_PCM_FORMAT_UNKNOWN;
            return 0;
    }
}

#if 0
#define MAKE_REORDER_FUNC(NAME, TYPE, CHANNELS, LAYOUT, MAP)                \
static void alsa_reorder_ ## NAME ## _ ## LAYOUT(const void *in_v,          \
                                                 void *out_v,               \
                                                 int n)                     \
{                                                                           \
    const TYPE *in = in_v;                                                  \
    TYPE      *out = out_v;                                                 \
                                                                            \
    while (n-- > 0) {                                                       \
        MAP                                                                 \
        in  += CHANNELS;                                                    \
        out += CHANNELS;                                                    \
    }                                                                       \
}

#define MAKE_REORDER_FUNCS(CHANNELS, LAYOUT, MAP) \
    MAKE_REORDER_FUNC(int8,  int8_t,  CHANNELS, LAYOUT, MAP) \
    MAKE_REORDER_FUNC(int16, int16_t, CHANNELS, LAYOUT, MAP) \
    MAKE_REORDER_FUNC(int32, int32_t, CHANNELS, LAYOUT, MAP) \
    MAKE_REORDER_FUNC(f32,   float,   CHANNELS, LAYOUT, MAP)

MAKE_REORDER_FUNCS(5, out_50, \
        out[0] = in[0]; \
        out[1] = in[1]; \
        out[2] = in[3]; \
        out[3] = in[4]; \
        out[4] = in[2]; \
        )

MAKE_REORDER_FUNCS(6, out_51, \
        out[0] = in[0]; \
        out[1] = in[1]; \
        out[2] = in[4]; \
        out[3] = in[5]; \
        out[4] = in[2]; \
        out[5] = in[3]; \
        )

MAKE_REORDER_FUNCS(8, out_71, \
        out[0] = in[0]; \
        out[1] = in[1]; \
        out[2] = in[4]; \
        out[3] = in[5]; \
        out[4] = in[2]; \
        out[5] = in[3]; \
        out[6] = in[6]; \
        out[7] = in[7]; \
        )

#define FORMAT_I8  0
#define FORMAT_I16 1
#define FORMAT_I32 2
#define FORMAT_F32 3

#define PICK_REORDER(layout)\
switch(format) {\
    case FORMAT_I8:  s->reorder_func = alsa_reorder_int8_out_ ##layout;  break;\
    case FORMAT_I16: s->reorder_func = alsa_reorder_int16_out_ ##layout; break;\
    case FORMAT_I32: s->reorder_func = alsa_reorder_int32_out_ ##layout; break;\
    case FORMAT_F32: s->reorder_func = alsa_reorder_f32_out_ ##layout;   break;\
}
static av_cold int find_reorder_func(AlsaData *s, int format_id, uint64_t layout, int out)
{
    int format;

    /* reordering input is not currently supported */
    if (!out)
        return AVERROR(ENOSYS);

    /* reordering is not needed for QUAD or 2_2 layout */
    if (layout == AV_CH_LAYOUT_QUAD || layout == AV_CH_LAYOUT_2_2)
        return 0;

    switch (format_id) {
    case AV_CODEC_ID_PCM_S8:
    case AV_CODEC_ID_PCM_U8:
    case AV_CODEC_ID_PCM_ALAW:
    case AV_CODEC_ID_PCM_MULAW: format = FORMAT_I8;  break;
    case AV_CODEC_ID_PCM_S16LE:
    case AV_CODEC_ID_PCM_S16BE:
    case AV_CODEC_ID_PCM_U16LE:
    case AV_CODEC_ID_PCM_U16BE: format = FORMAT_I16; break;
    case AV_CODEC_ID_PCM_S32LE:
    case AV_CODEC_ID_PCM_S32BE:
    case AV_CODEC_ID_PCM_U32LE:
    case AV_CODEC_ID_PCM_U32BE: format = FORMAT_I32; break;
    case AV_CODEC_ID_PCM_F32LE:
    case AV_CODEC_ID_PCM_F32BE: format = FORMAT_F32; break;
    default:                 return AVERROR(ENOSYS);
    }

    if      (layout == AV_CH_LAYOUT_5POINT0_BACK || layout == AV_CH_LAYOUT_5POINT0)
        PICK_REORDER(50)
    else if (layout == AV_CH_LAYOUT_5POINT1_BACK || layout == AV_CH_LAYOUT_5POINT1)
        PICK_REORDER(51)
    else if (layout == AV_CH_LAYOUT_7POINT1)
        PICK_REORDER(71)

    return s->reorder_func ? 0 : AVERROR(ENOSYS);
}
#endif

#if 1
int ff_alsa_open(DeviceContex_t *ctx, snd_pcm_stream_t mode,
                         unsigned int *sample_rate,
                         int channels, snd_pcm_format_t *format_id)
{
    AlsaData *s = ctx->priv_data;
    const char *audio_device;
    int ret;
    int flags = O_RDWR | O_NONBLOCK;
    snd_pcm_format_t format;
    snd_pcm_t *h;
    snd_pcm_hw_params_t *hw_params;
    snd_pcm_uframes_t buffer_size, period_size;
//    uint64_t layout = ctx->streams[0]->codecpar->channel_layout;

    if (ctx->url[0] == 0)
        //audio_device = "default";
        audio_device = "hw:0,0";
    else
        audio_device = ctx->url;

    if(!pcm_format_is_support(*format_id))
        return -EINVAL;

    format = *format_id;
    s->frame_size = get_bits_per_sample(format) / 8 * channels;

    ret = snd_pcm_open(&h, audio_device, mode, flags);
    if (ret < 0) {
        DEMO_ERR("cannot open audio device %s (%s)\n",
               audio_device, snd_strerror(ret));
        return -EIO;
    }

    ret = snd_pcm_hw_params_malloc(&hw_params);
    if (ret < 0) {
        DEMO_ERR("cannot allocate hardware parameter structure (%s)\n",
               snd_strerror(ret));
        goto fail1;
    }

    ret = snd_pcm_hw_params_any(h, hw_params);
    if (ret < 0) {
        DEMO_ERR("cannot initialize hardware parameter structure (%s)\n",
               snd_strerror(ret));
        goto fail;
    }

    ret = snd_pcm_hw_params_set_access(h, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    if (ret < 0) {
        DEMO_ERR("cannot set access type (%s)\n",
               snd_strerror(ret));
        goto fail;
    }

    ret = snd_pcm_hw_params_set_format(h, hw_params, format);
    if (ret < 0) {
        DEMO_ERR("cannot set sample format 0x%04x %d (%s)\n",
               *format_id, format, snd_strerror(ret));
        goto fail;
    }

    ret = snd_pcm_hw_params_set_rate_near(h, hw_params, sample_rate, 0);
    if (ret < 0) {
        DEMO_ERR("cannot set sample rate (%s)\n",
               snd_strerror(ret));
        goto fail;
    }

    ret = snd_pcm_hw_params_set_channels(h, hw_params, channels);
    if (ret < 0) {
        DEMO_ERR("cannot set channel count to %d (%s)\n",
               channels, snd_strerror(ret));
        goto fail;
    }

    snd_pcm_hw_params_get_buffer_size_max(hw_params, &buffer_size);
    printf("%s %d buffer_size%lu\n", __func__, __LINE__, buffer_size);
    buffer_size = MIN(buffer_size, ALSA_BUFFER_SIZE_MAX);
buffer_size = 2048;
    /* TODO: maybe use ctx->max_picture_buffer somehow */
    ret = snd_pcm_hw_params_set_buffer_size_near(h, hw_params, &buffer_size);
    if (ret < 0) {
        DEMO_ERR("cannot set ALSA buffer size (%s)\n",
               snd_strerror(ret));
        goto fail;
    }

    snd_pcm_hw_params_get_period_size_min(hw_params, &period_size, NULL);
    printf("%s %d period_size%lu\n", __func__, __LINE__, period_size);
    if (!period_size)
        period_size = buffer_size / 4;
period_size = 1024;
    ret = snd_pcm_hw_params_set_period_size_near(h, hw_params, &period_size, NULL);
    if (ret < 0) {
        DEMO_ERR("cannot set ALSA period size (%s)\n",
               snd_strerror(ret));
        goto fail;
    }
    s->period_size = period_size;

    ret = snd_pcm_hw_params(h, hw_params);
    if (ret < 0) {
        DEMO_ERR("cannot set parameters (%s)\n",
               snd_strerror(ret));
        goto fail;
    }

    snd_pcm_hw_params_free(hw_params);
#if 0
    if (channels > 2 && layout) {
        if (find_reorder_func(s, *format_id, layout, mode == SND_PCM_STREAM_PLAYBACK) < 0) {
            char name[128];
            av_get_channel_layout_string(name, sizeof(name), channels, layout);
            av_log(ctx, AV_LOG_WARNING, "ALSA channel layout unknown or unimplemented for %s %s.\n",
                   name, mode == SND_PCM_STREAM_PLAYBACK ? "playback" : "capture");
        }
        if (s->reorder_func) {
            s->reorder_buf_size = buffer_size;
            s->reorder_buf = av_malloc_array(s->reorder_buf_size, s->frame_size);
            if (!s->reorder_buf)
                goto fail1;
        }
    }
#endif
    s->h = h;
    print_alsaData(s);
    return 0;

fail:
    snd_pcm_hw_params_free(hw_params);
fail1:
    snd_pcm_close(h);
    return -EIO;
}

int ff_alsa_close(DeviceContex_t *ctx)
{
    AlsaData *s = ctx->priv_data;

    snd_pcm_nonblock(s->h, 0);
    snd_pcm_drain(s->h);
#if 0
    av_freep(&s->reorder_buf);
    if (CONFIG_ALSA_INDEV)
        ff_timefilter_destroy(s->timefilter);
#endif
    snd_pcm_close(s->h);
    return 0;
}

int ff_alsa_xrun_recover(DeviceContex_t *ctx, int err)
{
    AlsaData *s = ctx->priv_data;
    snd_pcm_t *handle = s->h;

    DEMO_WRN("ALSA buffer xrun.\n");
    if (err == -EPIPE) {
        err = snd_pcm_prepare(handle);
        if (err < 0) {
            DEMO_ERR("cannot recover from underrun (snd_pcm_prepare failed: %s)\n",
                            snd_strerror(err));

            return -EIO;
        }
    } else if (err == -ESTRPIPE) {
        DEMO_ERR("-ESTRPIPE... Unsupported!\n");

        return -1;
    }
    return err;
}
#endif
#if 0

int ff_alsa_extend_reorder_buf(AlsaData *s, int min_size)
{
    int size = s->reorder_buf_size;
    void *r;

    av_assert0(size != 0);
    while (size < min_size)
        size *= 2;
    r = av_realloc_array(s->reorder_buf, size, s->frame_size);
    if (!r)
        return AVERROR(ENOMEM);
    s->reorder_buf = r;
    s->reorder_buf_size = size;
    return 0;
}
#endif

#include <alsa/asoundlib.h>
#if 0
#include "libavutil/internal.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"

#include "libavformat/internal.h"

#include "avdevice.h"
#endif
#include "alsa.h"

static int audio_read_header(DeviceContex_t *ctx)
{
    AlsaData *s = ctx->priv_data;
    int ret;
    ret = ff_alsa_open(ctx, SND_PCM_STREAM_CAPTURE, &s->sample_rate, s->channels,
        &s->format_id);
    if (ret < 0) {
        return -EIO;
    }

    return 0;
}

static int audio_read_packet(DeviceContex_t *ctx, Packet *pkt)
{
    AlsaData *s  = ctx->priv_data;
    int ret;
    int64_t dts;
    snd_pcm_sframes_t delay = 0;

    assert(ctx);
    assert(s);
    if (new_packet(pkt, s->period_size * s->frame_size) < 0) {
        return -EIO;
    }
    while ((ret = snd_pcm_readi(s->h, pkt->data, s->period_size)) < 0) {
         DEMO_ERR("ALSA read error: %s\n", snd_strerror(ret));
        if (ret == -EAGAIN) {
            free_packet(pkt);

            return -EAGAIN;
        }
        if (ff_alsa_xrun_recover(ctx, ret) < 0) {
            DEMO_ERR("ALSA read error: %s\n",
                   snd_strerror(ret));
            free_packet(pkt);

            return -EIO;
        }
    }

    dts = gettime();
    snd_pcm_delay(s->h, &delay);
    dts -= delay;
    pkt->pts = dts;
    s->last_period = ret;

    pkt->size = ret * s->frame_size;

    return 0;
}
#endif//decode
#if 0 //encode
#include <alsa/asoundlib.h>

#include "libavutil/internal.h"
#include "libavutil/time.h"


#include "libavformat/internal.h"
#include "avdevice.h"
#include "alsa.h"

static int audio_write_header(DeviceContex_t *ctx)
{
    AlsaData *s = ctx->priv_data;
    AVStream *st = NULL;
    unsigned int sample_rate;
    snd_pcm_format_t format_id;
    int ret;

    if (ctx->nb_streams != 1 || ctx->streams[0]->codecpar->codec_type != AVMEDIA_TYPE_AUDIO) {
        av_log(ctx, AV_LOG_ERROR, "Only a single audio stream is supported.\n");
        return AVERROR(EINVAL);
    }
    st = ctx->streams[0];

    sample_rate = st->codecpar->sample_rate;
    format_id    = st->codecpar->format_id;
    ret = ff_alsa_open(ctx, SND_PCM_STREAM_PLAYBACK, &sample_rate,
        st->codecpar->channels, &format_id);
    if (sample_rate != st->codecpar->sample_rate) {
        av_log(ctx, AV_LOG_ERROR,
               "sample rate %d not available, nearett is %d\n",
               st->codecpar->sample_rate, sample_rate);
        goto fail;
    }
    avpriv_set_pts_info(st, 64, 1, sample_rate);

    return ret;

fail:
    snd_pcm_close(s->h);
    return AVERROR(EIO);
}

static int audio_write_packet(DeviceContex_t *ctx, Packet *pkt)
{
    AlsaData *s = ctx->priv_data;
    int ret;
    int size     = pkt->size;
    uint8_t *buf = pkt->data;

    size /= s->frame_size;
    if (pkt->dts != AV_NOPTS_VALUE)
        s->timestamp = pkt->dts;
    s->timestamp += pkt->duration ? pkt->duration : size;

    if (s->reorder_func) {
        if (size > s->reorder_buf_size)
            if (ff_alsa_extend_reorder_buf(s, size))
                return AVERROR(ENOMEM);
        s->reorder_func(buf, s->reorder_buf, size);
        buf = s->reorder_buf;
    }
    while ((ret = snd_pcm_writei(s->h, buf, size)) < 0) {
        if (ret == -EAGAIN) {

            return AVERROR(EAGAIN);
        }

        if (ff_alsa_xrun_recover(ctx, ret) < 0) {
            av_log(ctx, AV_LOG_ERROR, "ALSA write error: %s\n",
                   snd_strerror(ret));

            return AVERROR(EIO);
        }
    }

    return 0;
}

static int audio_write_frame(DeviceContex_t *ctx, int stream_index,
                             AVFrame **frame, unsigned flags)
{
    AlsaData *s = ctx->priv_data;
    Packet pkt;

    /* ff_alsa_open() should have accepted only supported formats */
    if ((flags & AV_WRITE_UNCODED_FRAME_QUERY))
        return av_sample_fmt_is_planar(ctx->streams[stream_index]->codecpar->format) ?
               AVERROR(EINVAL) : 0;
    /* set only used fields */
    pkt.data     = (*frame)->data[0];
    pkt.size     = (*frame)->nb_samples * s->frame_size;
    pkt.dts      = (*frame)->pkt_dts;
    pkt.duration = (*frame)->pkt_duration;
    return audio_write_packet(ctx, &pkt);
}

static void
audio_get_output_timestamp(DeviceContex_t *ctx, int stream,
    int64_t *dts, int64_t *wall)
{
    AlsaData *s  = ctx->priv_data;
    snd_pcm_sframes_t delay = 0;
    *wall = av_gettime();
    snd_pcm_delay(s->h, &delay);
    *dts = s->timestamp - delay;
}

static int audio_get_device_list(DeviceContex_t *h, AVDeviceInfoList *device_list)
{
    return ff_alsa_get_device_list(device_list, SND_PCM_STREAM_PLAYBACK);
}

AVOutputFormat ff_alsa_muxer = {
    .name           = "alsa",
    .long_name      = NULL_IF_CONFIG_SMALL("ALSA audio output"),
    .priv_data_size = sizeof(AlsaData),
    .audio_codec    = DEFAULT_CODEC_ID,
    .video_codec    = AV_CODEC_ID_NONE,
    .write_header   = audio_write_header,
    .write_packet   = audio_write_packet,
    .write_trailer  = ff_alsa_close,
    .write_uncoded_frame = audio_write_frame,
    .get_device_list = audio_get_device_list,
    .get_output_timestamp = audio_get_output_timestamp,
    .flags          = AVFMT_NOFILE,
    .priv_class     = &alsa_muxer_class,
};
#endif//encode

int alsa_dev_init(DeviceContex_t **ctx)
{
    assert(ctx);

    DeviceContex_t *c = calloc(1, sizeof(DeviceContex_t));
    AlsaData *s = calloc(1, sizeof(AlsaData));

    assert(c);
    assert(s);

    c->priv_data = s;
    *ctx = c;
    return 0;
}

#if 1//decode
void alsa_dev_deinit(DeviceContex_t *ctx)
{
    DeviceContex_t *c = ctx;
    AlsaData *s = ctx->priv_data;

    assert(c);
    assert(s);

    free(s);
    free(c);
}

void audio_set_fmt(DeviceContex_t *ctx, snd_pcm_stream_t mode,
                         unsigned int sample_rate,
                         int channels, snd_pcm_format_t format_id)
{
    DeviceContex_t *c = ctx;
    AlsaData *s = ctx->priv_data;

    assert(c);
    assert(s);

    s->mode = mode;
    s->sample_rate = sample_rate;
    s->channels = channels;
    s->format_id = format_id;
    s->period_size = 1024;
}

void print_alsaData(AlsaData *s)
{
    printf("%11s  %8s  %9s  %10s  %11s\n", "sample_rate", "channels", "format_id", "frame_size", "period_size");
    printf("%11d  %8d  %9d  %10d  %11d\n", s->sample_rate, s->channels, s->format_id, s->frame_size, s->period_size);
}

int alsa_test()
{
    DeviceContex_t *ctx;
    Packet pkt;
    snd_pcm_stream_t mode = SND_PCM_STREAM_CAPTURE;
    unsigned int sample_rate =  16000;
    int channels = 1;
    snd_pcm_format_t format_id = SND_PCM_FORMAT_S16_LE;

    alsa_dev_init(&ctx);

    audio_set_fmt(ctx, mode, sample_rate, channels, format_id);
    audio_read_header(ctx);
    if(audio_read_packet(ctx, &pkt) >=0)
	    free_packet(&pkt);
    ff_alsa_close(ctx);

    alsa_dev_deinit(ctx);
    return 0;
}
#endif
