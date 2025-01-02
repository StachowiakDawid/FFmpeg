/*
 * Copyright (c) 2003 Rich Felker
 * Copyright (c) 2012 Stefano Sabatini
 * Copyright (c) 2024 Dawid Stachowiak
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with FFmpeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/**
 * @file reverse_mpdecimate filter, based on vf_mpdecimate.c
 */

#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/pixelutils.h"
#include "libavutil/timestamp.h"
#include "avfilter.h"
#include "filters.h"
#include "video.h"

typedef struct DecimateContext {
    const AVClass *class;
    int lo, hi;                    ///< lower and higher threshold number of differences
                                   ///< values for 8x8 blocks

    float frac;                    ///< threshold of changed pixels over the total fraction

    int min_dup_count;             ///< minimum number of previous frames that need to be duplicated to keep frame

    int dup_count;                 ///< number of duplicated frames

    int hsub, vsub;                ///< chroma subsampling values
    AVFrame *ref;                  ///< reference picture
    av_pixelutils_sad_fn sad;      ///< sum of absolute difference function
} DecimateContext;

#define OFFSET(x) offsetof(DecimateContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption reverse_mpdecimate_options[] = {
    { "min",  "set minimum number of previous frames that need to be duplicated to keep current frame",
      OFFSET(min_dup_count), AV_OPT_TYPE_INT, {.i64=10}, 0, INT_MAX, FLAGS },
    { "hi",   "set high dropping threshold", OFFSET(hi), AV_OPT_TYPE_INT, {.i64=64*12}, INT_MIN, INT_MAX, FLAGS },
    { "lo",   "set low dropping threshold", OFFSET(lo), AV_OPT_TYPE_INT, {.i64=64*5}, INT_MIN, INT_MAX, FLAGS },
    { "frac", "set fraction dropping threshold",  OFFSET(frac), AV_OPT_TYPE_FLOAT, {.dbl=0.33}, 0, 1, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(reverse_mpdecimate);

/**
 * Return 1 if the two planes are different, 0 otherwise.
 */
static int diff_planes(AVFilterContext *ctx,
                       uint8_t *cur, int cur_linesize,
                       uint8_t *ref, int ref_linesize,
                       int w, int h)
{
    DecimateContext *decimate = ctx->priv;

    int x, y;
    int d, c = 0;
    int t = (w/16)*(h/16)*decimate->frac;

    /* compute difference for blocks of 8x8 bytes */
    for (y = 0; y < h-7; y += 4) {
        for (x = 8; x < w-7; x += 4) {
            d = decimate->sad(cur + y*cur_linesize + x, cur_linesize,
                              ref + y*ref_linesize + x, ref_linesize);
            if (d > decimate->hi) {
                av_log(ctx, AV_LOG_DEBUG, "%d>=hi ", d);
                return 1;
            }
            if (d > decimate->lo) {
                c++;
                if (c > t) {
                    av_log(ctx, AV_LOG_DEBUG, "lo:%d>=%d ", c, t);
                    return 1;
                }
            }
        }
    }

    av_log(ctx, AV_LOG_DEBUG, "lo:%d<%d ", c, t);
    return 0;
}

/**
 * Tell if the frame is different with respect to the reference frame ref.
 */
static int is_frame_different(AVFilterContext *ctx,
                          AVFrame *cur, AVFrame *ref)
{
    DecimateContext *decimate = ctx->priv;
    int plane;

    for (plane = 0; ref->data[plane] && ref->linesize[plane]; plane++) {
        /* use 8x8 SAD even on subsampled planes.  The blocks won't match up with
         * luma blocks, but hopefully nobody is depending on this to catch
         * localized chroma changes that wouldn't exceed the thresholds when
         * diluted by using what's effectively a larger block size.
         */
        int vsub = plane == 1 || plane == 2 ? decimate->vsub : 0;
        int hsub = plane == 1 || plane == 2 ? decimate->hsub : 0;
        if (diff_planes(ctx,
                        cur->data[plane], cur->linesize[plane],
                        ref->data[plane], ref->linesize[plane],
                        AV_CEIL_RSHIFT(ref->width,  hsub),
                        AV_CEIL_RSHIFT(ref->height, vsub)))
            return 1;
    }

    return 0;
}

static av_cold int init(AVFilterContext *ctx)
{
    DecimateContext *decimate = ctx->priv;

    decimate->sad = av_pixelutils_get_sad_fn(3, 3, 0, ctx); // 8x8, not aligned on blocksize
    if (!decimate->sad)
        return AVERROR(EINVAL);

    av_log(ctx, AV_LOG_VERBOSE, "min_dup_count:%d hi:%d lo:%d frac:%f\n",
           decimate->min_dup_count, decimate->hi, decimate->lo, decimate->frac);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    DecimateContext *decimate = ctx->priv;
    av_frame_free(&decimate->ref);
}

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_YUV444P,      AV_PIX_FMT_YUV422P,
    AV_PIX_FMT_YUV420P,      AV_PIX_FMT_YUV411P,
    AV_PIX_FMT_YUV410P,      AV_PIX_FMT_YUV440P,
    AV_PIX_FMT_YUVJ444P,     AV_PIX_FMT_YUVJ422P,
    AV_PIX_FMT_YUVJ420P,     AV_PIX_FMT_YUVJ440P,
    AV_PIX_FMT_YUVA420P,

    AV_PIX_FMT_GBRP,

    AV_PIX_FMT_YUVA444P,
    AV_PIX_FMT_YUVA422P,

    AV_PIX_FMT_NONE
};

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    DecimateContext *decimate = ctx->priv;
    const AVPixFmtDescriptor *pix_desc = av_pix_fmt_desc_get(inlink->format);
    decimate->hsub = pix_desc->log2_chroma_w;
    decimate->vsub = pix_desc->log2_chroma_h;

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *cur)
{
    DecimateContext *decimate = inlink->dst->priv;
    AVFilterLink *outlink = inlink->dst->outputs[0];
    int ret;

    if (decimate->ref && is_frame_different(inlink->dst, cur, decimate->ref)) {
        decimate->dup_count = 0;
    } else {
        decimate->dup_count++;
    }

    av_log(inlink->dst, AV_LOG_DEBUG,
           "%s pts:%s pts_time:%s dup_count:%d \n",
           decimate->dup_count == decimate->min_dup_count ? "keep" : "drop",
           av_ts2str(cur->pts), av_ts2timestr(cur->pts, &inlink->time_base),
           decimate->dup_count);

    if (decimate->dup_count == decimate->min_dup_count && (ret = ff_filter_frame(outlink, av_frame_clone(cur))) < 0)
            return ret;

    av_frame_free(&decimate->ref);
    decimate->ref = av_frame_clone(cur);
    av_frame_free(&cur);

    return 0;
}

static const AVFilterPad reverse_mpdecimate_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_input,
        .filter_frame = filter_frame,
    },
};

const FFFilter ff_vf_reverse_mpdecimate = {
    .p.name          = "reverse_mpdecimate",
    .p.description   = NULL_IF_CONFIG_SMALL("Remove non-duplicate frames."),
    .p.priv_class  = &reverse_mpdecimate_class,
    .init          = init,
    .uninit        = uninit,
    .priv_size     = sizeof(DecimateContext),
    FILTER_INPUTS(reverse_mpdecimate_inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
    FILTER_PIXFMTS_ARRAY(pix_fmts),
};
