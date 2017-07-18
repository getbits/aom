/*
 * Copyright (c) 2016, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 2 Clause License and
 * the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
 * was not distributed with this source code in the LICENSE file, you can
 * obtain it at www.aomedia.org/license/software. If the Alliance for Open
 * Media Patent License 1.0 was not distributed with this source code in the
 * PATENTS file, you can obtain it at www.aomedia.org/license/patent.
 */

#include <float.h>
#include <limits.h>
#include <math.h>

#include "./av1_rtcd.h"
#include "./aom_dsp_rtcd.h"

#include "av1/encoder/encoder.h"
#include "aom_dsp/aom_dsp_common.h"
#include "av1/encoder/segmentation.h"
#include "av1/encoder/encodeframe.h"
#include "av1/encoder/mbgraph.h"
#include "av1/common/reconintra.h"
#include "av1/common/reconinter.h"
#include "av1/encoder/aq_mbtree.h"
#include "aom_ports/system_state.h"

static int find_best_16x16_intra(AV1_COMP *cpi)
{
  MACROBLOCK *const x = &cpi->td.mb;
  MACROBLOCKD *const xd = &x->e_mbd;
  PREDICTION_MODE mode;
  unsigned int best_err = INT_MAX;

  // calculate SATD for each intra prediction mode;
  // we're intentionally not doing 4x4, we just want a rough estimate
  for (mode = DC_PRED; mode <= TM_PRED; mode++) {
    unsigned int err;

    xd->mi[0]->mbmi.mode = mode;
    av1_predict_intra_block(xd, 16, 16, BLOCK_16X16, mode, x->plane[0].src.buf,
                            x->plane[0].src.stride, xd->plane[0].dst.buf,
                            xd->plane[0].dst.stride, 0, 0, 0);
    err = aom_sad16x16(x->plane[0].src.buf, x->plane[0].src.stride,
                       xd->plane[0].dst.buf, xd->plane[0].dst.stride);

    // find best
    if (err < best_err)
      best_err = err;
  }

  return best_err;
}

static unsigned int do_16x16_motion_iteration(AV1_COMP *cpi, const MV *ref_mv,
                                              int mb_row, int mb_col) {
  MACROBLOCK *const x = &cpi->td.mb;
  MACROBLOCKD *const xd = &x->e_mbd;
  const MV_SPEED_FEATURES *const mv_sf = &cpi->sf.mv;
  const aom_variance_fn_ptr_t v_fn_ptr = cpi->fn_ptr[BLOCK_16X16];

  const MvLimits tmp_mv_limits = x->mv_limits;
  MV ref_full;
  int cost_list[5];

  // Further step/diamond searches as necessary
  int step_param = mv_sf->reduce_first_step_size;
  step_param = AOMMIN(step_param, MAX_MVSEARCH_STEPS - 2);

  av1_set_mv_search_range(&x->mv_limits, ref_mv);

  ref_full.col = ref_mv->col >> 3;
  ref_full.row = ref_mv->row >> 3;

  /*cpi->sf.search_method == HEX*/
  av1_hex_search(x, &ref_full, step_param, x->errorperbit, 0,
                 cond_cost_list(cpi, cost_list), &v_fn_ptr, 0, ref_mv);

  // Try sub-pixel MC
  // if (bestsme > error_thresh && bestsme < INT_MAX)
  {
    int distortion;
    unsigned int sse;
    cpi->find_fractional_mv_step(
        x, ref_mv, cpi->common.allow_high_precision_mv, x->errorperbit,
        &v_fn_ptr, 0, mv_sf->subpel_iters_per_step,
        cond_cost_list(cpi, cost_list), NULL, NULL, &distortion, &sse, NULL,
#if CONFIG_EXT_INTER
        NULL, 0, 0,
#endif
        0, 0, 0);
  }

#if CONFIG_EXT_INTER
  if (has_second_ref(&xd->mi[0]->mbmi))
    xd->mi[0]->mbmi.mode = NEW_NEWMV;
  else
#endif  // CONFIG_EXT_INTER
    xd->mi[0]->mbmi.mode = NEWMV;

  xd->mi[0]->mbmi.mv[0] = x->best_mv;
#if CONFIG_EXT_INTER
  xd->mi[0]->mbmi.ref_frame[1] = NONE_FRAME;
#endif  // CONFIG_EXT_INTER

  av1_build_inter_predictors_sby(&cpi->common, xd, mb_row, mb_col, NULL,
                                 BLOCK_16X16);

  /* restore UMV window */
  x->mv_limits = tmp_mv_limits;

  return aom_sad16x16(x->plane[0].src.buf, x->plane[0].src.stride,
                      xd->plane[0].dst.buf, xd->plane[0].dst.stride);
}

static int do_16x16_motion_search(AV1_COMP *cpi, const MV *ref_mv, int mb_row,
                                  int mb_col) {
  MACROBLOCK *const x = &cpi->td.mb;
  MACROBLOCKD *const xd = &x->e_mbd;
  unsigned int err, tmp_err;
  MV best_mv;

  // Try zero MV first
  // FIXME should really use something like near/nearest MV and/or MV prediction
  err = aom_sad16x16(x->plane[0].src.buf, x->plane[0].src.stride,
                     xd->plane[0].pre[0].buf, xd->plane[0].pre[0].stride);
  best_mv.col = best_mv.row = 0;

  // Test last reference frame using the previous best mv as the
  // starting point (best reference) for the search
  tmp_err = do_16x16_motion_iteration(cpi, ref_mv, mb_row, mb_col);
  if (tmp_err < err) {
    err = tmp_err;
    best_mv = x->best_mv.as_mv;
  }

  // If the current best reference mv is not centered on 0,0 then do a 0,0
  // based search as well.
  if (ref_mv->row != 0 || ref_mv->col != 0) {
    MV zero_ref_mv = { 0, 0 };

    tmp_err = do_16x16_motion_iteration(cpi, &zero_ref_mv, mb_row, mb_col);
    if (tmp_err < err) {
      err = tmp_err;
      best_mv = x->best_mv.as_mv;
    }
  }

  x->best_mv.as_mv = best_mv;
  return err;
}

static MBTreeEntry *get_mb(MBTreeContext *mbctx, int x, int y)
{
  int x64 = x >> 2;
  int y64 = y >> 2;
  MBTreeEntry *lvl0 = &mbctx->tree[y64 * mbctx->tree_width + x64];
  x64 = (x - (x64 << 2));
  y64 = (y - (y64 << 2));
  int x32 = x64 >> 1;
  int y32 = x64 >> 1;
  MBTreeEntry *lvl1 = &lvl0->subdiv[y32*2 + x32];
  int x16 = x32 >> 1;
  int y16 = y32 >> 1;
  return &lvl1->subdiv[y16*2 + x16];
}

static void process_frame(AV1_COMP *cpi, int index)
{
  int row, col;
  AV1_COMMON *const cm = &cpi->common;
  MBTreeContext *mbctx = &cpi->mbtree;
  struct lookahead_entry *q_0;
  MACROBLOCK *const x = &cpi->td.mb;
  MACROBLOCKD *const xd = &x->e_mbd;

  if (!xd->cur_buf)
    return;

  q_0 = av1_lookahead_peek(cpi->lookahead, index);

  MODE_INFO mi_local;
  av1_zero(mi_local);

  xd->mi[0] = &mi_local;
  mi_local.mbmi.sb_type = BLOCK_16X16;
  mi_local.mbmi.ref_frame[0] = LAST_FRAME;
  mi_local.mbmi.ref_frame[1] = NONE_FRAME;

  YV12_BUFFER_CONFIG *buf0 = &q_0->img;
  YV12_BUFFER_CONFIG tsa = *buf0;
  tsa.y_buffer = get_frame_new_buffer(cm)->y_buffer;
  tsa.y_stride = get_frame_new_buffer(cm)->y_stride;
  YV12_BUFFER_CONFIG *buf1 = &tsa;

  if (!buf0 || !buf1)
    return;

  struct scale_factors *sf = (struct scale_factors *)&xd->block_refs[1]->sf;

#if CONFIG_AOM_HIGHBITDEPTH
    av1_setup_scale_factors_for_frame(
        sf, buf1->y_crop_width, buf1->y_crop_height,
        buf1->y_crop_width, buf1->y_crop_height,
        cpi->common.use_highbitdepth);
#else
    av1_setup_scale_factors_for_frame(
        sf, buf1->y_crop_width, buf1->y_crop_height,
        buf1->y_crop_width, buf1->y_crop_height, cpi->common.use_highbitdepth);
#endif  // CONFIG_AOM_HIGHBITDEPTH

  x->nmvjointcost = x->nmv_vec_cost[0];
 // x->nmvjointsadcost = x->nmvjointcost;
  x->plane[0].src.stride = buf0->y_stride;
  xd->plane[0].dst.stride = buf1->y_stride;
  xd->plane[1].dst.stride = buf1->uv_stride;
  xd->plane[0].pre[0].stride = buf1->y_stride;

  MV gld_top_mv = { 0, 0 };

  uint8_t *buf0_start = buf0->y_buffer;
  uint8_t *buf1_start = buf1->y_buffer;

  int basis_rows = mbctx->tree_width  << 2;
  int basis_cols = mbctx->tree_height << 2;

  for (row = 0; row < basis_rows; row++) {
    for (col = 0; col < basis_cols; col++) {
      MBTreeEntry *mb_stats = get_mb(mbctx, row, col);

      x->plane[0].src.buf = buf0_start + col*16;
      xd->plane[0].dst.buf = buf1_start + col*16;
      xd->plane[0].pre[0].buf = xd->plane[0].dst.buf;

      int ierr = find_best_16x16_intra(cpi);
      mb_stats->last_intra = (float)ierr;

      int perr = do_16x16_motion_search(cpi, &gld_top_mv, row, col);
      if (perr > ierr)
        perr = ierr;

      float prop_fraction = 1.0f - (ierr/((float)perr + FLT_MIN));
      float prop_amount = (ierr + mb_stats->prop_cost)*prop_fraction;
      mb_stats->prop_cost += prop_amount;
    }
    buf0_start += buf0->y_stride*16;
    buf1_start += buf1->y_stride*16;
  }
}

static void alloc_mb_tree(MBTreeEntry *mbt, int level, int limit)
{
  if (level >= limit)
    return;
  mbt->subdiv = aom_calloc(4, sizeof(MBTreeEntry));
  for (int i = 0; i < 4; i++)
    alloc_mb_tree(&mbt->subdiv[i], level + 1, limit);
}

static void free_mb_tree(MBTreeEntry *mbt)
{
  if (!mbt)
    return;
  for (int i = 0; i < 4; i++)
    free_mb_tree(&mbt->subdiv[i]);
  aom_free(mbt->subdiv);
}

static void alloc_root_tree(struct AV1_COMP *cpi)
{
  MBTreeContext *mbctx = &cpi->mbtree;
  AV1_COMMON *const cm = &cpi->common;

  int w = AV1ALIGN(cm->width, 64)  / 64;
  int h = AV1ALIGN(cm->height, 64) / 64;

  if ((w != mbctx->tree_width) || (h != mbctx->tree_height)) {
    aom_free(mbctx->scratch_buf);
    if (mbctx->tree)
      for (int i = 0; i < (mbctx->tree_width*mbctx->tree_height); i++)
        free_mb_tree(&mbctx->tree[i]);
    mbctx->scratch_buf = aom_malloc(cm->render_width*cm->render_height*8);
    mbctx->tree = aom_calloc(w*h, sizeof(MBTreeEntry));
    for (int i = 0; i < w*h; i++)
      alloc_mb_tree(&mbctx->tree[i], 0, 2); /* 2 for 16x16 */
    mbctx->tree_width = w;
    mbctx->tree_height = h;
  }
}

static void zero_mb_tree(MBTreeEntry *mbt, int level, int limit)
{
  if (level >= limit)
    return;
  mbt->prop_cost = 0;
  mbt->last_intra = 0;
  for (int i = 0; i < 4; i++)
    zero_mb_tree(&mbt->subdiv[i], level + 1, limit);
}

void av1_mbtree_update(struct AV1_COMP *cpi)
{
  MBTreeContext *mbctx = &cpi->mbtree;

  alloc_root_tree(cpi);

  for (int i = 0; i < (mbctx->tree_width*mbctx->tree_height); i++)
    zero_mb_tree(&mbctx->tree[i], 0, 2);

  int last_lookahead = av1_lookahead_depth(cpi->lookahead);
  for (int i = last_lookahead - 1; i >= 0; i--)
    process_frame(cpi, i);
}

#define AQ_C_SEGMENTS 5

void av1_mbtree_frame_setup(struct AV1_COMP *cpi)
{
  AV1_COMMON *const cm = &cpi->common;
  struct segmentation *const seg = &cm->seg;

  int segment;

  // Clear down the segment map.
  memset(cpi->segmentation_map, 0, cm->mi_rows * cm->mi_cols);

  av1_clearall_segfeatures(seg);

  // Segmentation only makes sense if the target bits per SB is above a
  // threshold. Below this the overheads will usually outweigh any benefit.
  if (cpi->rc.sb64_target_rate < 256) {
    av1_disable_segmentation(seg);
    return;
  }

  av1_enable_segmentation(seg);

  // Select delta coding method.
  seg->abs_delta = SEGMENT_DELTADATA;

  // Default segment "Q" feature is disabled so it defaults to the baseline Q.
  av1_disable_segfeature(seg, 0, SEG_LVL_ALT_Q);

  // Use some of the segments for in frame Q adjustment.
  for (segment = 0; segment < AQ_C_SEGMENTS; ++segment) {
    int qindex_delta = segment;

    // For AQ complexity mode, we dont allow Q0 in a segment if the base
    // Q is not 0. Q0 (lossless) implies 4x4 only and in AQ mode 2 a segment
    // Q delta is sometimes applied without going back around the rd loop.
    // This could lead to an illegal combination of partition size and q.
    if ((cm->base_qindex != 0) && ((cm->base_qindex + qindex_delta) == 0)) {
      qindex_delta = -cm->base_qindex + 1;
    }

    if (qindex_delta && ((cm->base_qindex + qindex_delta) > 0)) {
      av1_enable_segfeature(seg, segment, SEG_LVL_ALT_Q);
      av1_set_segdata(seg, segment, SEG_LVL_ALT_Q, qindex_delta);
    }
  }
}

void av1_mbtree_select_segment(struct AV1_COMP *cpi, MACROBLOCK *mb, BLOCK_SIZE bs,
                               int mi_row, int mi_col)
{
  MBTreeContext *mbctx = &cpi->mbtree;
  AV1_COMMON *const cm = &cpi->common;
  int x, y;
  const int mi_offset = mi_row * cm->mi_cols + mi_col;
  const int xmis = AOMMIN(cm->mi_cols - mi_col, mi_size_wide[bs]);
  const int ymis = AOMMIN(cm->mi_rows - mi_row, mi_size_high[bs]);
  uint8_t seg = 0;

  if (bs == BLOCK_16X16) {
    int px_x = (mi_col * 4)/16;
    int px_y = (mi_row * 4)/16;
    if (mbctx->tree) {
      MBTreeEntry *mb_stats = get_mb(mbctx, px_x, px_y);
      float prop_cost = mb_stats->prop_cost;
      float last_intra = mb_stats->last_intra;
      float qdif = -log2f((last_intra + prop_cost + 1)/(prop_cost + 1));
      if (!isnan(qdif))
        seg = (int)lrintf(qdif);
    }
  } else if (bs == BLOCK_32X32) {
    int off;
    int segs[4] = { 0, 0, 0, 0 };
    for (off = 0; off < 4; off++) {
      int px_x = ((mi_col +  (off % 2)) * 4)/16;
      int px_y = ((mi_row + !(off % 2)) * 4)/16;
      if (mbctx->tree) {
        MBTreeEntry *mb_stats = get_mb(mbctx, px_x, px_y);
        float prop_cost = mb_stats->prop_cost;
        float last_intra = mb_stats->last_intra;
        float qdif = -log2f((last_intra + prop_cost + 1)/(prop_cost + 1));
        if (!isnan(qdif))
          segs[off] = (int)lrintf(qdif);
      }
    }
    float avg = (segs[0] + segs[1] + segs[2] + segs[3])/4.0f;
    seg = (int)lrintf(avg);
  }

  seg += (AQ_C_SEGMENTS >> 1);
  if (seg < 0)
   seg = 0;
  if (seg >= AQ_C_SEGMENTS)
   seg = AQ_C_SEGMENTS - 1;

  // Fill in the entires in the segment map corresponding to this SB64.
  for (y = 0; y < ymis; y++) {
    for (x = 0; x < xmis; x++) {
      cpi->segmentation_map[mi_offset + y * cm->mi_cols + x] = seg;
    }
  }
}

void av1_mbtree_uninit(struct AV1_COMP *cpi)
{
    MBTreeContext *mbctx = &cpi->mbtree;
    aom_free(mbctx->scratch_buf);
    //for (int i = 0; i < (mbctx->tree_width*mbctx->tree_height); i++)
      //free_mb_tree(&mbctx->tree[i]);
}
