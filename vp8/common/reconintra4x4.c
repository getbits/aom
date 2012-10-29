/*
 *  Copyright (c) 2010 The WebM project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */


#include "vpx_ports/config.h"
#include "vpx_mem/vpx_mem.h"
#include "reconintra.h"
#include "vpx_rtcd.h"

void vp8_intra4x4_predict_c(BLOCKD *x, int b_mode,
                            unsigned char *predictor) {
  int i, r, c;

  unsigned char *Above = *(x->base_dst) + x->dst - x->dst_stride;
  unsigned char Left[4];
  unsigned char top_left = Above[-1];

  Left[0] = (*(x->base_dst))[x->dst - 1];
  Left[1] = (*(x->base_dst))[x->dst - 1 + x->dst_stride];
  Left[2] = (*(x->base_dst))[x->dst - 1 + 2 * x->dst_stride];
  Left[3] = (*(x->base_dst))[x->dst - 1 + 3 * x->dst_stride];

  switch (b_mode) {
    case B_DC_PRED: {
      int expected_dc = 0;

      for (i = 0; i < 4; i++) {
        expected_dc += Above[i];
        expected_dc += Left[i];
      }

      expected_dc = (expected_dc + 4) >> 3;

      for (r = 0; r < 4; r++) {
        for (c = 0; c < 4; c++) {
          predictor[c] = expected_dc;
        }

        predictor += 16;
      }
    }
    break;
    case B_TM_PRED: {
      /* prediction similar to true_motion prediction */
      for (r = 0; r < 4; r++) {
        for (c = 0; c < 4; c++) {
          int pred = Above[c] - top_left + Left[r];

          if (pred < 0)
            pred = 0;

          if (pred > 255)
            pred = 255;

          predictor[c] = pred;
        }

        predictor += 16;
      }
    }
    break;

    case B_VE_PRED: {

      unsigned int ap[4];
      ap[0] = Above[0];
      ap[1] = Above[1];
      ap[2] = Above[2];
      ap[3] = Above[3];

      for (r = 0; r < 4; r++) {
        for (c = 0; c < 4; c++) {

          predictor[c] = ap[c];
        }

        predictor += 16;
      }

    }
    break;


    case B_HE_PRED: {

      unsigned int lp[4];
      lp[0] = Left[0];
      lp[1] = Left[1];
      lp[2] = Left[2];
      lp[3] = Left[3];

      for (r = 0; r < 4; r++) {
        for (c = 0; c < 4; c++) {
          predictor[c] = lp[r];
        }

        predictor += 16;
      }
    }
    break;
    case B_LD_PRED: {
      unsigned char *ptr = Above;
      predictor[0 * 16 + 0] = (ptr[0] + ptr[1] * 2 + ptr[2] + 2) >> 2;
      predictor[0 * 16 + 1] =
        predictor[1 * 16 + 0] = (ptr[1] + ptr[2] * 2 + ptr[3] + 2) >> 2;
      predictor[0 * 16 + 2] =
        predictor[1 * 16 + 1] =
          predictor[2 * 16 + 0] = (ptr[2] + ptr[3] * 2 + ptr[4] + 2) >> 2;
      predictor[0 * 16 + 3] =
        predictor[1 * 16 + 2] =
          predictor[2 * 16 + 1] =
            predictor[3 * 16 + 0] = (ptr[3] + ptr[4] * 2 + ptr[5] + 2) >> 2;
      predictor[1 * 16 + 3] =
        predictor[2 * 16 + 2] =
          predictor[3 * 16 + 1] = (ptr[4] + ptr[5] * 2 + ptr[6] + 2) >> 2;
      predictor[2 * 16 + 3] =
        predictor[3 * 16 + 2] = (ptr[5] + ptr[6] * 2 + ptr[7] + 2) >> 2;
      predictor[3 * 16 + 3] = (ptr[6] + ptr[7] * 2 + ptr[7] + 2) >> 2;

    }
    break;
    case B_RD_PRED: {

      unsigned char pp[9];

      pp[0] = Left[3];
      pp[1] = Left[2];
      pp[2] = Left[1];
      pp[3] = Left[0];
      pp[4] = top_left;
      pp[5] = Above[0];
      pp[6] = Above[1];
      pp[7] = Above[2];
      pp[8] = Above[3];

      predictor[3 * 16 + 0] = (pp[0] + pp[1] * 2 + pp[2] + 2) >> 2;
      predictor[3 * 16 + 1] =
        predictor[2 * 16 + 0] = (pp[1] + pp[2] * 2 + pp[3] + 2) >> 2;
      predictor[3 * 16 + 2] =
        predictor[2 * 16 + 1] =
          predictor[1 * 16 + 0] = (pp[2] + pp[3] * 2 + pp[4] + 2) >> 2;
      predictor[3 * 16 + 3] =
        predictor[2 * 16 + 2] =
          predictor[1 * 16 + 1] =
            predictor[0 * 16 + 0] = (pp[3] + pp[4] * 2 + pp[5] + 2) >> 2;
      predictor[2 * 16 + 3] =
        predictor[1 * 16 + 2] =
          predictor[0 * 16 + 1] = (pp[4] + pp[5] * 2 + pp[6] + 2) >> 2;
      predictor[1 * 16 + 3] =
        predictor[0 * 16 + 2] = (pp[5] + pp[6] * 2 + pp[7] + 2) >> 2;
      predictor[0 * 16 + 3] = (pp[6] + pp[7] * 2 + pp[8] + 2) >> 2;

    }
    break;
    case B_VR_PRED: {

      unsigned char pp[9];

      pp[0] = Left[3];
      pp[1] = Left[2];
      pp[2] = Left[1];
      pp[3] = Left[0];
      pp[4] = top_left;
      pp[5] = Above[0];
      pp[6] = Above[1];
      pp[7] = Above[2];
      pp[8] = Above[3];


      predictor[3 * 16 + 0] = (pp[1] + pp[2] * 2 + pp[3] + 2) >> 2;
      predictor[2 * 16 + 0] = (pp[2] + pp[3] * 2 + pp[4] + 2) >> 2;
      predictor[3 * 16 + 1] =
        predictor[1 * 16 + 0] = (pp[3] + pp[4] * 2 + pp[5] + 2) >> 2;
      predictor[2 * 16 + 1] =
        predictor[0 * 16 + 0] = (pp[4] + pp[5] + 1) >> 1;
      predictor[3 * 16 + 2] =
        predictor[1 * 16 + 1] = (pp[4] + pp[5] * 2 + pp[6] + 2) >> 2;
      predictor[2 * 16 + 2] =
        predictor[0 * 16 + 1] = (pp[5] + pp[6] + 1) >> 1;
      predictor[3 * 16 + 3] =
        predictor[1 * 16 + 2] = (pp[5] + pp[6] * 2 + pp[7] + 2) >> 2;
      predictor[2 * 16 + 3] =
        predictor[0 * 16 + 2] = (pp[6] + pp[7] + 1) >> 1;
      predictor[1 * 16 + 3] = (pp[6] + pp[7] * 2 + pp[8] + 2) >> 2;
      predictor[0 * 16 + 3] = (pp[7] + pp[8] + 1) >> 1;

    }
    break;
    case B_VL_PRED: {

      unsigned char *pp = Above;

      predictor[0 * 16 + 0] = (pp[0] + pp[1] + 1) >> 1;
      predictor[1 * 16 + 0] = (pp[0] + pp[1] * 2 + pp[2] + 2) >> 2;
      predictor[2 * 16 + 0] =
        predictor[0 * 16 + 1] = (pp[1] + pp[2] + 1) >> 1;
      predictor[1 * 16 + 1] =
        predictor[3 * 16 + 0] = (pp[1] + pp[2] * 2 + pp[3] + 2) >> 2;
      predictor[2 * 16 + 1] =
        predictor[0 * 16 + 2] = (pp[2] + pp[3] + 1) >> 1;
      predictor[3 * 16 + 1] =
        predictor[1 * 16 + 2] = (pp[2] + pp[3] * 2 + pp[4] + 2) >> 2;
      predictor[0 * 16 + 3] =
        predictor[2 * 16 + 2] = (pp[3] + pp[4] + 1) >> 1;
      predictor[1 * 16 + 3] =
        predictor[3 * 16 + 2] = (pp[3] + pp[4] * 2 + pp[5] + 2) >> 2;
      predictor[2 * 16 + 3] = (pp[4] + pp[5] * 2 + pp[6] + 2) >> 2;
      predictor[3 * 16 + 3] = (pp[5] + pp[6] * 2 + pp[7] + 2) >> 2;
    }
    break;

    case B_HD_PRED: {
      unsigned char pp[9];
      pp[0] = Left[3];
      pp[1] = Left[2];
      pp[2] = Left[1];
      pp[3] = Left[0];
      pp[4] = top_left;
      pp[5] = Above[0];
      pp[6] = Above[1];
      pp[7] = Above[2];
      pp[8] = Above[3];


      predictor[3 * 16 + 0] = (pp[0] + pp[1] + 1) >> 1;
      predictor[3 * 16 + 1] = (pp[0] + pp[1] * 2 + pp[2] + 2) >> 2;
      predictor[2 * 16 + 0] =
        predictor[3 * 16 + 2] = (pp[1] + pp[2] + 1) >> 1;
      predictor[2 * 16 + 1] =
        predictor[3 * 16 + 3] = (pp[1] + pp[2] * 2 + pp[3] + 2) >> 2;
      predictor[2 * 16 + 2] =
        predictor[1 * 16 + 0] = (pp[2] + pp[3] + 1) >> 1;
      predictor[2 * 16 + 3] =
        predictor[1 * 16 + 1] = (pp[2] + pp[3] * 2 + pp[4] + 2) >> 2;
      predictor[1 * 16 + 2] =
        predictor[0 * 16 + 0] = (pp[3] + pp[4] + 1) >> 1;
      predictor[1 * 16 + 3] =
        predictor[0 * 16 + 1] = (pp[3] + pp[4] * 2 + pp[5] + 2) >> 2;
      predictor[0 * 16 + 2] = (pp[4] + pp[5] * 2 + pp[6] + 2) >> 2;
      predictor[0 * 16 + 3] = (pp[5] + pp[6] * 2 + pp[7] + 2) >> 2;
    }
    break;


    case B_HU_PRED: {
      unsigned char *pp = Left;
      predictor[0 * 16 + 0] = (pp[0] + pp[1] + 1) >> 1;
      predictor[0 * 16 + 1] = (pp[0] + pp[1] * 2 + pp[2] + 2) >> 2;
      predictor[0 * 16 + 2] =
        predictor[1 * 16 + 0] = (pp[1] + pp[2] + 1) >> 1;
      predictor[0 * 16 + 3] =
        predictor[1 * 16 + 1] = (pp[1] + pp[2] * 2 + pp[3] + 2) >> 2;
      predictor[1 * 16 + 2] =
        predictor[2 * 16 + 0] = (pp[2] + pp[3] + 1) >> 1;
      predictor[1 * 16 + 3] =
        predictor[2 * 16 + 1] = (pp[2] + pp[3] * 2 + pp[3] + 2) >> 2;
      predictor[2 * 16 + 2] =
        predictor[2 * 16 + 3] =
          predictor[3 * 16 + 0] =
            predictor[3 * 16 + 1] =
              predictor[3 * 16 + 2] =
                predictor[3 * 16 + 3] = pp[3];
    }
    break;


  }
}

#if CONFIG_COMP_INTRA_PRED
void vp8_comp_intra4x4_predict_c(BLOCKD *x,
                               int b_mode, int b_mode2,
                               unsigned char *out_predictor) {
  unsigned char predictor[2][4 * 16];
  int i, j;

  vp8_intra4x4_predict(x, b_mode, predictor[0]);
  vp8_intra4x4_predict(x, b_mode2, predictor[1]);

  for (i = 0; i < 16 * 4; i += 16) {
    for (j = i; j < i + 4; j++) {
      out_predictor[j] = (predictor[0][j] + predictor[1][j] + 1) >> 1;
    }
  }
}
#endif

/* copy 4 bytes from the above right down so that the 4x4 prediction modes using pixels above and
 * to the right prediction have filled in pixels to use.
 */
void vp8_intra_prediction_down_copy(MACROBLOCKD *xd) {
  int extend_edge = (xd->mb_to_right_edge == 0 && xd->mb_index < 2);
  unsigned char *above_right = *(xd->block[0].base_dst) + xd->block[0].dst -
                               xd->block[0].dst_stride + 16;
  unsigned int *src_ptr = (unsigned int *)
      (above_right - (xd->mb_index == 3 ? 16 * xd->block[0].dst_stride : 0));

  unsigned int *dst_ptr0 = (unsigned int *)above_right;
  unsigned int *dst_ptr1 =
    (unsigned int *)(above_right + 4 * xd->block[0].dst_stride);
  unsigned int *dst_ptr2 =
    (unsigned int *)(above_right + 8 * xd->block[0].dst_stride);
  unsigned int *dst_ptr3 =
    (unsigned int *)(above_right + 12 * xd->block[0].dst_stride);

  if (extend_edge) {
    *src_ptr = ((uint8_t *) src_ptr)[-1] * 0x01010101U;
  }

  *dst_ptr0 = *src_ptr;
  *dst_ptr1 = *src_ptr;
  *dst_ptr2 = *src_ptr;
  *dst_ptr3 = *src_ptr;
}
