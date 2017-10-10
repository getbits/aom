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

#ifndef AV1_ENCODER_AQ_MBTREE_H_
#define AV1_ENCODER_AQ_MBTREE_H_

#include <limits.h>
#include <math.h>

#include "av1/encoder/segmentation.h"

#define AV1ALIGN(x, a) (((x)+(a)-1)&~((a)-1))

#define AQ_C_SEGMENTS 7

typedef struct MBTreeEntry {
    float prop_cost;
    float last_intra;
    struct MBTreeEntry *subdiv; /* 4 subdivisions */
} MBTreeEntry;

typedef struct MBTreeContext {
  uint8_t *scratch_buf;

  MBTreeEntry *tree;
  int tree_width;
  int tree_height;

  int segment_qs[AQ_C_SEGMENTS];
} MBTreeContext;

struct macroblock;

void av1_mbtree_update(struct AV1_COMP *cpi);
void av1_mbtree_frame_setup(struct AV1_COMP *cpi);
void av1_mbtree_select_segment(const struct AV1_COMP *cpi, struct macroblock *, BLOCK_SIZE bs,
                               int mi_row, int mi_col);
void av1_mbtree_init(struct AV1_COMP *cpi);
void av1_mbtree_uninit(struct AV1_COMP *cpi);

#endif  // AV1_ENCODER_AQ_MBTREE_H_
