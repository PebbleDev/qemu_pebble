/*
 * Virtio Support
 *
 * Copyright IBM, Corp. 2007-2008
 *
 * Authors:
 *  Anthony Liguori   <aliguori@us.ibm.com>
 *  Rusty Russell     <rusty@rustcorp.com.au>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 */

#ifndef _QEMU_VIRTIO_BALLOON_H
#define _QEMU_VIRTIO_BALLOON_H

#include "hw/virtio/virtio.h"
#include "hw/pci/pci.h"

#define TYPE_VIRTIO_BALLOON "virtio-balloon-device"
#define VIRTIO_BALLOON(obj) \
        OBJECT_CHECK(VirtIOBalloon, (obj), TYPE_VIRTIO_BALLOON)

/* from Linux's linux/virtio_balloon.h */

/* The ID for virtio_balloon */
#define VIRTIO_ID_BALLOON 5

/* The feature bitmap for virtio balloon */
#define VIRTIO_BALLOON_F_MUST_TELL_HOST 0 /* Tell before reclaiming pages */
#define VIRTIO_BALLOON_F_STATS_VQ 1       /* Memory stats virtqueue */

/* Size of a PFN in the balloon interface. */
#define VIRTIO_BALLOON_PFN_SHIFT 12

struct virtio_balloon_config
{
    /* Number of pages host wants Guest to give up. */
    uint32_t num_pages;
    /* Number of pages we've actually got in balloon. */
    uint32_t actual;
};

/* Memory Statistics */
#define VIRTIO_BALLOON_S_SWAP_IN  0   /* Amount of memory swapped in */
#define VIRTIO_BALLOON_S_SWAP_OUT 1   /* Amount of memory swapped out */
#define VIRTIO_BALLOON_S_MAJFLT   2   /* Number of major faults */
#define VIRTIO_BALLOON_S_MINFLT   3   /* Number of minor faults */
#define VIRTIO_BALLOON_S_MEMFREE  4   /* Total amount of free memory */
#define VIRTIO_BALLOON_S_MEMTOT   5   /* Total amount of memory */
#define VIRTIO_BALLOON_S_NR       6

typedef struct VirtIOBalloonStat {
    uint16_t tag;
    uint64_t val;
} QEMU_PACKED VirtIOBalloonStat;

typedef struct VirtIOBalloon {
    VirtIODevice parent_obj;
    VirtQueue *ivq, *dvq, *svq;
    uint32_t num_pages;
    uint32_t actual;
    uint64_t stats[VIRTIO_BALLOON_S_NR];
    VirtQueueElement stats_vq_elem;
    size_t stats_vq_offset;
    QEMUTimer *stats_timer;
    int64_t stats_last_update;
    int64_t stats_poll_interval;
} VirtIOBalloon;

#endif
