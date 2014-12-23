/*
 * Common code for block device models
 *
 * Copyright (C) 2012 Red Hat, Inc.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later.  See the COPYING file in the top-level directory.
 */

#include "sysemu/blockdev.h"
#include "hw/block/block.h"
#include "qemu/error-report.h"

void blkconf_serial(BlockConf *conf, char **serial)
{
    DriveInfo *dinfo;

    if (!*serial) {
        /* try to fall back to value set with legacy -drive serial=... */
        dinfo = drive_get_by_blockdev(conf->bs);
        *serial = g_strdup(dinfo->serial);
    }
}

int blkconf_geometry(BlockConf *conf, int *ptrans,
                     unsigned cyls_max, unsigned heads_max, unsigned secs_max)
{
    DriveInfo *dinfo;

    if (!conf->cyls && !conf->heads && !conf->secs) {
        /* try to fall back to value set with legacy -drive cyls=... */
        dinfo = drive_get_by_blockdev(conf->bs);
        conf->cyls  = dinfo->cyls;
        conf->heads = dinfo->heads;
        conf->secs  = dinfo->secs;
        if (ptrans) {
            *ptrans = dinfo->trans;
        }
    }
    if (!conf->cyls && !conf->heads && !conf->secs) {
        hd_geometry_guess(conf->bs,
                          &conf->cyls, &conf->heads, &conf->secs,
                          ptrans);
    } else if (ptrans && *ptrans == BIOS_ATA_TRANSLATION_AUTO) {
        *ptrans = hd_bios_chs_auto_trans(conf->cyls, conf->heads, conf->secs);
    }
    if (conf->cyls || conf->heads || conf->secs) {
        if (conf->cyls < 1 || conf->cyls > cyls_max) {
            error_report("cyls must be between 1 and %u", cyls_max);
            return -1;
        }
        if (conf->heads < 1 || conf->heads > heads_max) {
            error_report("heads must be between 1 and %u", heads_max);
            return -1;
        }
        if (conf->secs < 1 || conf->secs > secs_max) {
            error_report("secs must be between 1 and %u", secs_max);
            return -1;
        }
    }
    return 0;
}
