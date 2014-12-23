/*
 * Generic device-tree-driven paravirt PPC e500 platform
 *
 * Copyright 2012 Freescale Semiconductor, Inc.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of  the GNU General  Public License as published by
 * the Free Software Foundation;  either version 2 of the  License, or
 * (at your option) any later version.
 */

#include "config.h"
#include "qemu-common.h"
#include "e500.h"
#include "hw/boards.h"
#include "sysemu/device_tree.h"
#include "hw/pci/pci.h"
#include "hw/ppc/openpic.h"
#include "kvm_ppc.h"

static void e500plat_fixup_devtree(PPCE500Params *params, void *fdt)
{
    const char model[] = "QEMU ppce500";
    const char compatible[] = "fsl,qemu-e500";

    qemu_fdt_setprop(fdt, "/", "model", model, sizeof(model));
    qemu_fdt_setprop(fdt, "/", "compatible", compatible,
                     sizeof(compatible));
}

static void e500plat_init(MachineState *machine)
{
    PPCE500Params params = {
        .pci_first_slot = 0x1,
        .pci_nr_slots = PCI_SLOT_MAX - 1,
        .fixup_devtree = e500plat_fixup_devtree,
        .mpic_version = OPENPIC_MODEL_FSL_MPIC_42,
    };

    /* Older KVM versions don't support EPR which breaks guests when we announce
       MPIC variants that support EPR. Revert to an older one for those */
    if (kvm_enabled() && !kvmppc_has_cap_epr()) {
        params.mpic_version = OPENPIC_MODEL_FSL_MPIC_20;
    }

    ppce500_init(machine, &params);
}

static QEMUMachine e500plat_machine = {
    .name = "ppce500",
    .desc = "generic paravirt e500 platform",
    .init = e500plat_init,
    .max_cpus = 32,
};

static void e500plat_machine_init(void)
{
    qemu_register_machine(&e500plat_machine);
}

machine_init(e500plat_machine_init);
