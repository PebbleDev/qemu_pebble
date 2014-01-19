/*
 * ls01x_lcd OLED controller with OSRAM Pictiva 128x64 display.
 *
 * Copyright (c) 2006-2007 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

/* The controller can support a variety of different displays, but we only
   implement one.  Most of the commends relating to brightness and geometry
   setup are ignored. */
#include "hw/ssi.h"
#include "ui/console.h"
#include "qemu/bitops.h"
#define DEBUG_LS01X_LCD 1

#ifdef DEBUG_LS01X_LCD
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "ls01x_lcd: " fmt , ## __VA_ARGS__); } while (0)
#define BADF(fmt, ...) \
do { \
    fprintf(stderr, "ls01x_lcd: error: " fmt , ## __VA_ARGS__); abort(); \
} while (0)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#define BADF(fmt, ...) \
do { fprintf(stderr, "ls01x_lcd: error: " fmt , ## __VA_ARGS__);} while (0)
#endif

/* Scaling factor for pixels.  */
#define MAGNIFY 4

#define CMD_VCOM_CHANGE 0x0
#define CMD_CLEAR_SCREEN 0x4
#define CMD_WRITE_LINE 0x1

enum ls01x_lcd_state
{
    LS01X_LCD_CMD,
    LS01X_LCD_DATA,
    LS01X_LCD_LINENUM,
    LS01X_LCD_LINE_TRAILER,
    LS01X_LCD_TRAILER,
};

typedef struct {
    SSISlave ssidev;
    QemuConsole *con;

    int width;
    int height;
    int redraw;
    int linenum;
    int linepos;
    enum ls01x_lcd_state state;
    uint8_t *framebuffer;
} ls01x_lcd_state;


static uint32_t ls01x_lcd_transfer(SSISlave *dev, uint32_t data)
{
    ls01x_lcd_state *s = FROM_SSI_SLAVE(ls01x_lcd_state, dev);
    int i;
    switch (s->state) {
        /* We don't know ahead of time whether we have a trailer or linenum */
        case LS01X_LCD_LINENUM:
        case LS01X_LCD_TRAILER:
            if(data == 0)
            {
                DPRINTF("Got a final trailer\n");
                s->state = LS01X_LCD_CMD;
                s->redraw = 1;
            }
            else
            {
                DPRINTF("Writing to LineNum %d\n", data);
                s->linenum = data;
                s->linepos = 0;
                s->state = LS01X_LCD_DATA;
            }
            break;
        case LS01X_LCD_DATA:
            DPRINTF("data (%d, %d) = 0x%02x\n", s->linenum, s->linepos, data);
            uint8_t *p = &s->framebuffer[s->linepos + s->linenum * s->width];
            for(i=0;i<8; i++)
            {
                *p++ = !!(data & BIT(i));
            }
            s->linepos += 8;
            if(s->linepos == s->width)
                s->state = LS01X_LCD_LINE_TRAILER;
            break;
        case LS01X_LCD_CMD:
            DPRINTF("cmd 0x%02x\n", data);
            switch (data & 0xFD) {
                case CMD_VCOM_CHANGE:
                    break;
                case CMD_CLEAR_SCREEN:
                    DPRINTF("Clearing LCD\n");
                    s->state = LS01X_LCD_TRAILER;
                    break;
                case CMD_WRITE_LINE:
                    DPRINTF("Write line\n");
                    s->state = LS01X_LCD_LINENUM;
                    break;
            default:
                BADF("Unknown command: 0x%x\n", data);
            }
            return 0;
        case LS01X_LCD_LINE_TRAILER:
            DPRINTF("Got Line Trailer\n");
            s->state = LS01X_LCD_LINENUM;
            break;
        default:
            BADF("Unknown state!?\n");
    }
    return 0;
}

static void ls01x_lcd_update_display(void *opaque)
{
    ls01x_lcd_state *s = (ls01x_lcd_state *)opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    uint8_t *dest;
    uint8_t *src;
    uint32_t colors[2];
    int x;
    int y;
//    int i;
    int line;
    int dest_width;

    if (!s->redraw)
        return;

    DPRINTF("Redrawing\n");
    switch (surface_bits_per_pixel(surface)) {
    case 0:
        return;
    case 15:
    case 16:
        dest_width = 2;
        colors[0] = 0x00;
        colors[1] = 0xFF;
        break;
    case 24:
        dest_width = 3;
        colors[0] = 0x000000;
        colors[1] = 0xFFFFFF;
        break;
    case 32:
        dest_width = 4;
        colors[0] = 0x00000000;
        colors[1] = 0xFFFFFFFF;
        break;
    default:
        BADF("Bad color depth\n");
        return;
    }

    /* TODO: Implement row/column remapping.  */
    dest = surface_data(surface);
    for (y = s->height-1; y >= 0; y--) {
        line = y;
        DPRINTF("Drawing Line %d\n", line);
        src = s->framebuffer + s->width * line + s->width-1;
        for (x = 0; x < s->width; x++) {
            int val;
            val = *src;
//            for (i = 0; i < MAGNIFY; i++) {
                memcpy(dest, &colors[val], dest_width);
                dest += dest_width;
//            }
            src--;
        }
/*        for (i = 1; i < MAGNIFY; i++) {
            memcpy(dest, dest - dest_width * MAGNIFY * 128,
                   dest_width * 128 * MAGNIFY);
            dest += dest_width * 128 * MAGNIFY;
        }*/
    }
    s->redraw = 0;
    dpy_gfx_update(s->con, 0, 0, s->height * MAGNIFY, s->width * MAGNIFY);
}

static void ls01x_lcd_invalidate_display(void * opaque)
{
    ls01x_lcd_state *s = (ls01x_lcd_state *)opaque;
    s->redraw = 1;
}

/*
static void ssd0323_save(QEMUFile *f, void *opaque)
{
    SSISlave *ss = SSI_SLAVE(opaque);
    ssd0323_state *s = (ssd0323_state *)opaque;
    int i;

    qemu_put_be32(f, s->cmd_len);
    qemu_put_be32(f, s->cmd);
    for (i = 0; i < 8; i++)
        qemu_put_be32(f, s->cmd_data[i]);
    qemu_put_be32(f, s->row);
    qemu_put_be32(f, s->row_start);
    qemu_put_be32(f, s->row_end);
    qemu_put_be32(f, s->col);
    qemu_put_be32(f, s->col_start);
    qemu_put_be32(f, s->col_end);
    qemu_put_be32(f, s->redraw);
    qemu_put_be32(f, s->remap);
    qemu_put_be32(f, s->mode);
    qemu_put_buffer(f, s->framebuffer, sizeof(s->framebuffer));

    qemu_put_be32(f, ss->cs);
}

static int ls01x_lcd_load(QEMUFile *f, void *opaque, int version_id)
{
    SSISlave *ss = SSI_SLAVE(opaque);
    ssd0323_state *s = (ls01x_lcd_state *)opaque;
    int i;

    if (version_id != 1)
        return -EINVAL;

    s->cmd_len = qemu_get_be32(f);
    s->cmd = qemu_get_be32(f);
    for (i = 0; i < 8; i++)
        s->cmd_data[i] = qemu_get_be32(f);
    s->row = qemu_get_be32(f);
    s->row_start = qemu_get_be32(f);
    s->row_end = qemu_get_be32(f);
    s->col = qemu_get_be32(f);
    s->col_start = qemu_get_be32(f);
    s->col_end = qemu_get_be32(f);
    s->redraw = qemu_get_be32(f);
    s->remap = qemu_get_be32(f);
    s->mode = qemu_get_be32(f);
    qemu_get_buffer(f, s->framebuffer, sizeof(s->framebuffer));

    ss->cs = qemu_get_be32(f);

    return 0;
}*/

static const GraphicHwOps ls01x_lcd_ops = {
    .invalidate  = ls01x_lcd_invalidate_display,
    .gfx_update  = ls01x_lcd_update_display,
};

static int ls01x_lcd_init(SSISlave *dev)
{
    ls01x_lcd_state *s = FROM_SSI_SLAVE(ls01x_lcd_state, dev);
    DPRINTF("Allocating framebuffer of size %d, %d\n", s->width, s->height);
    fflush(stderr);
    s->framebuffer = g_new(uint8_t, s->width*s->height);
    memset(s->framebuffer, 0, s->width*s->height);
    s->con = graphic_console_init(DEVICE(dev), &ls01x_lcd_ops, s);
    qemu_console_resize(s->con, s->width, s->height);
    s->state = LS01X_LCD_CMD;
//    qdev_init_gpio_in(&dev->qdev, ls01x_lcd_cd, 1);

/*    register_savevm(&dev->qdev, "ssd0323_oled", -1, 1,
                    ssd0323_save, ssd0323_load, s);*/
    return 0;
}

static Property ls01x_lcd_properties[] = {
    DEFINE_PROP_INT32("width", ls01x_lcd_state, width, 144),
    DEFINE_PROP_INT32("height", ls01x_lcd_state, height, 168),
    DEFINE_PROP_END_OF_LIST(),
};

static void ls01x_lcd_class_init(ObjectClass *klass, void *data)
{
    SSISlaveClass *k = SSI_SLAVE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    k->init = ls01x_lcd_init;
    k->transfer = ls01x_lcd_transfer;
    k->cs_polarity = SSI_CS_HIGH;
    dc->props = ls01x_lcd_properties;
}

static const TypeInfo ls01x_lcd_info = {
    .name          = "ls01x_lcd",
    .parent        = TYPE_SSI_SLAVE,
    .instance_size = sizeof(ls01x_lcd_state),
    .class_init    = ls01x_lcd_class_init,
};

static void ls01x_lcd_register_types(void)
{
    type_register_static(&ls01x_lcd_info);
}

type_init(ls01x_lcd_register_types)
