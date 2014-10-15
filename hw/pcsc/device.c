/*
 * PC/SC pass-through device
 *
 * Copyright (c) 2014 Linaro Limited
 *
 * Author:
 *  Sheng-Yu Chiu <sy.chiu@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#include "hw/sysbus.h"

#include <winscard.h>

#define TYPE_PCSC "pcsc-passthru"
#define PCSCOBJ(obj) OBJECT_CHECK(PCSCState, (obj), TYPE_PCSC)

typedef struct Reader {
    uint8_t index;
    uint32_t ctrl_reg;
    uint32_t addr;
    char *name;

    SCARD_READERSTATE *state;
    SCARDHANDLE handle;
    DWORD active_protocol;
} Reader;

typedef struct PCSCState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t irq_status;

    char *reader_namestr;
    Reader **readers;
    uint8_t num_readers;

    QemuThread monitor_thread;

    /* pcsc-lite related struct */
    SCARDCONTEXT context;
    SCARD_READERSTATE *reader_state;

} PCSCState;

#define test_rv(fct, rv, context) \
do { \
    if (rv != SCARD_S_SUCCESS) \
    { \
        qemu_log("%s: %s\n", fct, pcsc_stringify_error(rv)); \
        (void)SCardReleaseContext(context); \
        exit(-1); \
    } \
} while(0)

/* common control registers */
#define PCSC_REG_NUM_READERS    0x0
#define PCSC_REG_IRQ_STATUS     0x4
#define     PCSC_IRQ_STATE_CHANGE   0x1
#define     PCSC_IRQ_TRANSIT_DONE   0x2
#define PCSC_REG_MAX            0x8

/* per-reader control/status registers */
#define PCSC_REG_READER_CONTROL 0x0
/* prefered protocol, directly mapped to pcsclite */
#define     PCSC_READER_CTL_PROTOCOL_T0     0x0001
#define     PCSC_READER_CTL_PROTOCOL_T1     0x0002
#define     PCSC_READER_CTL_PROTOCOL_T15    0x0004
#define     PCSC_READER_CTL_PROTOCOL_RAW    0x0008
#define     PCSC_READER_CTL_PROTOCOL_MASK   0x000f
/* shared mode, directly mapped to pcsclite */
#define     PCSC_READER_CTL_SHARE_EXCLUSIVE 0x0001
#define     PCSC_READER_CTL_SHARE_SHARED    0x0002
#define     PCSC_READER_CTL_SHARE_DIRECT    0x0003
#define     PCSC_READER_CTL_SHARE_MASK      0x0030
#define     PCSC_READER_CTL_SHARE_SHIFT     4
/* disposition mode, directly mapped to pcsclite */
#define     PCSC_READER_CTL_DISPOSITION_LEAVE_CARD	0x0000
#define     PCSC_READER_CTL_DISPOSITION_RESET_CARD	0x0001
#define     PCSC_READER_CTL_DISPOSITION_UNPOWER_CARD 0x0002
#define     PCSC_READER_CTL_DISPOSITION_EJECT_CARD  0x0003
#define     PCSC_READER_CTL_DISPOSITION_MASK		0x0300
#define     PCSC_READER_CTL_DISPOSITION_SHIFT	8
/* reader commands */
#define     PCSC_READER_CTL_CONNECT         0x1000
#define     PCSC_READER_CTL_DISCONNECT      0x2000
#define     PCSC_READER_CTL_READ_ATR        0x4000
#define PCSC_REG_READER_STATE   0x4
/* reader state, directly mapped to pcsclite */
#define     PCSC_READER_STATE_IGNORE    0x0001
#define     PCSC_READER_STATE_CHANGED   0x0002
#define     PCSC_READER_STATE_UNKNOWN   0x0004
#define     PCSC_READER_STATE_UNAVAILABLE   0x0008
#define     PCSC_READER_STATE_EMPTY     0x0010
#define     PCSC_READER_STATE_PRESENT   0x0020
#define     PCSC_READER_STATE_ATRMATCH  0x0040
#define     PCSC_READER_STATE_EXCLUSIVE 0x0080
#define     PCSC_READER_STATE_INUSE     0x0100
#define     PCSC_READER_STATE_MUTE      0x0200
#define     PCSC_READER_STATE_UNPOWERED 0x0400
#define PCSC_REG_READER_ADDR    0x8
#define PCSC_REG_READER_MAX     0xc
 
static void dump_reader_state(SCARD_READERSTATE *s) 
{
    qemu_log("%s: state: [\n", s->szReader);

    if (s->dwEventState & SCARD_STATE_IGNORE)
        qemu_log("Ignore this reader,\n");

    if (s->dwEventState & SCARD_STATE_UNKNOWN)
        qemu_log("Reader unknown,\n");

    if (s->dwEventState & SCARD_STATE_UNAVAILABLE)
        qemu_log("Status unavailable,\n");

    if (s->dwEventState & SCARD_STATE_EMPTY)
        qemu_log("Card removed,\n");

    if (s->dwEventState & SCARD_STATE_PRESENT)
        qemu_log("Card inserted,\n");

    if (s->dwEventState & SCARD_STATE_ATRMATCH)
        qemu_log("ATR matches card,\n");

    if (s->dwEventState & SCARD_STATE_EXCLUSIVE)
        qemu_log("Exclusive Mode,\n");

    if (s->dwEventState & SCARD_STATE_INUSE)
        qemu_log("Shared Mode,\n");

    if (s->dwEventState & SCARD_STATE_MUTE)
        qemu_log("Unresponsive card,\n");

    if (s->dwEventState & SCARD_STATE_UNPOWERED)
        qemu_log("Reader Unpowered,\n");

    qemu_log("]\n");
}

static void pcsc_update_irq(PCSCState *s) {
    qemu_set_irq(s->irq, !!s->irq_status);
}

static int count_num_of_readers(char *namestr, DWORD namestr_len) 
{

    /* Extract readers from the null separated string and get the total
     * number of readers */
    char *ptr;
    uint8_t num_readers = 0;
    ptr = namestr;
    while (*ptr != '\0')
    {
        ptr += strlen(ptr) + 1;
        num_readers++;
    }
    return num_readers;
}

static void init_reader(char *name, uint8_t index, Reader *r, SCARD_READERSTATE *s)
{
    r->name = name;
    r->index = index;
    r->state = s;

    s->szReader = name;
    s->dwCurrentState = SCARD_STATE_UNAWARE;
    s->cbAtr = sizeof(s->rgbAtr);
}

static void detect_readers(PCSCState *s) 
{
    DWORD reader_namestr_length;
    LONG rv;

    qemu_log("Scanning present readers...");
    rv = SCardListReaders(s->context, NULL, NULL, &reader_namestr_length);
    if (rv != SCARD_E_NO_READERS_AVAILABLE)
        test_rv("SCardListReaders", rv, s->context);

    s->reader_namestr = qemu_oom_check(malloc(reader_namestr_length));
    SCardListReaders(s->context, NULL, s->reader_namestr, &reader_namestr_length);

    s->num_readers = count_num_of_readers(s->reader_namestr, reader_namestr_length);
    qemu_log("%d Readers found\n", s->num_readers);
}

static void create_readers(PCSCState *s)
{
    int i;
    char *name = s->reader_namestr;

    s->readers = qemu_oom_check(malloc(sizeof(Reader *) * s->num_readers));
    s->reader_state = qemu_oom_check(malloc((s->num_readers) * sizeof(SCARD_READERSTATE)));

    for (i = 0; i < s->num_readers; i++)
    {
        s->readers[i] = qemu_oom_check(malloc(sizeof(Reader)));
        init_reader(name, i, s->readers[i], &s->reader_state[i]);

        name += strlen(name) + 1;
    }
}

static int reader_connect(PCSCState *s, Reader *r)
{
    DWORD shared_mode = 0;
    DWORD prefered_protocol = 0;
    LONG rv;

    shared_mode = (r->ctrl_reg >> PCSC_READER_CTL_SHARE_SHIFT) & PCSC_READER_CTL_SHARE_MASK;
    prefered_protocol = r->ctrl_reg & PCSC_READER_CTL_PROTOCOL_MASK;

    rv = SCardConnect(s->context, r->name, shared_mode, prefered_protocol,
                      &r->handle, &r->active_protocol);
    test_rv("SCardConnect", rv, s->context);

    return 0;
}

static int reader_disconnect(PCSCState *s, Reader *r)
{
    DWORD disposition_mode = 0;
    LONG rv;

    disposition_mode = (r->ctrl_reg >> PCSC_READER_CTL_DISPOSITION_SHIFT)
        & PCSC_READER_CTL_DISPOSITION_MASK;

    rv = SCardDisconnect(r->handle, disposition_mode);
    test_rv("SCardDisconnect", rv, s->context);

    return 0;
}

static void *monitor_thread(void *opaque)
{
    PCSCState *s = opaque;
    LONG rv;

    rv = SCardGetStatusChange(s->context, INFINITE, s->reader_state, s->num_readers);
    test_rv("SCardGetStatusChange", rv, s->context);
    while (1) {
        int i;

        /* Now we have an event, check all the readers in the list to see what
         * happened */
        for (i = 0; i < s->num_readers; i++)
        {
            if (s->reader_state[i].dwEventState & SCARD_STATE_CHANGED)
            {
                dump_reader_state(&s->reader_state[i]);
                /* If something has changed the new state is now the current
                 * state */
                s->reader_state[i].dwCurrentState =
                    s->reader_state[i].dwEventState;

                s->irq_status |= PCSC_IRQ_STATE_CHANGE;
            } else {
                /* If nothing changed then skip to the next reader */
                continue;
            }
        }
        pcsc_update_irq(s);

        rv = SCardGetStatusChange(s->context, INFINITE, s->reader_state, s->num_readers);
        test_rv("SCardGetStatusChange", rv, s->context);
    }

    return NULL;
}

static uint64_t pcsc_reader_read(PCSCState *s, Reader *r, hwaddr offset,
                            unsigned size)
{

    switch (offset) {
    case PCSC_REG_READER_CONTROL:
        return r->ctrl_reg;
    case PCSC_REG_READER_ADDR:
        return r->addr;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pcsc_reader_write: Bad offset %x\n", (int)offset);
        return 0;
    }
}

static void pcsc_reader_write(PCSCState *s, Reader *r, hwaddr offset,
                        uint64_t value, unsigned size)
{

    switch (offset) {
    case PCSC_REG_READER_CONTROL:
        r->ctrl_reg = value;
        if (value & PCSC_READER_CTL_CONNECT) {
            reader_connect(s, r);
        }
        else if (value & PCSC_READER_CTL_DISCONNECT) {
            reader_disconnect(s, r);
        }
        break;
    case PCSC_REG_READER_ADDR:
        r->addr = value;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pcsc_reader_write: Bad offset %x\n", (int)offset);
    }
}

static uint8_t get_reader_idx_from_reg_offset(hwaddr off, hwaddr *new_off)
{
    int reader;
    off -= PCSC_REG_MAX;
    reader = off / PCSC_REG_READER_MAX;
    *new_off = off % PCSC_REG_READER_MAX;
    return reader;
}

static uint64_t pcsc_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    PCSCState *s = (PCSCState *)opaque;

    if (offset >= PCSC_REG_MAX) {
        hwaddr reader_reg_offset;
        int idx = get_reader_idx_from_reg_offset(offset, &reader_reg_offset);
        return pcsc_reader_read(s, s->readers[idx], reader_reg_offset, size);
    }

    switch (offset) {
    case PCSC_REG_NUM_READERS:
        return s->num_readers;
    case PCSC_REG_IRQ_STATUS:
        return s->irq_status;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pcsc_read: Bad offset %x\n", (int)offset);
        return 0;
    }
}

static void pcsc_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    PCSCState *s = (PCSCState *)opaque;

    if (offset >= PCSC_REG_MAX) {
        hwaddr reader_reg_offset;
        int idx = get_reader_idx_from_reg_offset(offset, &reader_reg_offset);
        pcsc_reader_write(s, s->readers[idx], reader_reg_offset, value, size);
        return;
    }

    switch (offset) {
    case PCSC_REG_NUM_READERS:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pcsc_write: write to RO reg, offset %x\n", (int)offset);
        break;
    case PCSC_REG_IRQ_STATUS:
        s->irq_status &= ~value;
        pcsc_update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pcsc_write: Bad offset %x\n", (int)offset);
    }
}

static const MemoryRegionOps pcsc_ops = {
    .read = pcsc_read,
    .write = pcsc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void pcsc_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    PCSCState *s = PCSCOBJ(obj);
    LONG rv;

    memory_region_init_io(&s->iomem, OBJECT(s), &pcsc_ops, s, "pcsc", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    rv = SCardEstablishContext(SCARD_SCOPE_SYSTEM, NULL, NULL, &s->context);
    test_rv("SCardEstablishContext", rv, s->context);

    detect_readers(s);
    create_readers(s);

    qemu_thread_create(&s->monitor_thread, "monitor_thread",
                       monitor_thread,
                       s, QEMU_THREAD_JOINABLE);

}

static void pcsc_class_init(ObjectClass *oc, void *data)
{
}

static const TypeInfo pcsc_info = {
    .name          = TYPE_PCSC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PCSCState),
    .instance_init = pcsc_init,
    .class_init    = pcsc_class_init,
};

static void pcsc_register_types(void)
{
    type_register_static(&pcsc_info);
}

type_init(pcsc_register_types)
