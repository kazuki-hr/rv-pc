/******************************************************************************************/
/**** SimCore/RISC-V since 2018-07-05                             ArchLab. TokyoTech   ****/
/******************************************************************************************/
//#include <cstdint.h>

#include <stdint.h>
#include "simrv.h"

//volatile uint8_t *HOGE = (uint8_t *)0x0;
#define DISK_DEBUG 1
//#include "define.h"
/**** VirtIO 0x40000000 ~                                                              ****/
/******************************************************************************************/
#define VIRTIO_BASE_ADDR (0x40000000) // NotChange
#define VIRTIO_SIZE      (0x08000000) //
#define VRING_DESC_F_NEXT     (1)
#define VRING_DESC_F_WRITE    (2)
#define VRING_DESC_F_INDIRECT (4)

/* console */
#define CONSOLE_MAX_QUEUE_NUM (2)
#define VIRTIO_CONSOLE_IRQ (1)

/* block device (disk) */
#define SECTOR_SIZE         (512)
#define DISK_BUF_SIZE       (512 * 512)
#define DISK_SIZE           (64 * 1024 * 1024)
#define DISK_MAX_QUEUE_NUM  (4)
#define VIRTIO_DISK_IRQ     (2)
#define VIRTIO_BLK_T_IN     (0)
#define VIRTIO_BLK_T_OUT    (1)
#define VIRTIO_BLK_S_OK     (0)
#define VIRTIO_BLK_S_IOERR  (1)
#define VIRTIO_BLK_S_UNSUPP (2)

/**** PLIC (Platform-Level Interrupt Contoroller) 0x50000000 ~                         ****/
/******************************************************************************************/
#define PLIC_BASE_ADDR   (0x50000000) // NotChange:
#define PLIC_SIZE        (0x00400000)
#define PLIC_HART_BASE   (0x200000)
#define PLIC_HART_SIZE   (0x1000)

/**** CLINT (Core Local Interruputer) 0x60000000 ~                                     ****/
/******************************************************************************************/
#define CLINT_BASE_ADDR  (0x60000000) // NotChange:
#define CLINT_SIZE       (0x000c0000) //

/**** DRAM (Main Memory) 0x80000000 ~                                                  ****/
/******************************************************************************************/
#define DRAM_BASE_ADDR (0x80000000)
#define DRAM_SIZE      (64 * 1024 * 1024)
#define DRAM_MASK      (0x3fffffff)
#define D_PAGE_SHIFT   (12)          // page shift for page size of4KB
#define D_PAGE_MASK    (0x00000fff)  // page mask  for page size of4KB
#define TLB_SIZE       (4)


/******************************************************************************************/
#define DISK_MAGIC_VALUE       0x74726976
#define DISK_VERSION           2
#define DISK_DEVIDE_ID         2
#define DISK_VENDOR_ID         0xffff
#define DISK_DEVICE_FEATURES   1
#define DISK_CONFIG_GENERATION 0
#define DISK_QUEUE_NUM_MAX     4

#define ETHER_MTU              1500
#define ETHER_MIN_SIZE         60
#define CRCPOLY2 0xEDB88320UL


#define ETHER_REG_SEND_BUSY       0x0
#define ETHER_REG_SEND_PACKET_LEN 0x4
#define ETHER_REG_CMD_SEND        0x8
#define ETHER_REG_RECV_PACKET_LEN 0xc
#define ETHER_REG_RECV_DONE       0x10

volatile uint32_t *ETHER_REG_BASE = (uint32_t*)0x4000d800;

#define ETHER_REG(r) (ETHER_REG_BASE[r/4])

typedef struct QueueState {
    uint32_t Ready;
    uint32_t Notify;
    uint32_t DescLow;
    uint32_t DescHigh;
    uint32_t AvailLow;
    uint32_t AvailHigh;
    uint32_t UsedLow;
    uint32_t UsedHigh;
    uint16_t last_avail_idx;
}QueueState;

typedef struct BlockRequestHeader {
    uint32_t type;
    uint32_t ioprio;
    uint64_t sector_num;
}BlockRequestHeader;

typedef struct EtherPacketHeader {
        uint8_t flags;
        uint8_t gso_type;
        uint16_t hdr_len;
        uint16_t gso_size;
        uint16_t csum_start;
        uint16_t csum_offset;
        uint16_t num_buffers;
}EtherPacketHeader;

typedef struct Descriptor {
    uint64_t adr;
    uint32_t len;
    uint16_t flags;
    uint16_t next;
}Descriptor;


int disk_debug_num;

extern uint8_t scancode_to_keycode[];

/******************************************************************************************/
uint32_t ram_ld(uint32_t addr, int n, volatile uint8_t *ram){
    if(n!=1 && n!=2 && n!=4){
        //printf("__ Error: ram_r() not supported n=%d\n", n);
        simrv_puts("__ Error: ram_r() not supported n=");
        simrv_puth(n);
        simrv_puts("\n");
        simrv_exit(0);
    }
    uint32_t data = 0;
    for (int i=0; i<n; i++) {
        //data |= ((uint32_t)ram[(addr + i) & DRAM_MASK]) << (8*i);     // NOTE!
        data |= ((uint32_t)ram[(addr + i) /*& DRAM_MASK*/]) << (8*i);
    }

    return data;
}
/******************************************************************************************/
void ram_st(uint32_t addr, uint32_t data, int n, uint8_t *ram){
    if(n!=1 && n!=2 && n!=4){
        //printf("__ Error: dsk_w() not supported n=%d\n", n);
        simrv_puts("__ Error: dsk_w() not supported n=");
        simrv_puth(n);
        simrv_puts("\n");
        simrv_exit(0);
    }
    if(n==1){
        ram[addr /*& DRAM_MASK*/] = data & 0xff;
    }
    else if (n==2){
        ram[ addr    /*& DRAM_MASK*/] =  data       & 0xff;
        ram[(addr+1) /*& DRAM_MASK*/] = (data >> 8) & 0xff;
    }
    else if (n==4){
        ram[ addr    /*& DRAM_MASK*/] =  data       & 0xff;
        ram[(addr+1) /*& DRAM_MASK*/] = (data >> 8) & 0xff;
        ram[(addr+2) /*& DRAM_MASK*/] = (data >>16) & 0xff;
        ram[(addr+3) /*& DRAM_MASK*/] = (data >>24) & 0xff;
    }
}

/***** disk load                                                                      *****/
/******************************************************************************************/
uint32_t dsk_ld(uint32_t addr, int n, uint8_t *dsk){
    if(n!=1 && n!=2 && n!=4){simrv_puts("__ Error: dsk_r() not supported n="); simrv_puth(n); simrv_puts("\n"); simrv_exit(0);}
    uint32_t data = 0;

    uint32_t* dsk_tmp = (uint32_t*)dsk;

    //for (int i=0; i<n; i++) { data |= ((uint32_t)dsk[addr + i]) << (8*i); }
    uint32_t t_data = (dsk_tmp[addr/4]);
    //uint32_t data2 = ((t_data & 0xff) << 24) | ((t_data & 0xff00) << 8) | ((t_data & 0xff0000) >> 8) | ((t_data & 0xff000000) >> 24);
    /*if(data != t_data){
        simrv_puth(addr);
        simrv_puts(" ");
        simrv_puth(data);
        simrv_puts(" ");
        simrv_puth(t_data);
        simrv_puts("\n");
    }*/
    //data = t_data;
    //if(DISK_DEBUG) printf("__%03d dsk r %d %08x %08x\n", disk_debug_num, n, addr, data);
    return t_data;
}

/***** disk store                                                                     *****/
/******************************************************************************************/
void dsk_st(uint32_t addr, uint32_t data, int n, uint8_t *dsk){
    if(n!=1 && n!=2 && n!=4){simrv_puts("__ Error: dsk_w() not supported n="); simrv_puth(n); simrv_puts("\n"); simrv_exit(0);}

    uint32_t* dsk_tmp = (uint32_t*)dsk;
    dsk_tmp[addr/4] = data;

    /*if(n==1){
        dsk[addr] = data & 0xff;
    }
    else if (n==2){
        dsk[ addr   ] =  data       & 0xff;
        dsk[(addr+1)] = (data >> 8) & 0xff;
    }
    else if (n==4){
        dsk[ addr   ] =  data       & 0xff;
        dsk[(addr+1)] = (data >> 8) & 0xff;
        dsk[(addr+2)] = (data >>16) & 0xff;
        dsk[(addr+3)] = (data >>24) & 0xff;
    }*/
    //if(DISK_DEBUG) printf("__%03d dsk w %d %08x %08x\n", disk_debug_num, 1, addr, data);
}

/*** update the used ring                                                              ****/
/******************************************************************************************/
void update_descriptor(uint32_t desc_idx, uint32_t desc_len, int q_num,
                       QueueState *qs, uint8_t *mmem){
    uint32_t addr_used_idx = qs->UsedLow + 2;
    uint32_t index = (uint16_t)ram_ld(addr_used_idx, 2, mmem);

    ram_st(addr_used_idx, index+1, 2, mmem);

    uint32_t addr_used_entry = qs->UsedLow + 4 + (index & (q_num - 1)) * 8;
    ram_st(addr_used_entry,   desc_idx, 4, mmem);
    ram_st(addr_used_entry+4, desc_len, 4, mmem);
}

/******************************************************************************************/
#define DESC_SIZE 16 /* descriptor size 16 byte */
/******************************************************************************************/
/*** console_request for display output                                                ****/
/******************************************************************************************/
void cons_request(uint8_t *mmem, uint32_t q_num, QueueState *qs){
    Descriptor desc;
    uint8_t *p;
    disk_debug_num++; /* just for debug */

    uint16_t avail_idx = (uint16_t)ram_ld(qs->AvailLow+2, 2, mmem);
    while (qs->last_avail_idx != avail_idx) { ///// Note!!

        uint32_t adr = qs->AvailLow + 4 + (qs->last_avail_idx & (q_num - 1)) * 2;
        uint16_t desc_idx_header = ram_ld(adr, 2, mmem);
        uint32_t desc_adr_header = desc_idx_header * DESC_SIZE + qs->DescLow;

        p = (uint8_t*)&desc;
        for(int i=0; i<DESC_SIZE; i++){ *p = ram_ld(desc_adr_header+i, 1, mmem); p++; }

        for(int i=0; i<(int)desc.len; i++){ /***** write to stdout *****/
            uint8_t d = ram_ld(desc.adr+i, 1, mmem);
            simrv_putc(d);
        }

        update_descriptor(desc_idx_header, 0, q_num, qs, mmem);
        qs->last_avail_idx++;
    }
}

/******************************************************************************************/
/*** disc sector read & write                                                          ****/
/******************************************************************************************/
void disk_request(uint8_t *mmem, uint8_t *mdsk, int q_num, QueueState *qs){
    Descriptor desc;
    BlockRequestHeader header;
    uint8_t *p;
    disk_debug_num++; /* just for debug */


    uint16_t avail_idx = (uint16_t)ram_ld(qs->AvailLow+2, 2, mmem);
    while (qs->last_avail_idx != avail_idx) { /***** header -> sector -> footer *****/

        // (1) header
        uint32_t adr = qs->AvailLow + 4 + (qs->last_avail_idx & (q_num - 1)) * 2;
        uint16_t desc_idx_header = ram_ld(adr, 2, mmem);
        uint32_t desc_adr_header = desc_idx_header * DESC_SIZE + qs->DescLow;

        p = (uint8_t*)&desc;
        for(int i=0; i<DESC_SIZE; i++){ *p = ram_ld(desc_adr_header+i, 1, mmem); p++; }

        p = (uint8_t*)&header;
        for(int i=0; i<(int)desc.len; i++){ *p = ram_ld(desc.adr+i, 1, mmem); p++; }
        if (desc.len!=16) { simrv_puts("__ ERROR: disk_request() desc.len!=16\n"); simrv_exit(0); }

        // (2) sector
        uint16_t desc_idx_sector = desc.next;
        uint32_t desc_adr_sector = desc_idx_sector * DESC_SIZE + qs->DescLow;
        p = (uint8_t*)&desc;
        for(int i=0; i<DESC_SIZE; i++){ *p = ram_ld(desc_adr_sector+i, 1, mmem); p++; }

        uint32_t sector_len = desc.len;
        uint32_t sector_adr = (uint32_t)desc.adr;

        // (3) footer
        uint16_t desc_idx_footer = desc.next;
        uint32_t desc_adr_footer = desc_idx_footer * DESC_SIZE + qs->DescLow;
        p = (uint8_t*)&desc;
        for(int i=0; i<DESC_SIZE; i++){ *p = ram_ld(desc_adr_footer+i, 1, mmem); p++; }

        uint32_t footer_adr = (uint32_t)desc.adr;

        uint32_t request_size = 0;
        switch (header.type) {
        case VIRTIO_BLK_T_IN: { /////  disk -> dram
            request_size = sector_len + 1;
            for(int i=0; i<(int)sector_len; i=i+4){ //++){
                uint32_t d = dsk_ld(header.sector_num * SECTOR_SIZE + i, 4, mdsk);
                ram_st(sector_adr+i, d, 4, mmem);
            }
            ram_st(footer_adr, VIRTIO_BLK_S_OK, 1, mmem);
            break; }
        case VIRTIO_BLK_T_OUT: { ///// dram -> disk
            request_size = 1;
            for(int i=0; i<(int)sector_len; i=i+4){
                uint32_t d = ram_ld(sector_adr+i, 4, mmem);
                dsk_st(header.sector_num * SECTOR_SIZE + i, d, 4, mdsk);
            }
            ram_st(footer_adr, VIRTIO_BLK_S_OK, 1, mmem);
            break; }
        default: {
            /*Linux implements command type VIRTIO_BLK_T_GET_ID (=8) in include/uapi/linux/virtio_blk.h
              even though the VirtIO spec does not define it.
              We just respond with VIRTIO_BLK_S_UNSUPP.
            */
            request_size = 1;
            simrv_puts("__ ERROR: disk unknown header "); simrv_puth(header.type); simrv_puts("\n");
            ram_st(footer_adr, VIRTIO_BLK_S_UNSUPP, 1, mmem);
         }
        }

        update_descriptor(desc_idx_header, request_size, q_num, qs, mmem);
        qs->last_avail_idx++;
    }
}

/******************************************************************************************/
/*** input from uart                                                              ****/
/******************************************************************************************/
void uart_request(uint8_t *mmem, uint32_t q_num, QueueState *qs, uint8_t buf){
    Descriptor desc;
    uint8_t *p;

    //buf = 'a';
    if (!qs->Ready) return;

    //simrv_puts("READY\n");

    uint16_t avail_idx = (uint16_t)ram_ld(qs->AvailLow+2, 2, mmem);
    if (qs->last_avail_idx == avail_idx) return;
    //simrv_puts("AVAIL\n");
    uint32_t adr = qs->AvailLow + 4 + (qs->last_avail_idx & (q_num - 1)) * 2;
    uint16_t desc_idx_header = ram_ld(adr, 2, mmem);
    uint32_t desc_adr_header = desc_idx_header * DESC_SIZE + qs->DescLow;

    p = (uint8_t*)&desc;
    for(int i=0; i<DESC_SIZE; i++){ *p = ram_ld(desc_adr_header+i, 1, mmem); p++; }

    ram_st(desc.adr, (uint32_t)buf, 1, mmem); /*****/

    update_descriptor(desc_idx_header, 1, 2, qs, mmem);
    qs->last_avail_idx++;
    //simrv_puts("RET\n");
}

static inline uint32_t update_crc(uint32_t crc, uint8_t d){
    crc ^= d;
    for (int j = 0; j < 8; j++){
        if (crc & 1) {
            crc = (crc >> 1) ^ CRCPOLY2;
        } else {
            crc >>= 1;
        }
    }
    return crc;
}

void ether_send(uint8_t *mmem, uint32_t buf_addr, int q_num, QueueState *qs){
    Descriptor desc;
    EtherPacketHeader header;
    uint8_t *p;

    uint16_t avail_idx = (uint16_t)ram_ld(qs->AvailLow+2, 2, mmem);
    while (qs->last_avail_idx != avail_idx) {

        uint32_t adr = qs->AvailLow + 4 + (qs->last_avail_idx & (q_num - 1)) * 2;
        uint16_t desc_idx_header = ram_ld(adr, 2, mmem);
        uint32_t desc_adr_header = desc_idx_header * DESC_SIZE + qs->DescLow;
        uint32_t crc = 0xffffffff;

        p = (uint8_t*)&desc;
        for(int i=0; i<DESC_SIZE; i++){ *p = ram_ld(desc_adr_header+i, 1, mmem); p++; }

        p = (uint8_t*)&header;
        for(int i=0; i < sizeof(EtherPacketHeader); i++){ *p = ram_ld(desc.adr+i, 1, mmem); p++; }

        uint32_t packet_byte_cnt = desc.len - sizeof(EtherPacketHeader);
        for(int i = 0; i < packet_byte_cnt; i++){
            uint8_t d;
            d = ram_ld((uint32_t)desc.adr+sizeof(EtherPacketHeader)+i, 1, mmem);
            ram_st(buf_addr+i, d, 1, mmem);
            crc = update_crc(crc, d);
        }


        while (desc.flags & VRING_DESC_F_NEXT){
            uint16_t desc_idx_next = desc.next;
            uint32_t desc_addr_next = desc_idx_next * DESC_SIZE + qs->DescLow;
            p = (uint8_t*)&desc;
            for(int i=0; i<DESC_SIZE; i++){ *p = ram_ld(desc_addr_next+i, 1, mmem); p++; }

            for(int i=0; i<(int)desc.len; i++){
                uint8_t d;
                d = ram_ld((uint32_t)desc.adr+i, 1, mmem);
                ram_st(buf_addr+packet_byte_cnt+i, d, 1, mmem);
                crc = update_crc(crc, d);
            }
            packet_byte_cnt += desc.len;
        }


        if (packet_byte_cnt < ETHER_MIN_SIZE){
            for (int i = 0; i < ETHER_MIN_SIZE - packet_byte_cnt; i++){
                uint8_t d = 0;
                ram_st(buf_addr+packet_byte_cnt+i, d, 1, mmem);
                crc = update_crc(crc, d);
            }
            packet_byte_cnt = ETHER_MIN_SIZE;
        }

        crc ^= 0xffffffff;
        for (int i = 0; i < 4; i++){
            uint8_t byte = (crc >> (8*i)) & 0xff;
            ram_st(buf_addr+packet_byte_cnt+i, byte , 1, mmem);
        }

        volatile int i = 0;
        while(ETHER_REG(ETHER_REG_SEND_BUSY) || i < 100){i++;}
        ETHER_REG(ETHER_REG_SEND_PACKET_LEN) = packet_byte_cnt+12;
        ETHER_REG(ETHER_REG_CMD_SEND) = 1;
        update_descriptor(desc_idx_header, 0, q_num, qs, mmem);
        qs->last_avail_idx++;
    }
}

static void mem_set(uint8_t *s, uint8_t c, uint32_t n){
    for (int i = 0; i < (int)n; i++) {
        s[i] = c;
    }
}


void ether_recv(uint8_t *mmem, uint32_t buf_addr, int q_num, QueueState *qs){
       Descriptor desc;
       EtherPacketHeader header;
       uint8_t *p;


       if(!qs->Ready) goto done;


       uint16_t avail_idx = (uint16_t)ram_ld(qs->AvailLow+2, 2, mmem);

       if (qs->last_avail_idx == avail_idx) goto done;
       uint32_t adr = qs->AvailLow + 4 + (qs->last_avail_idx & (q_num - 1)) * 2;
       uint16_t desc_idx_header = ram_ld(adr, 2, mmem);
       uint32_t desc_adr_header = desc_idx_header * DESC_SIZE + qs->DescLow;

       p = (uint8_t*)&desc;
       for(int i=0; i<DESC_SIZE; i++){ *p = ram_ld(desc_adr_header+i, 1, mmem); p++; }

       mem_set((uint8_t *)&header, 0, sizeof(header));
       header.num_buffers = 1;

       p = (uint8_t *)&header;
       for (int i = 0; i < sizeof(header); i++){
           ram_st(desc.adr+i, *p, 1, mmem);
           p++;
       }


       uint32_t packet_len = ETHER_REG(ETHER_REG_RECV_PACKET_LEN);
       for (int i = 0; i < packet_len -4; i++){
           uint8_t d;
           d = ram_ld(buf_addr+i, 1, mmem);
           ram_st(desc.adr+sizeof(header)+i, d, 1, mmem);

       }

       update_descriptor(desc_idx_header, packet_len+sizeof(header)-4, q_num, qs, mmem);
       qs->last_avail_idx++;

done:
        ETHER_REG(ETHER_REG_RECV_DONE) = 1;
}


void keybrd_request(uint8_t *mmem, uint32_t q_num, QueueState *qs){
    Descriptor desc;
    uint8_t *p;

    //buf = 'a';
    if (!qs->Ready) return;


    uint16_t avail_idx = (uint16_t)ram_ld(qs->AvailLow+2, 2, mmem);
    if (qs->last_avail_idx == avail_idx) return;

    uint32_t adr = qs->AvailLow + 4 + (qs->last_avail_idx & (q_num - 1)) * 2;
    uint16_t desc_idx_header = ram_ld(adr, 2, mmem);
    uint32_t desc_adr_header = desc_idx_header * DESC_SIZE + qs->DescLow;

    p = (uint8_t*)&desc;
    for(int i=0; i<DESC_SIZE; i++){ *p = ram_ld(desc_adr_header+i, 1, mmem); p++; }


    update_descriptor(desc_idx_header, 0, q_num, qs, mmem);
    qs->last_avail_idx++;
}

// from tinyemu/virtio.c

#define VIRTIO_INPUT_EV_SYN 0x00
#define VIRTIO_INPUT_EV_KEY 0x01
#define VIRTIO_INPUT_EV_REL 0x02
#define VIRTIO_INPUT_EV_ABS 0x03
#define VIRTIO_INPUT_EV_REP 0x14

#define REL_X 0x00
#define REL_Y 0x01
#define REL_Z 0x02
#define REL_WHEEL 0x08

int virtio_input_event(uint8_t *mmem, uint32_t q_num, QueueState *qs, uint16_t type, uint16_t code, uint32_t value){
    Descriptor desc;
    uint8_t *p;

    //buf = 'a';
    if (!qs->Ready) return -1;

    uint16_t avail_idx = (uint16_t)ram_ld(qs->AvailLow+2, 2, mmem);
    if (qs->last_avail_idx == avail_idx) return -1;
    //simrv_puts("AVAIL\n");
    uint32_t adr = qs->AvailLow + 4 + (qs->last_avail_idx & (q_num - 1)) * 2;
    uint16_t desc_idx_header = ram_ld(adr, 2, mmem);
    uint32_t desc_adr_header = desc_idx_header * DESC_SIZE + qs->DescLow;

    p = (uint8_t*)&desc;
    for(int i=0; i<DESC_SIZE; i++){ *p = ram_ld(desc_adr_header+i, 1, mmem); p++; }

    ram_st(desc.adr, (uint32_t)type, 2, mmem); /*****/
    ram_st(desc.adr+2, (uint32_t)code, 2, mmem);
    ram_st(desc.adr+4, (uint32_t)value, 4, mmem);

    update_descriptor(desc_idx_header, 8, q_num, qs, mmem);
    qs->last_avail_idx++;
    return 0;
}

void keybrd_event(uint8_t *mmem, uint32_t q_num, QueueState *qs, uint16_t scancode){
    uint8_t is_down = scancode >> 8;
    uint8_t code    = scancode_to_keycode[scancode & 0xff];
    int ret = virtio_input_event(mmem, q_num, qs, VIRTIO_INPUT_EV_KEY, code, is_down);
    if (ret) return;
    virtio_input_event(mmem, q_num, qs, VIRTIO_INPUT_EV_SYN, 0, 0);
}

#define BTN_LEFT         0x110
#define BTN_RIGHT        0x111
#define BTN_MIDDLE       0x112

static const uint16_t buttons_list[] = {
    BTN_LEFT, BTN_RIGHT, BTN_MIDDLE
};


void mouse_event(uint8_t *mmem, uint32_t q_num, QueueState *qs, int32_t dx, int32_t dy, uint32_t btn){
    int ret, i, b, last_b;
    unsigned int buttons;
    volatile uint32_t *buttons_state = ((uint32_t * )0x40011ff0);

    ret = virtio_input_event(mmem, q_num, qs, VIRTIO_INPUT_EV_REL, REL_X, dx);
    if (ret) return;
    ret = virtio_input_event(mmem, q_num, qs, VIRTIO_INPUT_EV_REL, REL_Y, (-1)*dy);
    if (ret) return;

    buttons = btn & 7;
    if (buttons != (*buttons_state)) {
        for(i = 0; i < 3; i++) {
            b = (buttons >> i) & 1;
            last_b = ((*buttons_state) >> i) & 1;
            if (b != last_b) {
                ret = virtio_input_event(mmem, q_num, qs, VIRTIO_INPUT_EV_KEY, buttons_list[i], b);
                if (ret != 0)
                    return;
            }
        }
        *buttons_state = buttons;
    }

    virtio_input_event(mmem, q_num, qs, VIRTIO_INPUT_EV_SYN, 0, 0);
}

/******************************************************************************************/

/****************** Microcontroller memory map **********************
0x40009000 +------------------------+
           | Mode, qnum, qsel       |
0x4000a000 +------------------------+
           | Console queue          |
0x4000b000 +------------------------+
           | Disk queue             |
0x4000c000 +------------------------+
           | UART receive buffer    |
0x4000d000 +------------------------+
           | Ethernet queue + regs  |
0x4000e000 +------------------------+
           | Ethernet send buffer   |
0x4000f000 +------------------------+
           | Ethernet receive buffer|
0x40010000 +------------------------+
           | Keyboard queue         |
0x40011000 +------------------------+
           | Mouse queue            |
0x40012000 +------------------------+
            ...
0x80000000 +------------------------+
           | DRAM                   |
0x84000000 +------------------------+
            ...
0x90000000 +------------------------+
           | SD card contents       |
0x100000000 +------------------------+


*/


int main(){
    // Mode (1: Console, 2: Disk)
    uint32_t *MODE = (uint32_t*)0x40009000;
    uint32_t mode = *MODE;

    // QNUM
    uint32_t* QNUM;
    QNUM = (uint32_t*)0x40009004;
    int qnum = *QNUM;

    // QSEL
    uint32_t* QSEL;
    QSEL = (uint32_t*)0x40009008;
    int idx = *QSEL;

    // Queues for Console
    QueueState *CONS_Q;
    CONS_Q = (QueueState *)0x4000a000;
    // Queues for Disk
    QueueState *DISK_Q;
    DISK_Q = (QueueState *)0x4000b000;

    // Keyboard input buffer
    uint8_t *CONS_FIFO = (uint8_t*)0x4000c000;
    int cons_fifo = *CONS_FIFO;

    QueueState *ETHER_Q;
    ETHER_Q = (QueueState *)0x4000d000;

    QueueState *KEYBRD_Q;
    KEYBRD_Q = (QueueState *)0x40010000;
    uint16_t scancode = *((uint16_t * )0x40010ffc);

    QueueState *MOUSE_Q;
    MOUSE_Q = (QueueState *)0x40011000;
    uint32_t btn = *((uint32_t * )0x40011ff4);
    int32_t dy = *((uint32_t * )0x40011ff8);
    int32_t dx = *((uint32_t * )0x40011ffc);

    if((idx > 1 && mode==1)){
        simrv_puts("ERROR! INDEX OVERFLOW MODE1\n");
    }
    else if((idx > 3 && mode==2)){
        simrv_puts("ERROR! INDEX OVERFLOW MODE2\n");
    }
    /*else if((idx > 0 && mode==3)){
        simrv_puts("ERROR! INDEX OVERFLOW MODE3\n");
    }*/
    QueueState* tc_queue = (idx==0) ? &CONS_Q[0] : &CONS_Q[1];
    QueueState* td_queue = (idx==0) ? &DISK_Q[0] : (idx==1) ? &DISK_Q[1] : (idx==2) ? &DISK_Q[2] : &DISK_Q[3];
    QueueState* te_queue = (idx==0) ? &ETHER_Q[0] : &ETHER_Q[1];
    QueueState* tk_queue = (idx==0) ? &KEYBRD_Q[0] : &KEYBRD_Q[1];

    if(mode == 1){
        cons_request(0, qnum, tc_queue);
    }
    else if(mode == 2){
        disk_request(0, (uint8_t *)0x90000000, qnum, td_queue);
    }
    else if(mode == 3){
        //simrv_puts("MODE3\n");
        uart_request(0, qnum, &CONS_Q[0], cons_fifo);
    }
    else if(mode == 4){
        if (idx==1) ether_send(0, 0x4000e000+8, qnum, te_queue);
    }
    else if (mode == 5){
        ether_recv(0, 0x4000f000, qnum, &ETHER_Q[0]);
    }
    else if (mode == 6){
        if (idx == 0) keybrd_event(0, qnum, tk_queue, scancode);
        else keybrd_request(0, qnum, tk_queue);
    } else if (mode == 7){
        mouse_event(0, qnum, MOUSE_Q, dx, dy, btn);
    }
    simrv_exit();
}
/******************************************************************************************/
