/******************************************************************************************/
/**** SimCore/RISC-V since 2018-07-05                               ArchLab. TokyoTech ****/
/******************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

void load_binfile(char *fname, uint8_t *mem);

#define DRAM_SIZE    (64*1024*1024) // 64MB main memory
#define DISK_SIZE    (128*1024*1024) // 128MB disk
#define D_INITD_ADDR (16*1024*1024) // after 16MB area, write the init data
//#define D_INITD_ADDR (0x01000000)

#define D_SIZE_DRAM ( 9*1024*1024) //  9MB of bbl + kernel
#define D_SIZE_DEVT ( 4*1024)      //  4KB of device tree
#define D_SIZE_DISK (128*1024*1024) // 128MB of disk image

/*****  generate binary image file for FPGA run                                       *****/
/******************************************************************************************/
int main(int argc, char *argv[]){

    printf("__ generate binary image file to initialize memory for FPGA run\n");
    printf("__ Version 1.0.0, 2020/05/19\n");

    if (argc!=1+3) {
        printf("__ The usage is wrong.\n");
        printf("__ The usage is as follows:\n");
        printf("__ $ ./initmem_gen [bbl binary file] [disk image file] [device tree file]\n");
        return 1;
    }

    char *st_bbl  = argv[1];
    char *st_disk = argv[2];
    char *st_dtb  = argv[3];

    uint8_t *mmem;  // main memory
    mmem = (uint8_t*) malloc(DRAM_SIZE);
    uint8_t *disk;  // disk
    disk = (uint8_t*) malloc(DISK_SIZE);
    if(mmem==NULL || disk==NULL) {
        printf("__ Memory can not be allocated.\n");
        return 1;
    }

    load_binfile(st_bbl, mmem);
    load_binfile(st_dtb, mmem+D_INITD_ADDR);
    load_binfile(st_disk, disk);

    FILE *fp = fopen("initmem.bin", "wb");
    fwrite(mmem,              sizeof(uint8_t), D_SIZE_DRAM, fp);
    fwrite(mmem+D_INITD_ADDR, sizeof(uint8_t), D_SIZE_DEVT, fp);
    fwrite(disk,              sizeof(uint8_t), D_SIZE_DISK, fp);
    fclose(fp);
    printf("__ File initmem.bin was generated.\n");


    free(mmem);
    free(disk);

    fp = fopen("initmem.bin", "rb");
    int i=0;
    uint32_t sum=0;
    uint32_t buf;
    while(fread(&buf, 4, 1, fp)){
        sum += buf;
        i++;
    }
    fclose(fp);
    printf("__ %8d byte file, checksum %08x\n", i*4, sum);

    return 0;
}

/******************************************************************************************/
void load_binfile(char *fname, uint8_t *mem){
    FILE *fp;
    fp = fopen(fname, "rb");
    if(fp == NULL) {
        printf("__ The image file %s cannot be found\n", fname);
        exit(1);
    }
    int i=0;
    while(!feof(fp)){ mem[i++] = getc(fp); }
    fclose(fp);
}

/******************************************************************************************/
