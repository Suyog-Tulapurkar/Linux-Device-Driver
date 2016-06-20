//
//kyouko3 register list and ioctl codes
//

#define KYOUKO3_CONTROL_SIZE (65536)
#define GRAPHICS_ON   (unsigned long)1
#define GRAPHICS_OFF  (unsigned long)0

//DMA Header:
struct kyouko3_dma_hdr {
    uint32_t address:14;
    uint32_t count:10;
    uint32_t opcode:8;
}hdr;

#define VMODE       _IOW(0xcc,0,unsigned long)
#define BIND_DMA    _IOW(0xcc,1,unsigned long)
#define UNBIND_DMA  _IOW(0xcc,5,unsigned long)
#define START_DMA   _IOWR(0xcc,2,unsigned long)
#define FIFO_QUEUE  _IOWR(0xcc,3,unsigned long)
#define FIFO_FLUSH  _IO(0xcc,4)

//registers with basic offset
#define Major_Version     0x0000
#define Minor_Version     0x0004
#define Vendor            0x0008
#define Supported_Features 0x000C
#define Manufacture_Week   0x0010
#define Manufacture_Year   0x0014
#define Manufacture_Lot    0x0018
#define Device_RAM         0x0020
#define Number_of_Frames   0x0024
#define Number_of_DACS     0x0028
#define Internal_FIFO_Size 0x002C

#define Reboot             0x1000
#define Flags              0x1004
#define ModeSet            0x1008
#define InterruptSet       0x100C
#define Acceleration       0x1010
#define FifoStart          0x1020
#define FifoEnd            0x1024

#define BufferA_Address    0x2000
#define BufferA_Config     0x2008
#define DMA_Target         0x2100
#define DMA_Source         0x2104
#define DMA_Config         0x2108

#define Command_Primitive  0x3000
#define Vertex_Emit        0x3004
#define Clear_Buffer       0x3008
#define Flush              0x3FFC

#define status             0x4008
#define FifoHead           0x4010
#define FifoTail           0x4014

#define Vertex_Coordinate  0x5000
#define Vertex_Color       0x5010
#define Clear_Color        0x5100

#define Frame_Objects      0x8000
#define DAC_Objects        0x9000

enum{
     /*Offsets into frame configuration*/
     _FColumns          = 0x0000,
     _FRows             = 0x0004,
     _FRowPitch         = 0x0008,
     _FFormat           = 0x000C,
     _FAddress          = 0x0010,

     /*Offsets into DAC configurations*/
     _DWidth            = 0x0000,
     _DHeight           = 0x0004,
     _DVirtX            = 0x0008,
     _DVirtY            = 0x000C,
     _DFrame            = 0x0010,
};
