
#define readreg(addr) ioread32be(drvdata->regs + addr)
#define writereg(addr, val) iowrite32be(val, drvdata->regs + addr)

#define setreg(addr, reg, val) writereg(addr, (readreg(addr) & ~reg) | (val & reg))

#define D1GRPH_ENABLE 0x6100
    #define D1GRPH_ENABLE_REG 0x1
#define D1GRPH_CONTROL 0x6104
    #define D1GRPH_DEPTH 0x3
        #define D1GRPH_DEPTH_8BPP 0x0
        #define D1GRPH_DEPTH_16BPP 0x1
        #define D1GRPH_DEPTH_32BPP 0x2
        #define D1GRPH_DEPTH_64BPP 0x3
    #define D1GRPH_FORMAT 0x700
        #define D1GRPH_FORMAT_8BPP_INDEXED 0x000
        #define D1GRPH_FORMAT_16BPP_ARGB1555 0x000
        #define D1GRPH_FORMAT_16BPP_RGB565 0x100
        #define D1GRPH_FORMAT_16BPP_ARGB4444 0x200
        //todo: 16bpp alpha index 88, mono 16, brga 5551
        #define D1GRPH_FORMAT_32BPP_ARGB8888 0x000
        #define D1GRPH_FORMAT_32BPP_ARGB2101010 0x100
        #define D1GRPH_FORMAT_32BPP_DIGITAL 0x200
        #define D1GRPH_FORMAT_32BPP_8ARGB2101010 0x300
        #define D1GRPH_FORMAT_32BPP_BGRA1010102 0x400
        #define D1GRPH_FORMAT_32BPP_8BGRA1010102 0x500
        #define D1GRPH_FORMAT_32BPP_RGB111110 0x600
        #define D1GRPH_FORMAT_32BPP_BGR101111 0x700
        //todo: 64bpp
    #define D1GRPH_ADDRESS_TRANSLATION 0x10000
        #define D1GRPH_ADDRESS_TRANSLATION_PHYS 0x00000
        #define D1GRPH_ADDRESS_TRANSLATION_VIRT 0x10000
    #define D1GRPH_PRIVILEGED_ACCESS 0x20000
        #define D1GRPH_PRIVILEGED_ACCESS_DISABLE 0x00000
        #define D1GRPH_PRIVILEGED_ACCESS_ENABLE 0x20000
    #define D1GRPH_ARRAY_MODE 0xF00000
        #define D1GRPH_ARRAY_LINEAR_GENERAL 0x000000
        #define D1GRPH_ARRAY_LINEAR_ALIGNED 0x100000
        #define D1GRPH_ARRAY_1D_TILES_THIN1 0x200000
        //todo: rest of these array modes
		
#define D1GRPH_SWAP_CNTL 0x610C
	#define D1GRPH_ENDIAN_SWAP 0x3
		#define D1GRPH_ENDIAN_SWAP_NONE 0x0
		#define D1GRPH_ENDIAN_SWAP_16 0x1
		#define D1GRPH_ENDIAN_SWAP_32 0x2
		#define D1GRPH_ENDIAN_SWAP_64 0x3
	#define D1GRPH_RED_CROSSBAR 0x30
		#define D1GRPH_RED_CROSSBAR_RED 0x00
	#define D1GRPH_GREEN_CROSSBAR 0xC0
		#define D1GRPH_GREEN_CROSSBAR_GREEN 0x00
	#define D1GRPH_BLUE_CROSSBAR 0x300
		#define D1GRPH_BLUE_CROSSBAR_BLUE 0x000
	#define D1GRPH_ALPHA_CROSSBAR 0xC00
		#define D1GRPH_ALPHA_CROSSBAR_ALPHA 0x000
	// todo: other values for crossbars

#define D1GRPH_PRIMARY_SURFACE_ADDRESS 0x6110
    #define D1GRPH_PRIMARY_DFQ_ENABLE 0x1
        #define D1GRPH_PRIMARY_DFQ_OFF 0x0
        #define D1GRPH_PRIMARY_DFQ_ON 0x1
    #define D1GRPH_PRIMARY_SURFACE_ADDR 0xFFFFFF00

#define D1GRPH_PITCH 0x6120
    #define D1GRPH_PITCH_VAL 0x3FFF
