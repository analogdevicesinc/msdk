
#if defined __ICCARM__ 
#pragma diag_suppress=Pe188
#endif

#define SWAP(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) | (((x) & 0x0000FF00) << 8) | (((x) & 0x000000FF) << 24))


typedef enum
{
	ROM_A1_VERSION = 0x01000000,
} enum_rom_version_t;

//#ifdef __SLA_FWK__
/***** Includes *****/
#include <stdio.h>
#include <stdint.h>

extern unsigned int Reset_Handler();


int i;
typedef enum
{
    MagicH = 0x48495357,
    MagicL = 0x45444744,
} enum_magic_t;

typedef struct
{
	enum_magic_t		MagicHigh;				//> SLA Header magic
	enum_magic_t		MagicLow;				//> SLA Header magic
} magic_t;

#ifdef __ICCARM__
typedef struct
{
	magic_t			Magic;
	enum_rom_version_t	RomVersion;           //> ROM version
	unsigned int	LoadAddr;		      //> Relocation address.
	unsigned int	SLA_CodeSize_;  	      //> SLA code size in bytes
	unsigned int	JumpAddr; 		      //> Rom code will jump at this address
	unsigned int 	ArgSize;  		      //> Size of the Argument
	unsigned int 	AppVersionNumber;  	      //> Version of this application
} flash_app_header_t;

#else
typedef struct
{
	magic_t			Magic;
	enum_rom_version_t	RomVersion;				//> ROM version
	unsigned int	LoadAddr;				//> Relocation address.
	unsigned int	SLA_CodeSize;  			//> SLA code size in bytes
	unsigned int*	JumpAddr; 				//> Rom code will jump at this address
	unsigned int 	ArgSize;  				//> Size of the Argument
	unsigned int 	AppVersionNumber;  		//> Version of this application
} flash_app_header_t;
#endif


#ifdef __ICCARM__
typedef struct
{
	unsigned int    unused_sla[120];        //> 512 bytes alligned.  i.e 128 int, 128-7used=121,
} flash_app_header_pad_t;

typedef struct
{
        unsigned int    endofcode[17];
} code_tail_t;

#endif

 


// VERY IMPORTANT:- If more item is added to the .sb_sla_header, then more item
// trimmed out of the .sb_sla_header_padding below and vice versa
//
#ifdef __ICCARM__
#pragma location=".sb_sla_header"  
__root const flash_app_header_t sb_header = 
#else
__attribute__ ((section(".sb_sla_header"))) __attribute__ ((__used__))
const flash_app_header_t sb_header =
#endif
{
		.Magic 				= {
				.MagicHigh			= SWAP(MagicH),
				.MagicLow			= SWAP(MagicL),
		},

		.RomVersion		= SWAP(ROM_A1_VERSION), // ROM_A1_VERSION, 
		.LoadAddr     		= SWAP(0x10000000),   // 0x10000000, 
#ifdef __ICCARM__  
              .SLA_CodeSize_ 		= SWAP(0xabcdabcd),  // Trick to get constant defined at link time
		.JumpAddr   		= SWAP(0x100003ed),  // Reset_Handler from *.map file, Hard code for now.
#else
		.SLA_CodeSize 		= (unsigned int)&_SLA_Size_SWAP,  // Trick to get constant defined at link time
		.JumpAddr   		=  &_start_SWAP,  //(unsigned int *)Reset_Handler,
#endif
		.ArgSize  		= 0,
		.AppVersionNumber       = SWAP(0x01000000), // 0xAABBCCCC for version AA.BB.CCCC
 };



#ifdef __ICCARM__
#pragma location=".sb_sla_header_padding" 
__root const flash_app_header_pad_t padding =
{
        .unused_sla[0]      = 0xFFFFFFFF,
        .unused_sla[1]      = 0xFFFFFFFF,	
        .unused_sla[2]      = 0xFFFFFFFF,
        .unused_sla[3]      = 0xFFFFFFFF,    
        .unused_sla[4]      = 0xFFFFFFFF,
        .unused_sla[5]      = 0xFFFFFFFF,	
        .unused_sla[6]      = 0xFFFFFFFF,
        .unused_sla[7]      = 0xFFFFFFFF,
        .unused_sla[8]      = 0xFFFFFFFF,
        .unused_sla[9]      = 0xFFFFFFFF,	
        .unused_sla[10]      = 0xFFFFFFFF,
        .unused_sla[11]      = 0xFFFFFFFF,
        .unused_sla[12]      = 0xFFFFFFFF,
        .unused_sla[13]      = 0xFFFFFFFF,	
        .unused_sla[14]      = 0xFFFFFFFF,
        .unused_sla[15]      = 0xFFFFFFFF,
        .unused_sla[16]      = 0xFFFFFFFF,
        .unused_sla[17]      = 0xFFFFFFFF,	
        .unused_sla[18]      = 0xFFFFFFFF,
        .unused_sla[19]      = 0xFFFFFFFF,
        .unused_sla[20]      = 0xFFFFFFFF,
        .unused_sla[21]      = 0xFFFFFFFF,	
        .unused_sla[22]      = 0xFFFFFFFF,
        .unused_sla[23]      = 0xFFFFFFFF,
        .unused_sla[24]      = 0xFFFFFFFF,
        .unused_sla[25]      = 0xFFFFFFFF,	
        .unused_sla[26]      = 0xFFFFFFFF,
        .unused_sla[27]      = 0xFFFFFFFF,
        .unused_sla[28]      = 0xFFFFFFFF,
        .unused_sla[29]      = 0xFFFFFFFF,	
        .unused_sla[30]      = 0xFFFFFFFF,
        .unused_sla[31]      = 0xFFFFFFFF,	
        .unused_sla[32]      = 0xFFFFFFFF,
        .unused_sla[33]      = 0xFFFFFFFF,
        .unused_sla[34]      = 0xFFFFFFFF,
        .unused_sla[35]      = 0xFFFFFFFF,	
        .unused_sla[36]      = 0xFFFFFFFF,
        .unused_sla[37]      = 0xFFFFFFFF,
        .unused_sla[38]      = 0xFFFFFFFF,
        .unused_sla[39]      = 0xFFFFFFFF,	
        .unused_sla[40]      = 0xFFFFFFFF,
        .unused_sla[41]      = 0xFFFFFFFF,	
        .unused_sla[42]      = 0xFFFFFFFF,
        .unused_sla[43]      = 0xFFFFFFFF,
        .unused_sla[44]      = 0xFFFFFFFF,
        .unused_sla[45]      = 0xFFFFFFFF,	
        .unused_sla[46]      = 0xFFFFFFFF,
        .unused_sla[47]      = 0xFFFFFFFF,
        .unused_sla[48]      = 0xFFFFFFFF,
        .unused_sla[49]      = 0xFFFFFFFF,	
        .unused_sla[50]      = 0xFFFFFFFF,	
        .unused_sla[51]      = 0xFFFFFFFF,	
        .unused_sla[52]      = 0xFFFFFFFF,
        .unused_sla[53]      = 0xFFFFFFFF,
        .unused_sla[54]      = 0xFFFFFFFF,
        .unused_sla[55]      = 0xFFFFFFFF,	
        .unused_sla[56]      = 0xFFFFFFFF,
        .unused_sla[57]      = 0xFFFFFFFF,
        .unused_sla[58]      = 0xFFFFFFFF,
        .unused_sla[59]      = 0xFFFFFFFF,	
        .unused_sla[60]      = 0xFFFFFFFF,
        .unused_sla[61]      = 0xFFFFFFFF,	
        .unused_sla[62]      = 0xFFFFFFFF,
        .unused_sla[63]      = 0xFFFFFFFF,
        .unused_sla[64]      = 0xFFFFFFFF,
        .unused_sla[65]      = 0xFFFFFFFF,	
        .unused_sla[66]      = 0xFFFFFFFF,
        .unused_sla[67]      = 0xFFFFFFFF,
        .unused_sla[68]      = 0xFFFFFFFF,
        .unused_sla[69]      = 0xFFFFFFFF,	
        .unused_sla[70]      = 0xFFFFFFFF,
        .unused_sla[71]      = 0xFFFFFFFF,	
        .unused_sla[72]      = 0xFFFFFFFF,
        .unused_sla[73]      = 0xFFFFFFFF,
        .unused_sla[74]      = 0xFFFFFFFF,
        .unused_sla[75]      = 0xFFFFFFFF,	
        .unused_sla[76]      = 0xFFFFFFFF,
        .unused_sla[77]      = 0xFFFFFFFF,
        .unused_sla[78]      = 0xFFFFFFFF,
        .unused_sla[79]      = 0xFFFFFFFF,		
        .unused_sla[80]      = 0xFFFFFFFF,
        .unused_sla[81]      = 0xFFFFFFFF,	
        .unused_sla[82]      = 0xFFFFFFFF,
        .unused_sla[83]      = 0xFFFFFFFF,
        .unused_sla[84]      = 0xFFFFFFFF,
        .unused_sla[85]      = 0xFFFFFFFF,	
        .unused_sla[86]      = 0xFFFFFFFF,
        .unused_sla[87]      = 0xFFFFFFFF,
        .unused_sla[88]      = 0xFFFFFFFF,
        .unused_sla[89]      = 0xFFFFFFFF,	
        .unused_sla[80]      = 0xFFFFFFFF,
        .unused_sla[81]      = 0xFFFFFFFF,	
        .unused_sla[82]      = 0xFFFFFFFF,
        .unused_sla[83]      = 0xFFFFFFFF,
        .unused_sla[84]      = 0xFFFFFFFF,
        .unused_sla[85]      = 0xFFFFFFFF,	
        .unused_sla[86]      = 0xFFFFFFFF,
        .unused_sla[87]      = 0xFFFFFFFF,
        .unused_sla[88]      = 0xFFFFFFFF,
        .unused_sla[89]      = 0xFFFFFFFF,	
        .unused_sla[90]      = 0xFFFFFFFF,
        .unused_sla[91]      = 0xFFFFFFFF,	
        .unused_sla[92]      = 0xFFFFFFFF,
        .unused_sla[93]      = 0xFFFFFFFF,
        .unused_sla[94]      = 0xFFFFFFFF,
        .unused_sla[95]      = 0xFFFFFFFF,	
        .unused_sla[96]      = 0xFFFFFFFF,
        .unused_sla[97]      = 0xFFFFFFFF,
        .unused_sla[98]      = 0xFFFFFFFF,
        .unused_sla[99]      = 0xFFFFFFFF,
        .unused_sla[100]      = 0xFFFFFFFF,
        .unused_sla[101]      = 0xFFFFFFFF,	
        .unused_sla[102]      = 0xFFFFFFFF,
        .unused_sla[103]      = 0xFFFFFFFF,
        .unused_sla[104]      = 0xFFFFFFFF,
        .unused_sla[105]      = 0xFFFFFFFF,	
        .unused_sla[106]      = 0xFFFFFFFF,
        .unused_sla[107]      = 0xFFFFFFFF,
        .unused_sla[108]      = 0xFFFFFFFF,
        .unused_sla[109]      = 0xFFFFFFFF,	
        .unused_sla[110]      = 0xFFFFFFFF,
        .unused_sla[111]      = 0xFFFFFFFF,	
        .unused_sla[112]      = 0xFFFFFFFF,
        .unused_sla[113]      = 0xFFFFFFFF,
        .unused_sla[114]      = 0xFFFFFFFF,
        .unused_sla[115]      = 0xFFFFFFFF,	
        .unused_sla[116]      = 0xFFFFFFFF,
        .unused_sla[117]      = 0xFFFFFFFF,
        .unused_sla[118]      = 0xFFFFFFFF,
        .unused_sla[119]      = 0xFFFFFFFF,
};

//#pragma location=".sb_sla_trailer"
//__root const unsigned int EndOfCode = 0xDEADC0DE; 

//#pragma location=".sig"
//__root const unsigned int Mark_This = 0xDEADBEEF; 

#pragma location=".sb_sla_trailer"
__root const code_tail_t end_of_code = 
{
        .endofcode[0] = 0xDEADC0DE, 
        .endofcode[1] = 0xFAFAFFFF,
        .endofcode[2] = 0xFFFFFFFF,
        .endofcode[3] = 0xFFFFFFFF,
        .endofcode[4] = 0xFFFFFFFF,
        .endofcode[5] = 0xFFFFFFFF,
        .endofcode[6] = 0xFFFFFFFF,
        .endofcode[7] = 0xFFFFFFFF,
        .endofcode[8] = 0xFFFFFFFF,
        .endofcode[9] = 0xFFFFFFFF,
        .endofcode[10] = 0xFFFFFFFF,
        .endofcode[11] = 0xFFFFFFFF,
        .endofcode[12] = 0xFFFFFFFF,
        .endofcode[13] = 0xFFFFFFFF,
        .endofcode[14] = 0xFFFFFFFF,
        .endofcode[15] = 0xFFFFFFFF,
        .endofcode[16] = 0xFFFFFFFF,    
};
#endif

