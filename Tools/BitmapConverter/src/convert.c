const char *copy_right = "\n"
"/******************************************************************************* \n"
"* Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.		 \n"
"*																				 \n"
"* This software is protected by copyright laws of the United States and		 \n"
"* of foreign countries. This material may also be protected by patent laws		 \n"
"* and technology transfer regulations of the United States and of foreign		 \n"
"* countries. This software is furnished under a license agreement and/or a		 \n"
"* nondisclosure agreement and may only be used or reproduced in accordance		 \n"
"* with the terms of those agreements. Dissemination of this information to		 \n"
"* any party or parties not specified in the license agreement and/or			 \n"
"* nondisclosure agreement is expressly prohibited.								 \n"
"*																				 \n"
"* The above copyright notice and this permission notice shall be included		 \n"
"* in all copies or substantial portions of the Software.						 \n"
"*																				 \n"
"* THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY KIND, EXPRESS	 \n"
"* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF					 \n"
"* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.		 \n"
"* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES			 \n"
"* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,		 \n"
"* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR		 \n"
"* OTHER DEALINGS IN THE SOFTWARE.												 \n"
"*																				 \n"
"* Except as contained in this notice, the name of Maxim Integrated				 \n"
"* Products, Inc. shall not be used except as stated in the Maxim Integrated	 \n"
"* Products, Inc. Branding Policy.												 \n"
"*																				 \n"
"* The mere transfer of this software does not imply any licenses				 \n"
"* of trade secrets, proprietary technology, copyrights, patents,				 \n"
"* trademarks, maskwork rights, or any other form of intellectual				 \n"
"* property whatsoever. Maxim Integrated Products, Inc. retains all				 \n"
"* ownership rights.															 \n"
"******************************************************************************* \n"
"*/																			     \n\n";

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <libgen.h>
#include <sys/stat.h>
#include <stdarg.h>
#include <errno.h>

#define APP_VERSION		"v2.0.0"

#define MAX_BITMAP_FILE		255
#define MAX_NB_CHAR     	255
#define NB_FONTS			64

#define ALL_IMGS_FILE	 	 "all_imgs.c" // Maxim Picture Information
#define DIR_BMP_RLE			 "bmp_rle"
#define DIR_BMP_SLIDES		 "bmp_slides"
#define DIR_FONTS			 "fonts"
#define FILE_IDS	 	 	 "bitmap.h"

#pragma pack (1)

typedef struct {
   unsigned short type;                 /* Magic identifier            */
   unsigned int   size;                       /* File size in bytes          */
   unsigned short reserved1, reserved2;
   unsigned int   offset;                     /* Offset to image data, bytes */
} Header_t;

typedef struct {
   unsigned int   size;               /* Header size in bytes      */
   int            width,height;                /* Width and height of image */
   unsigned short planes;       /* Number of colour planes   */
   unsigned short bits;         /* Bits per pixel            */
   unsigned int   compression;        /* Compression type          */
   unsigned int   imagesize;          /* Image size in bytes       */
   int            xresolution,yresolution;     /* Pixels per meter          */
   unsigned int   ncolors;           /* Number of colours         */
   unsigned int   importantcolors;   /* Important colours         */
} InfoHeader_t;

typedef struct {
  unsigned char R,G,B,Alpha;
} RGBA8_t;

typedef struct
{
       unsigned short x;
       unsigned char  w;
       unsigned char  hex_code;// hex code of char
} font_char_t;

typedef struct
{
       //unsigned char size;
       unsigned char nb_char;
       unsigned char bitmap_id;
       font_char_t	 char_info[MAX_NB_CHAR];
} font_info_t;

typedef struct
{
	unsigned int  w;
	unsigned int  h;
	unsigned char id_palatte;
	unsigned char rle;
	unsigned int  data_size;
    unsigned char *data;
} bitmap_info_t;

typedef struct {
	//unsigned short 	type;                        /* Magic identifier               */
	//unsigned short 	reserved1, reserved2;
	//unsigned short 	bits;                        /* Bits per pixel                 */
	//
	unsigned int	offset2info_palatte;
 	unsigned int	offset2info_font;
 	unsigned int	offset2info_bitmap;
 	//
	unsigned int 	nb_palette;		// number of palette
	unsigned int 	nb_font;		// number of fonts
	unsigned int 	nb_bitmap;		// number of bitmaps
} Header_images_t;

typedef struct
{
       unsigned char nb_palette;
       unsigned int  offset_palettex[MAX_BITMAP_FILE];
       RGBA8_t       palettex[MAX_BITMAP_FILE][256];
} palette_header_t;

typedef struct
{
       unsigned char nb_font;
       unsigned int  offset_fontx[NB_FONTS];
       font_info_t   fontx[NB_FONTS];
} font_header_t;

typedef struct
{
       unsigned char nb_bitmap;
       unsigned int  offset_bitmapx[MAX_BITMAP_FILE];
       bitmap_info_t bitmap[MAX_BITMAP_FILE];
} bitmap_header_t;

#pragma pack()

static Header_images_t 	images_header;
static palette_header_t palette_header;
static font_header_t	font_header;
static bitmap_header_t	bitmap_header;

static unsigned char *image = NULL;
static RGBA8_t       *bmp_palette = NULL;
static Header_t      Header;
static InfoHeader_t  InfoHeader;

static char bitmap_id_file[512];

static int g_debug_status = 1;
static int debug_printf(const char *format, ...)
{
    if (g_debug_status ) {
    	int 	ret;
        char 	msg[256] = {0};
        va_list argptr;

    	va_start(argptr, format);
		ret = vsnprintf(msg, sizeof(msg), format, argptr);
		va_end(argptr);

		if (ret > 0) {
	    	printf("%s", msg);
		}
    }

    return 0;
}

static int isFileExist(char *fileName)
{
	FILE *fd;

	fd= fopen(fileName, "r");
	if (fd == NULL) {
		return -1;
	}
	fclose(fd);

	return 0;
}

static int append_str(char *fileName, const char *str)
{
	FILE *fd;

	fd = fopen(fileName, "a");
	if (fd == NULL) {
		printf("%s File open error\n", fileName);
		return -1;
	}

	fwrite(str, strlen(str), 1, fd);
	fclose(fd);

	return 0;
}

static int createFile(char *fileName)
{
	FILE *fd;

	fd = fopen(fileName, "w");
	if (fd == NULL) {
		printf("File create error\n");
		return -1;
	}
	fclose(fd);

	return 0;
}

static int is_palette_exist(RGBA8_t *palette)
{
	int i;
	int isEqual = 0;

	for (i = 0; i < palette_header.nb_palette; i++) {
			isEqual = memcmp(palette_header.palettex[i], palette, sizeof(RGBA8_t)*256);
			if (isEqual == 0) {
				return i;
			}
	}

	return -1;
}

static int LoadBMP(char *filename, int first)
{
	FILE *BMPFile;
	int NbColor;
	// unsigned char *bmp_tmp = NULL;
	// int i,h,w;

	/* This checks for the file */
	BMPFile = fopen(filename, "rb");
	if (BMPFile == NULL) {
		printf("Cant open file: errno:%d\n", errno);
		return(1);
	}

	/* Read the header */
	fread(&Header, sizeof(Header_t), 1, BMPFile);

	// debug_printf("Header:\n\ttype: %04x\n\treserved1: %d\n\treserved2: %d\n\toffset : %d\n"
	//     ,Header.type,Header.reserved1,Header.reserved2, Header.offset);

	if (Header.type != 0x4d42 || Header.reserved1 != 0 || Header.reserved2 != 0 ) {
		/* Not a valid bitmap file - don't display */
		printf("Not a valid bitmap.\n");
		fclose(BMPFile);
		return(1);
	}

	/* Read the infoheader */
	fread(&InfoHeader, sizeof(InfoHeader_t), 1, BMPFile);

	if (InfoHeader.bits != 8) {
		/*If the file is other than 8-bit dont read.*/
		printf("Not an 8-bit bitmap.\n");
		fclose(BMPFile);
		return(1);
	}

//  debug_printf("\nInfoHeader:\n\twidth: %d\n\theight: %d\n\tnb_plane: %d\n\tbits per pixel: %d\n\tcompression_type: %d\n\timagesize: %d\n\tncolors: %d\n",
//          InfoHeader.width,InfoHeader.height,InfoHeader.planes,InfoHeader.bits,
//          InfoHeader.compression, InfoHeader.imagesize, InfoHeader.ncolors);

	if (InfoHeader.height < 0) {
		printf("BMP Reverse not supported\n");
		return(1);
	}

	if (first) {
		if (0 == InfoHeader.ncolors) {
			NbColor = (Header.offset - (sizeof(Header_t) + sizeof(InfoHeader_t))) / sizeof(RGBA8_t);
			// debug_printf("\nNb colo calculate: %d\n", NbColor);
			InfoHeader.ncolors = NbColor;
		}

		if (InfoHeader.ncolors) {
			bmp_palette = (RGBA8_t *)malloc(InfoHeader.ncolors * sizeof(RGBA8_t));
			fread((unsigned char *)bmp_palette, sizeof(RGBA8_t), InfoHeader.ncolors, BMPFile);
		}
	}

	fseek(BMPFile, Header.offset, SEEK_SET);

	image = (unsigned char *)malloc(InfoHeader.imagesize);
	fread(image, InfoHeader.imagesize, sizeof(unsigned char), BMPFile);


  /* print for verification purpose */
//  debug_printf("Header\n");
//  debug_printf("\ttype: %04x\n\treserved1: %d\n\treserved2: %d\n\toffset : %d\n\tfile size: %d\n"
//      ,Header.type, Header.reserved1, Header.reserved2, Header.offset, Header.size);
//  debug_printf("Header_info\n");
//  debug_printf("\tHeader size: %d\n\twidth: %d\n\theight: %d\n\tnb_plane: %d\n\tbits per pixel: %d\n\tcompression_type: %d\n\timagesize: %d\n\tncolors: %d\n\txREsolution: %d\n",
//          InfoHeader.size, InfoHeader.width, InfoHeader.height, InfoHeader.planes, InfoHeader.bits,
//          InfoHeader.compression, InfoHeader.imagesize, InfoHeader.ncolors, InfoHeader.xresolution);
//  debug_printf("Palette\n");
//  debug_printf("\tPalette R : %d\n", bmp_palette->R);
//  debug_printf("\tPalette G : %d\n", bmp_palette->G);
//  debug_printf("\tPalette B : %d\n", bmp_palette->B);
//  debug_printf("\tPalette Alpha : %d\n", bmp_palette->Alpha);

	debug_printf("\n");
	/* end of print */
	fclose(BMPFile);

	return 0;
}

static int loadMFFInfo(char *filename, font_info_t *info)
{
	FILE *fd;
	unsigned int tmp;

	fd = fopen(filename, "rb");
	if (fd == NULL) {
		printf("Cant open file: errno:%d\n", errno);
		return 1;
	}

	fseek(fd, 1, SEEK_SET);
	fread(&info->nb_char, 		sizeof(unsigned char), 	  			 1, fd);
	//fread(&info->bitmap_offset, sizeof(unsigned int), 	  		 1, fd);
	fread(&tmp, sizeof(unsigned int), 	 	1, fd);// pass bitmap offset
	fread(info->char_info, 		sizeof(font_char_t), info->nb_char, fd);

	fclose(fd);

	return 0;
}

static int writeFileId(char *fileName, char *def, int id)
{
	FILE 		*fd;
	char 		newName[128] = {0};
	char 		*ptr = newName;
	char 		byt;

	while (*def != '\0') {
		byt = *def++;
		if (  ( (byt >= 0x20) && (byt <= 0x2F))   ||
			  ( (byt >= 0x3A) && (byt <= 0x40))   ||
			  ( (byt >= 0x5B) && (byt <= 0x60))   ||
			  ( (byt >= 0x7B) && (byt <= 0xFF))
		) {
			byt = '_'; // replace special char with underscore
		}

		*ptr++ = byt;
	}

	fd= fopen(fileName, "a");
	//fseek(fd, 0, SEEK_END);
	if (fd != NULL) {
		fprintf(fd, "#define    %-50s %d\n", newName, id);
		fclose(fd);
	}

	return 0;
}

static int load_bitmaps(char *input_folder)
{
	int 			ret;
	DIR 			*dir;
	int 			cnt_imgs = 0, cmp = 0;
	struct dirent 	*ent;
	int 			offset;
	int    			id_palette;
	int				d_namlen;

	cnt_imgs 	= bitmap_header.nb_bitmap;

	/* Open bitmaps and get all data */
	if ((dir = opendir (input_folder)) != NULL) {

		/* Get all the bmp files within the directory */
		while ((ent = readdir (dir)) != NULL) {
			d_namlen = strlen(ent->d_name);
			if(memcmp("bmp", ent->d_name+d_namlen-3, 3) == 0) {
				char file[512];

				sprintf(file, "%s%s", input_folder, ent->d_name);
				ret = LoadBMP(file, 1);
				if (ret != 0) {
					continue;
				}

				// write file id
				debug_printf ("%s id = %d.\n", ent->d_name, cnt_imgs);
				writeFileId(bitmap_id_file, ent->d_name, cnt_imgs);

				offset = 0;
				for(cmp = 0; cmp < InfoHeader.ncolors; cmp++) {
					offset = 4*cmp;
					palette_header.palettex[palette_header.nb_palette][cmp].R      = *(&bmp_palette->R + offset);
					palette_header.palettex[palette_header.nb_palette][cmp].G      = *(&bmp_palette->G + offset);
					palette_header.palettex[palette_header.nb_palette][cmp].B      = *(&bmp_palette->B + offset);
					palette_header.palettex[palette_header.nb_palette][cmp].Alpha  = bmp_palette->Alpha;
				}
				free(bmp_palette);

				id_palette = is_palette_exist(palette_header.palettex[palette_header.nb_palette]);
				if ( id_palette == -1) {
					id_palette = palette_header.nb_palette;
					palette_header.nb_palette++;
				}

				/* Check if it's the first image, no need to copy everytime data into the new_header, always the same within all images */
				bitmap_header.bitmap[cnt_imgs].w 			= InfoHeader.width;
				bitmap_header.bitmap[cnt_imgs].h 			= InfoHeader.height;
				bitmap_header.bitmap[cnt_imgs].id_palatte 	= id_palette;
				bitmap_header.bitmap[cnt_imgs].rle			= InfoHeader.compression ? 1: 0;
				bitmap_header.bitmap[cnt_imgs].data			= image;
				bitmap_header.bitmap[cnt_imgs].data_size	= InfoHeader.imagesize;

				cnt_imgs++;
			}
		}
		closedir (dir);
	} else {
		/* could not open directory */
		printf ("Could not open %s\n", input_folder);
		return EXIT_FAILURE;
	}
	bitmap_header.nb_bitmap = cnt_imgs; //

	return 0;
}

static int load_fonts(char *input_folder)
{
	int				ret;
	DIR 			*dir;
	int 			cnt_imgs = 0, cmp = 0;
	struct dirent 	*ent;
	int 			offset;
	int    			id_palette;
	int				d_namlen;

	cnt_imgs 	= bitmap_header.nb_bitmap;

	/* Open bitmaps and get all data */
	if ((dir = opendir (input_folder)) != NULL) {

		/* Get all the bmp files within the directory */
		while ((ent = readdir (dir)) != NULL) {
			d_namlen = strlen(ent->d_name);
			if(memcmp("bmp", ent->d_name+d_namlen-3, 3) == 0) {
				char file[512];

				//debug_printf ("%s id = %d.\n", ent->d_name, cnt_imgs);

				sprintf(file, "%s%s", input_folder, ent->d_name);
				memcpy(&file[strlen(file)-3], "mff", 3); // replace file suffix
				if ( isFileExist(file) != 0) {
					continue; // file not exist
				}

				ret = loadMFFInfo(file, &font_header.fontx[font_header.nb_font]);
				if (ret != 0) {
					continue;
				}
				// set related font bitmap id
				font_header.fontx[font_header.nb_font].bitmap_id = cnt_imgs;

				sprintf(file, "%s%s", input_folder, ent->d_name);
				ret = LoadBMP(file, 1);
				if (ret != 0) {
					continue;
				}

				// do not support compression mode for font
				if (InfoHeader.compression != 0) {
					free(image);
					free(bmp_palette);
					continue;
				}

				// write file id
				sprintf(file, "%s", ent->d_name);
				file[d_namlen-4] = '\0'; // remove .bmp
				debug_printf ("%s id = %d.\n", file, font_header.nb_font);
				writeFileId(bitmap_id_file, file, font_header.nb_font);

				//
				offset = 0;
				for(cmp = 0; cmp < InfoHeader.ncolors; cmp++) {
					offset = 4*cmp;
					palette_header.palettex[palette_header.nb_palette][cmp].R      = *(&bmp_palette->R + offset);
					palette_header.palettex[palette_header.nb_palette][cmp].G      = *(&bmp_palette->G + offset);
					palette_header.palettex[palette_header.nb_palette][cmp].B      = *(&bmp_palette->B + offset);
					palette_header.palettex[palette_header.nb_palette][cmp].Alpha  = bmp_palette->Alpha;
				}
				free(bmp_palette);

				id_palette = is_palette_exist(palette_header.palettex[palette_header.nb_palette]);
				if ( id_palette == -1) {
					id_palette = palette_header.nb_palette;
					palette_header.nb_palette++;
				}

				/* Check if it's the first image, no need to copy everytime data into the new_header, always the same within all images */
				bitmap_header.bitmap[cnt_imgs].w 			= InfoHeader.width;
				bitmap_header.bitmap[cnt_imgs].h 			= InfoHeader.height;
				bitmap_header.bitmap[cnt_imgs].id_palatte 	= id_palette;
				bitmap_header.bitmap[cnt_imgs].rle			= 0;
				bitmap_header.bitmap[cnt_imgs].data			= image;
				bitmap_header.bitmap[cnt_imgs].data_size	= InfoHeader.imagesize;

				cnt_imgs++;
				font_header.nb_font++;
			}
		}
		closedir (dir);
	} else {
		printf ("Could not open %s\n", input_folder);
		return EXIT_FAILURE;
	}
	bitmap_header.nb_bitmap   = cnt_imgs; //

	return 0;
}

static void write_all_imgs_file(void *arr, unsigned int arrLen, FILE *file)
{
	static char buffer[10*1024];
	char *ptr;
	unsigned int  i;
	//
	unsigned int loop_counter;
	unsigned char *src = (unsigned char *)arr;

	while (arrLen > 0) {
		ptr 		 = buffer;
		loop_counter = (arrLen > 1024)? 1024: arrLen;

		for (i = 1; i <= loop_counter; i++) {

			if ( (i%16) == 0 ) {
				*ptr++ = '\n';
			}

			sprintf(ptr, "0x%02X,", *src++);
			ptr += 5; //
		}
		*ptr++ = '\n';

		fwrite(buffer, (unsigned int)(ptr-buffer), 1, file);
		arrLen -= loop_counter;
	}
}

static int create_all_imgs_file(char *out_file)
{
	int ret = 0;
	unsigned int i;

	/* Update images header  */
	images_header.nb_palette = palette_header.nb_palette;
	images_header.nb_font 	 = font_header.nb_font;
	images_header.nb_bitmap  = bitmap_header.nb_bitmap;

	images_header.offset2info_palatte = sizeof(images_header);
	images_header.offset2info_font = images_header.offset2info_palatte 						+ \
									 sizeof(unsigned char)									+ \
									 palette_header.nb_palette * 4 /* offset_palettex */ 	+ \
									 palette_header.nb_palette * sizeof(RGBA8_t) * 256	;

	images_header.offset2info_bitmap =	images_header.offset2info_font 						+ \
									 	sizeof(unsigned char)								+ \
										font_header.nb_font * sizeof(unsigned int);

										for (i = 0; i < font_header.nb_font; i++) {
											images_header.offset2info_bitmap += 2 /* nb_char + bitmap_id */ + \
																			    font_header.fontx[i].nb_char * sizeof(font_char_t);
										}

	/* Update palette header  */
	palette_header.offset_palettex[0] = images_header.offset2info_palatte 	+\
										1 /* nb_font */ 					+\
										(palette_header.nb_palette * 4 /* offset_palettex */);
	for (i = 1; i < palette_header.nb_palette; i++) {
		palette_header.offset_palettex[i] = palette_header.offset_palettex[i-1] + sizeof(RGBA8_t) * 256;
	}

	/* Update font header  */
	font_header.offset_fontx[0] =   images_header.offset2info_font 	+\
									1 /* nb_font */ 				+\
									(font_header.nb_font * 4 /* offset_fontx */);
	for (i = 1; i < font_header.nb_font; i++) {
		font_header.offset_fontx[i] = 	font_header.offset_fontx[i-1] 						+ 	\
							    		2 /* nb_char, bitmap_id */ 							+	\
										font_header.fontx[i-1].nb_char * sizeof(font_char_t) ;
	}

	/* Update bitmap header  */
	bitmap_header.offset_bitmapx[0] =   images_header.offset2info_bitmap 		+\
										1 /* nb_bitmap */ 						+\
										(bitmap_header.nb_bitmap * 4 /* offset_bitmapx */);
	for (i = 1; i < bitmap_header.nb_bitmap; i++) {
		bitmap_header.offset_bitmapx[i] = 	bitmap_header.offset_bitmapx[i-1] 						+\
											(sizeof(bitmap_info_t)-sizeof(unsigned char *) /* - for data pointer */)		+\
											bitmap_header.bitmap[i-1].data_size;
	}

	/*
	 * write output file
	 */
#if 1
	FILE *file= fopen(out_file, "r+");
	if (file != NULL) {
		const char *str;

		// go to end of file
		fseek(file, 0, SEEK_END);

		// header
		str = "\n/*\n  Header\n */\n";
		fwrite(str, strlen(str), 1, file);
		write_all_imgs_file(&images_header, sizeof(images_header), file);

		// palette
		str = "\n/*\n  Palette\n */\n";
		fwrite(str, strlen(str), 1, file);
		write_all_imgs_file(&palette_header.nb_palette, 		sizeof(palette_header.nb_palette), 					file);
		write_all_imgs_file(palette_header.offset_palettex, 	sizeof(unsigned int)*palette_header.nb_palette, 	file);
		write_all_imgs_file(palette_header.palettex, 			sizeof(RGBA8_t) * 256 *palette_header.nb_palette,	file);

		str = "\n/*\n  Fonts\n */\n";
		fwrite(str, strlen(str), 1, file);
		write_all_imgs_file(&font_header.nb_font, 			sizeof(font_header.nb_font),						file);
		write_all_imgs_file(&font_header.offset_fontx, 		sizeof(unsigned int)*font_header.nb_font,			file);

		for(i = 0; i < font_header.nb_font; i++) {
			write_all_imgs_file(&font_header.fontx[i].nb_char, 	sizeof(unsigned char), 	file);
			write_all_imgs_file(&font_header.fontx[i].bitmap_id, 	sizeof(unsigned char), 	file);
			write_all_imgs_file(&font_header.fontx[i].char_info, 	sizeof(font_char_t) * font_header.fontx[i].nb_char, file);
		}

		// bitmap
		str = "\n/*\n  Bitmaps\n */\n";
		fwrite(str, strlen(str), 1, file);

		write_all_imgs_file(&bitmap_header.nb_bitmap, sizeof(bitmap_header.nb_bitmap),  file);
		write_all_imgs_file(&bitmap_header.offset_bitmapx, sizeof(unsigned int) * bitmap_header.nb_bitmap, file);

		for(i = 0; i < bitmap_header.nb_bitmap; i++) {
			write_all_imgs_file(&bitmap_header.bitmap[i].w,   		sizeof(unsigned int),   file);
			write_all_imgs_file(&bitmap_header.bitmap[i].h,   		sizeof(unsigned int),   file);
			write_all_imgs_file(&bitmap_header.bitmap[i].id_palatte,sizeof(unsigned char), 	file);
			write_all_imgs_file(&bitmap_header.bitmap[i].rle, 		sizeof(unsigned char),  file);
			write_all_imgs_file(&bitmap_header.bitmap[i].data_size, sizeof(unsigned int),   file);
			//
			write_all_imgs_file(bitmap_header.bitmap[i].data, bitmap_header.bitmap[i].data_size, file);

			free(bitmap_header.bitmap[i].data);
		}
		fclose(file);
	} else {
		printf ("%s %s open failed\n", __func__, ALL_IMGS_FILE);
		ret = -1;
	}
#else
	FILE *file= fopen(out_file, "r+b");
	if (file != NULL) {
		// header
		fwrite(&images_header, sizeof(images_header), 1, file);

		// palette
		fwrite(&palette_header.nb_palette, 		sizeof(palette_header.nb_palette), 	1, 							file);
		fwrite(palette_header.offset_palettex, 	sizeof(unsigned int), 				palette_header.nb_palette, 	file);
		fwrite(palette_header.palettex, 		sizeof(RGBA8_t)*256, 				palette_header.nb_palette, 	file);

		// font
		fwrite(&font_header.nb_font, sizeof(font_header.nb_font), 1, file);
		fwrite(&font_header.offset_fontx, sizeof(unsigned int), font_header.nb_font, file);
		for(i = 0; i < font_header.nb_font; i++) {
			fwrite(&font_header.fontx[i].nb_char, 	  	sizeof(unsigned char), 	1, 							  file);
			fwrite(&font_header.fontx[i].bitmap_id,   	sizeof(unsigned char), 	1, 							  file);
			fwrite(&font_header.fontx[i].char_info, 	sizeof(font_char_t), 	font_header.fontx[i].nb_char, file);
		}

		// bitmap
		fwrite(&bitmap_header.nb_bitmap, sizeof(bitmap_header.nb_bitmap), 1, file);
		fwrite(&bitmap_header.offset_bitmapx, sizeof(unsigned int), bitmap_header.nb_bitmap, file);

		for(i = 0; i < bitmap_header.nb_bitmap; i++) {
			fwrite(&bitmap_header.bitmap[i].w,   		sizeof(unsigned int),  1, file);
			fwrite(&bitmap_header.bitmap[i].h,   		sizeof(unsigned int),  1, file);
			fwrite(&bitmap_header.bitmap[i].id_palatte, sizeof(unsigned char), 1, file);
			fwrite(&bitmap_header.bitmap[i].rle, 		sizeof(unsigned char), 1, file);
			fwrite(&bitmap_header.bitmap[i].data_size, 	sizeof(unsigned int),  1, file);
			//
			fwrite(bitmap_header.bitmap[i].data, bitmap_header.bitmap[i].data_size, 1, file);

			free(bitmap_header.bitmap[i].data);
		}
		fclose(file);
	} else {
		printf ("%s %s open failed\n", __func__, ALL_IMGS_FILE);
		ret = -1;
	}
#endif
	return ret;
}

static void reset_globals(void)
{
	memset(&images_header, 0, sizeof(images_header));
	palette_header.nb_palette 	= 0;
	font_header.nb_font 		= 0;
	bitmap_header.nb_bitmap		= 0;
}

#define TEST_MODE	0

int main(int argc, char **argv)
{
	char 	buffer[512] = {0};
	char 	*path;

	printf("MAXIM bitmap converter version:%s\n", APP_VERSION);

#if TEST_MODE == 0
	if (argc != 2) {
		printf("Usage Error:\n");
		printf("Format: ./app bitmap_path \n");
		return -1;
	}
	//
	path = argv[1];
	g_debug_status = 0;
#else
	// get source path
	path = "ext_tools/resources";
	g_debug_status = 1;
#endif

	reset_globals();

	//
	sprintf(bitmap_id_file, "%s/%s", path, FILE_IDS);
	createFile(bitmap_id_file);
	append_str(bitmap_id_file, copy_right);
	// inc guard
	append_str(bitmap_id_file, "\n#ifndef _BITMAP_H_\n");
	append_str(bitmap_id_file, "#define _BITMAP_H_\n");

	// load bitmaps
	append_str(bitmap_id_file, "\n// bitmaps id\n");
	sprintf(buffer, "%s/%s/", path, DIR_BMP_RLE);
	load_bitmaps(buffer);

	sprintf(buffer, "%s/%s/", path, DIR_BMP_SLIDES);
	load_bitmaps(buffer);

	// load fonts
	append_str(bitmap_id_file, "\n// fonts id\n");
	sprintf(buffer, "%s/%s/", path, DIR_FONTS);
	load_fonts(buffer);

	// end of include guard bitmap id files
	append_str(bitmap_id_file, "\n\n#endif //_BITMAP_H_\n");


	/*
	 * prepare all images file
	 */
	sprintf(buffer, "%s/%s", path, ALL_IMGS_FILE);
	createFile(buffer);
	append_str(buffer, copy_right);

	/*
	 * write array name and attribute
	 */
	append_str(buffer, "\n\n__attribute__ ((section(\".bin_storage_img\"))) __attribute__ ((__used__)) \n");
	append_str(buffer, "const unsigned char imgs_arr[ ] = { \n");
	// write array data
	create_all_imgs_file(buffer);
	// end of array
	append_str(buffer, "\n};");

	return 0;
}
