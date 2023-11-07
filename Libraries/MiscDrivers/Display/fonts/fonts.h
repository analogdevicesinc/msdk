
/**
 * @file fonts.h
 * @brief Header file for font definitions.
 *
 * This header file defines fonts for various sizes of the Liberation Sans font family.
 * Fonts are included as arrays of unsigned characters and can be accessed using the `font_table` macro,
 * or by accessing the character arrays directly.
 */

#ifndef FONTS_H
#define FONTS_H

#ifdef FONT_LiberationSans12x12
extern const unsigned char Liberation_Sans12x12[];
#ifndef _font_table
#define _font_table Liberation_Sans12x12
#endif
#endif

#ifdef FONT_LiberationSans16x16
extern const unsigned char Liberation_Sans16x16[];
#ifndef _font_table
#define _font_table Liberation_Sans16x16
#endif
#endif

#ifdef FONT_LiberationSans19x19
extern const unsigned char Liberation_Sans19x19[];
#ifndef _font_table
#define _font_table Liberation_Sans19x19
#endif
#endif

#ifdef FONT_LiberationSans24x24
extern const unsigned char Liberation_Sans24x24[];
#ifndef _font_table
#define _font_table Liberation_Sans24x24
#endif
#endif

#ifdef FONT_LiberationSans28x28
extern const unsigned char Liberation_Sans28x28[];
#ifndef _font_table
#define _font_table Liberation_Sans28x28
#endif
#endif

/**
 * @brief font table macro that can be used to access the selected font.  When multiple
 * fonts are enabled, the font table will point to the first font in the list.
 */
#define font_table _font_table

#endif

