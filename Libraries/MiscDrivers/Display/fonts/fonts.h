/******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

/**
 * @file fonts.h
 * @brief Header file for font definitions.
 *
 * This header file defines fonts for various sizes of the Liberation Sans font family.
 * Fonts are included as arrays of unsigned characters and can be accessed using the `font_table` macro,
 * or by accessing the character arrays directly.
 */

#ifndef LIBRARIES_MISCDRIVERS_DISPLAY_FONTS_FONTS_H_
#define LIBRARIES_MISCDRIVERS_DISPLAY_FONTS_FONTS_H_

#ifdef FONT_LiberationSans12x12
extern const unsigned char Liberation_Sans12x12[];
#ifndef _font_char_table
#define _font_char_table Liberation_Sans12x12
#endif
#endif

#ifdef FONT_LiberationSans16x16
extern const unsigned char Liberation_Sans16x16[];
#ifndef _font_char_table
#define _font_char_table Liberation_Sans16x16
#endif
#endif

#ifdef FONT_LiberationSans19x19
extern const unsigned char Liberation_Sans19x19[];
#ifndef _font_char_table
#define _font_char_table Liberation_Sans19x19
#endif
#endif

#ifdef FONT_LiberationSans24x24
extern const unsigned char Liberation_Sans24x24[];
#ifndef _font_char_table
#define _font_char_table Liberation_Sans24x24
#endif
#endif

#ifdef FONT_LiberationSans28x28
extern const unsigned char Liberation_Sans28x28[];
#ifndef _font_char_table
#define _font_char_table Liberation_Sans28x28
#endif
#endif

/**
 * @brief font table macro that can be used to access the selected font.  When multiple
 * fonts are enabled, the font table will point to the first font in the list.
 * 
 */
#define font_char_table _font_char_table

#endif // LIBRARIES_MISCDRIVERS_DISPLAY_FONTS_FONTS_H_
