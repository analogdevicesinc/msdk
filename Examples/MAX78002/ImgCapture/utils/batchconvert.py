"""
/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*
******************************************************************************/
"""

from pathlib import Path
import re
from imgConverter import convert
import traceback
import argparse

def batchconvert(input_dir, output_dir = "converted"):
    if not Path(output_dir).exists():
        Path(output_dir).mkdir()

    for file in sorted(Path(input_dir).iterdir()):
        with open(file, "rb") as f:
            header = f.readline() # First line in a valid image should contain the image header
            try:
                header = header.decode(encoding="ascii").strip()
                if "*IMG*" not in header:
                    print(f"Bad image header in {file}, skipping")
                else:

                    expr = re.compile("\*IMG\* (\w+) (\d+) (\d+) (\d+)")
                    match = expr.findall(header)

                    if len(match) == 1:
                        # Received expected header, parse parameters from regex
                        values = match[0]
                        pixel_format = values[0]
                        imglen = int(values[1])
                        w = int(values[2])
                        h = int(values[3])

                        content = f.read() # Read the rest of the file content

                        out_file = f"{Path(output_dir).joinpath(file.name)}.png"
                        convert(content, out_file, w, h, pixel_format)
                        print(f"Successfully converted {file} -> {out_file}")
            except:
                print(f"Error converting {file}, skipping...")
                print(traceback.format_exc())

argparser = argparse.ArgumentParser(description="Utility conversion script for converting batches of raw images captured by ImgCapture firmware into .pngs")
argparser.add_argument("input_dir", type=str, help="Input directory containing the raw images to convert.")
argparser.add_argument("-o", type=str, help="Output directory to save the converted images.", default="converted")

if __name__ == "__main__":
    args = argparser.parse_args()
    batchconvert(args.input_dir, args.o)