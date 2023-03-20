#! /usr/bin/env python3

################################################################################
# Copyright (C) 2020 Maxim Integrated Products, Inc., All Rights Reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
# OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.
#
# Except as contained in this notice, the name of Maxim Integrated
# Products, Inc. shall not be used except as stated in the Maxim Integrated
# Products, Inc. Branding Policy.
#
# The mere transfer of this software does not imply any licenses
# of trade secrets, proprietary technology, copyrights, patents,
# trademarks, maskwork rights, or any other form of intellectual
# property whatsoever. Maxim Integrated Products, Inc. retains all
# ownership rights.
#
###############################################################################

# plot_per_results.py
#
# Create plots of the CI PER test results
#

import sys
import argparse
from argparse import RawTextHelpFormatter
import json
from time import sleep
import matplotlib.pyplot as plt
import ntpath
import numpy as np
import os
import pandas
from pprint import pprint
from scipy.interpolate import interp1d
import socket
import itertools

RES_DIR = '/home/btm-ci/Workspace/ci_results/per'
TMP_PATH = '/tmp/msdk/ci/per'
SPEC = 30  # per spec in %
phy_str = ["", "1M", "2M", "S8", "S2"]

# Setup the command line description text
descText = """
PER plotting tools.

Takes .csv file arguments and creates PDF plots.

First row in each file should be a header row with the following columns:
packetLen,phy,atten,txPower,perMaster,perSlave
"""

# Parse the command line arguments
parser = argparse.ArgumentParser(
    description=descText, formatter_class=RawTextHelpFormatter)
parser.add_argument('csvFile', help='csv file containing PER data.')
parser.add_argument('desc', help='Description of data.')
parser.add_argument('basename', help='PDF file base name.')
parser.add_argument('--job_time', default="2023-01-01_00-00-00", help="current time of this job, use for part of url file name")

args = parser.parse_args()

#print("csvFile :", args.csvFile)

csv_full_path = f'{args.csvFile}'
csv_full_path = os.path.expanduser(csv_full_path)
print(f'csv full: {csv_full_path}')

pdf_file_name = args.csvFile.replace('.csv', '.pdf')
pdf_file_name = os.path.expanduser(pdf_file_name)

# Get the IP of this machine
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
ip = s.getsockname()[0]
#print(f'ip: {ip}')

# final url file
url_file_name = TMP_PATH + '/' + args.job_time + '.url'
url_file = open(url_file_name, 'a')

zip_file_list_name = TMP_PATH + '/' + args.job_time + '_zip_list.txt'
zip_file_list = open(zip_file_list_name, 'a')

url_links = list()

# Create the plots
phys = []
lens = []
attens = []
txPowers = []

df = pandas.read_csv(csv_full_path)
lens = df["packetLen"].unique()
phys = df["phy"].unique()
attens = df["atten"].unique()
txPowers = df["txPower"].unique()

# check if use PER mask
CONFIG_FILE = os.path.expanduser("~/Workspace/ci_config/RF-PHY-closed.json")
obj = json.load(open(CONFIG_FILE))
use_per_mask = obj['tests']["simple_per.yml"]["use_per_mask"]
per_mask_margin = int(obj['tests']['per_mask']['per_mask_margin'])
per_corr_dtm_to_cm = int(obj['tests']['per_mask']['per_corr_dtm_to_cm'])

phy_str = [
    "",
    "1M",
    "2M",
    "S8",
    "S2"
]

per_mask = {
    "1M": [
        [-20, per_mask_margin],
        [-90, per_mask_margin],
        [-93 + per_corr_dtm_to_cm, 5 + per_mask_margin],
        [-96 + per_corr_dtm_to_cm, 30 + per_mask_margin],
        [-99 + per_corr_dtm_to_cm, 100],
        [-114, 100]
    ],

    "2M": [
        [-20, per_mask_margin],
        [-87, per_mask_margin],
        [-90 + per_corr_dtm_to_cm, 5 + per_mask_margin],
        [-93 + per_corr_dtm_to_cm, 30 + per_mask_margin],
        [-96 + per_corr_dtm_to_cm, 100],
        [-111, 100]
    ],

    "S2": [
        [-20, per_mask_margin],
        [-95, per_mask_margin],
        [-98 + per_corr_dtm_to_cm, 5 + per_mask_margin],
        [-101 + per_corr_dtm_to_cm, 30 + per_mask_margin],
        [-104 + per_corr_dtm_to_cm, 100],
        [-119, 100]
    ],

    "S8": [
        [-20, per_mask_margin],
        [-98, per_mask_margin],
        [-101 + per_corr_dtm_to_cm, 5 + per_mask_margin],
        [-104 + per_corr_dtm_to_cm, 30 + per_mask_margin],
        [-107 + per_corr_dtm_to_cm, 100],
        [-122, 100]
    ]
}

# print("--------------------------------------------------------------------------------------------")
row = len(lens) * len(txPowers)
col = len(phys)
# print(f'row: {row}, col: {col}')

# local-full-per-test-2023-03-19_20-51-33_max32655_EvKit_V1.pdf
name = pdf_file_name.split('/')[-1]
board = name.split('_')[2].upper()
board_type = name.split("_")[3].upper() # EVKIT, WLP
if board == "MAX32655":
    board_rev = "PCB-00177-B-0"
elif board == "MAX32665":
    board_rev = "PCB-00129-C-0"
elif board == "MAX32690" and board_type == "EVKIT":
    board_rev = "PCB-00224-A-0"
elif board == "MAX32690" and board_type == "WLP":
    board_rev = "PCB-00247-A-0"
else:
    board_rev = ""

if row > 1 or col > 1:
    fig, axs = plt.subplots(row, col)

    fig.suptitle(f'Packet Error Rate vs Rx Power\n{board} {board_rev}', fontsize=10)

    if axs.ndim == 1:
        plt.subplots_adjust(top=0.83, hspace=0.5)
    else:
        fig.tight_layout()
        plt.subplots_adjust(top=0.80, bottom=0.1, hspace=0.5)

    case = 0
    for packetLen, phy, txPower in itertools.product(lens, phys, txPowers):
        # print(f'CASE: {case + 1}')
        col = case % len(phys)
        row = int(case / len(phys))
        # print(f'row: {row}, col: {col}')

        # Create line plot with atten to perSlave
        # print("len     :", packetLen)
        # print("phy     :", phy)
        # print("txPower :", txPower)
        tempDf = df.loc[(df['packetLen'] == packetLen) & (
                df['phy'] == phy) & (df['txPower'] == txPower)]

        # generate per mask profile
        if use_per_mask == "1":
            x = [item[0] for item in per_mask[phy_str[phy]]]
            x = x[::-1]

            y = [item[1] for item in per_mask[phy_str[phy]]]
            y = y[::-1]

            f = interp1d(x, y, kind='linear')

            xx = np.arange(x[0], x[-1] + 1)
            # Interpolate the y values for the new x values
            yy = f(xx)

        title = f'packet len: {packetLen}, txPower: 0.7 dBm\nphy: {phy_str[phy]}'
        if axs.ndim == 1:
            axs[col].set_title(title, fontdict={'fontsize': 6, 'fontweight': 'medium'})
            axs[col].set_xlabel('Rx Power (dBm)', fontdict={"fontsize": 5})
            axs[col].set_ylabel('PER (%)', fontdict={"fontsize": 5})
            axs[col].tick_params(axis='both', which='major', labelsize=4)
            axs[col].plot(tempDf["atten"], tempDf["perSlave"], "-x", linewidth=0.25, ms=0.5)

            axs[col].axhline(y=SPEC, color='r', linestyle=':', linewidth=0.1)

            if use_per_mask == "1":
                axs[col].plot(xx, yy, "r-", linewidth=0.1, ms=0.1)
                axs[col].text(0.40, 0.99, f'PER mask margin: {per_mask_margin} dBm', ha='left', va='top', 
                              fontsize=3, transform=axs[col].transAxes)
        else:
            axs[row, col].set_title(title, fontdict={'fontsize': 6, 'fontweight': 'medium'})
            axs[row, col].set_xlabel('Rx Power (dBm)', fontdict={"fontsize": 4})
            axs[row, col].set_ylabel('PER (%)', fontdict={"fontsize": 4})
            axs[row, col].tick_params(axis='both', which='major', labelsize=4)
            axs[row, col].plot(tempDf["atten"], tempDf["perSlave"], "-x", linewidth=0.25, ms=0.5)

            axs[row, col].axhline(y=SPEC, color='r', linestyle=':', linewidth=0.1)

            if use_per_mask == "1":
                axs[row, col].plot(xx, yy, "r-", linewidth=0.1, ms=0.1)
                axs[row, col].text(0.40, 0.99, f'PER mask margin: {per_mask_margin} dBm', ha='left', va='top', fontsize=3,
                                   transform=axs[row, col].transAxes)

        a = list(tempDf['atten'])
        p = list(tempDf['perSlave'])
        failed = False
        failed_index = 0
        for i in range(len(a)):
            if p[i] > SPEC:
                # print(f'{a[i]}, {p[i]}')
                failed = True
                failed_index = i
                if axs.ndim == 1:
                    #axs[col].plot((a[i], a[i]), [0, p[i]], color='r', linestyle=':', linewidth=0.1)
                    axs[col].text(a[i], p[i], f'  {p[i]}% @ {a[i]} dBm', ha='left',
                                    va='center', fontsize=3, color='red')
                    axs[col].text(0.6, 0.75, 'FAIL', fontsize=6, color='red', transform=axs[col].transAxes)
                else:
                    #axs[row, col].plot((a[i], a[i]), [0, p[i]], color='r', linestyle=':', linewidth=0.5)
                    axs[row, col].text(a[i], p[i], f'  {p[i]}% @ {a[i]} dBm', ha='left',
                                        va='center', fontsize=3, color='red')
                    axs[row, col].text(0.6, 0.75, 'FAIL', fontsize=6, color='red', transform=axs[row, col].transAxes)
                break

        # mark the last point
        i = -1
        if not failed:
            if axs.ndim == 1:
                axs[col].text(a[i], p[i], f'  {p[i]}% @ {a[i]} dBm', ha='left',
                            va='center', fontsize=3, color='black')
                axs[col].text(0.6, 0.75, 'PASS', fontsize=6, color='green', transform=axs[col].transAxes)
            else:
                axs[row, col].text(a[i], p[i], f'  {p[i]}% @ {a[i]} dBm', ha='left',
                                va='center', fontsize=3, color='black')
                axs[row, col].text(0.6, 0.75, 'PASS', fontsize=6, color='green', transform=axs[row, col].transAxes)

        if failed and a[failed_index] != a[i]:
            if axs.ndim == 1:
                axs[col].text(a[i], p[i], f'  {p[i]}% @ {a[i]} dBm', ha='left',
                            va='center', fontsize=3, color='red')
            else:
                axs[row, col].text(a[i], p[i], f'  {p[i]}% @ {a[i]} dBm', ha='left',
                                va='center', fontsize=3, color='red')
        
        # note
        fig.text(.5, .01, f'Run on all data channels (no advertising channels).\n{args.desc}', ha='center',
                 fontdict={"fontsize": 5})

        # print()
        case += 1

    saved_file = pdf_file_name
    # print(saved_file)
    zip_file_list.write(saved_file + '\n')

    # http://10.20.14.104:8000/per/pdf/?pdf=msdk-2023-02-13_15-34-35_max32655_EvKit_V1.pdf
    url = f'http://{ip}:8000/per/pdf/?pdf={ntpath.basename(saved_file)}'
    #print(url)
    url_links.append(url)

    plt.savefig(saved_file)

    if False:
        # save to a png file
        png_file = pdf_file_name.replace(".pdf", ".png")
        # print(f'Save to file: {png_file}.')
        plt.savefig(png_file)
        zip_file_list.write(saved_file + '\n')

        # plt.show()

        # http://10.20.14.104:8000/per/image/?image=msdk-2023-02-13_15-34-35_max32655_EvKit_V1.png
        url = f'http://{ip}:8000/per/image/?image={ntpath.basename(png_file)}'
        url_links.append(url)

if False:
    # -------------------------------------------------------------------------------------------------
    # Save each test into separated PDF files.
    # -------------------------------------------------------------------------------------------------
    case = 0
    for packetLen, phy, txPower in itertools.product(lens, phys, txPowers):
        # print(f'CASE: {case + 1}')

        # Create line plot with atten to perSlave
        # print("len     :", packetLen)
        # print("phy     :", phy)
        # print("txPower :", txPower)
        tempDf = df.loc[(df['packetLen'] == packetLen) & (
                df['phy'] == phy) & (df['txPower'] == txPower)]

        name = pdf_file_name.split('/')[-1]
        board = name.split('_')[2].replace('.pdf', '').upper()

        fig = plt.figure()
        ax1 = fig.add_axes((0.1, 0.2, 0.8, 0.7))

        title = f'Packet Error Rate vs Rx Power\n' \
                f'\n{board}\n' \
                f'Packet length: {packetLen}, txPower: 0.7 dBm, PHY: {phy_str[phy]}'
        # f'Packet length: {packetLen}, PHY: {phy_str[phy]}, txPower:{txPower}'
        ax1.set_title(title)
        ax1.set_xlabel('Rx Power (dBm)')
        ax1.set_ylabel('PER (%)')

        ax1.plot(tempDf["atten"], tempDf["perSlave"], "-x", linewidth=1, ms=2)

        a = list(tempDf['atten'])
        p = list(tempDf['perSlave'])
        plt.axhline(y=SPEC, color='r', linestyle=':', linewidth=0.75)
        failed = False
        for i in range(len(a)):
            if p[i] > SPEC:
                failed = True
                print(f'{a[i]}, {p[i]}')
                plt.plot((a[i], a[i]), ([0, p[i]]), color='r', linestyle=':', linewidth=0.75)
                plt.text(a[i], p[i], f'  {p[i]}% @ {a[i]} dBm', ha='left',
                         va='center', fontsize=8)
                break

        # mark the last point
        i = -1
        if failed and a[failed_index] != a[i]:
            plt.text(a[i], p[i], f'  {p[i]}% @ {a[i]} dBm', ha='left',
                    va='center', fontsize=8)

        # resize the figure to match the aspect ratio of the Axes
        fig.set_size_inches(7, 8, forward=True)
        # fig.text(.5, .10, args.desc, ha='center', fontdict={"fontsize": 12})
        fig.text(.5, .05, f'Run on all data channels (no advertising channels).\n{args.desc}', ha='center',
                 fontdict={"fontsize": 12})

        filename = csv_full_path.replace('.csv', '')
        filename += f'_{packetLen}_{phy_str[phy]}_{txPower}.pdf'
        # print(f'Save to file: {filename}.')
        plt.savefig(filename)

        zip_file_list.write(filename + '\n')

        url = f'http://{ip}:8000/per/pdf/?pdf={ntpath.basename(filename)}'
        url_links.append(url)

        # save to a png file
        png_file = filename.replace(".pdf", ".png")
        # print(f'Save to file: {png_file}.')
        plt.savefig(png_file)
        zip_file_list.write(png_file + '\n')

        url = f'http://{ip}:8000/per/image/?image={ntpath.basename(png_file)}'
        url_links.append(url)

        # print()
        case += 1

print("PLOT DONE!\n")

for url in url_links:
    url_file.write(url + '\n')
    print(url)

url_file.close()
zip_file_list.close()

sys.exit(0)
