#!/usr/bin/python3
import argparse
from intelhex import IntelHex
import struct
import datetime


def main():
    parser = argparse.ArgumentParser(description="Merge bootloader, main fw and CRC+len")
    parser.add_argument("--boot", required=True)
    parser.add_argument("--main", required=True)
    parser.add_argument("--prefix", required=True)
    args = parser.parse_args()

    hexfile = IntelHex()

    with open(args.boot, "r") as boot_file:
        hexfile.loadhex(boot_file)

    with open(args.main, "r") as main_file:
        main_hex = IntelHex(main_file)
        hexfile.merge(main_hex, overlap='replace')  # Merge bootloader and main fw

    # Write it down
    dt_now = datetime.datetime.now()
    file_name = args.prefix + dt_now.strftime("%Y%m%d%H%M")
    hexfile.write_hex_file(file_name + ".hex")
    # hexfile.tobinfile(file_name + ".bin")


if __name__ == '__main__':
    main()
