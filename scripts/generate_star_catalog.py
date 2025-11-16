#!/usr/bin/env python3
"""
BSC5 Binary Catalog Reader and STM32 Converter

Reads the Yale Bright Star Catalog 5 binary format and converts to
optimized 16-byte format for STM32F407VET6.

Download BSC5 from: http://tdc-www.harvard.edu/catalogs/BSC5
"""

import os
from pathlib import Path
import pathlib
import struct
import sys
from urllib import request
import numpy as np
from dataclasses import dataclass
from typing import List


@dataclass
class StarEntry:
    """Single star entry from BSC5 catalog"""

    catalog_num: int
    ra_rad: float  # J2000 Right Ascension (radians)
    dec_rad: float  # J2000 Declination (radians)
    spectral_type: str  # 2-character spectral type
    magnitude: int  # Visual magnitude × 100
    pmra: float  # RA proper motion (rad/yr)
    pmdec: float  # Dec proper motion (rad/yr)


def download_bsc5(url: str, output_path: Path) -> None:
    """Download BSC5 catalog from Harvard TDC."""
    print(f"📥 Downloading BSC5 catalog from {url}...")

    try:
        with request.urlopen(url) as response:
            data = response.read()
            output_path.write_bytes(data)
        print(f"✅ Downloaded {len(data):,} bytes to {output_path}")
    except Exception as e:
        print(f"❌ Error downloading catalog: {e}")
        sys.exit(1)


def read_bsc5_header(f):
    """Read 28-byte BSC5 header"""
    header_data = f.read(28)

    # Unpack header
    star0, star1, starn, stnum, mprop, nmag, nbent = struct.unpack("<7i", header_data)
    starn = abs(starn)

    print("BSC5 Header:")
    print(f"  Stars in file: {starn}")
    print(f"  First star number: {star1}")
    print(f"  Proper motion included: {mprop == 1}")
    print(f"  Number of magnitudes: {nmag}")
    print(f"  Bytes per entry: {nbent}")
    print()

    return starn


def read_bsc5_entry(f) -> StarEntry | None:
    """Read single 32-byte BSC5 star entry"""
    entry_data = f.read(32)

    if len(entry_data) < 32:
        return None

    # Unpack binary data (little-endian)
    # Format: f=float(4), d=double(8), 2s=2 char string, h=short(2)
    catalog_num = struct.unpack("<f", entry_data[0:4])[0]
    ra_rad = struct.unpack("<d", entry_data[4:12])[0]
    dec_rad = struct.unpack("<d", entry_data[12:20])[0]
    spectral = entry_data[20:22].decode("ascii", errors="ignore")
    magnitude = struct.unpack("<h", entry_data[22:24])[0]
    pmra = struct.unpack("<f", entry_data[24:28])[0]
    pmdec = struct.unpack("<f", entry_data[28:32])[0]

    return StarEntry(
        catalog_num=int(catalog_num),
        ra_rad=ra_rad,
        dec_rad=dec_rad,
        spectral_type=spectral,
        magnitude=magnitude,
        pmra=pmra,
        pmdec=pmdec,
    )


def read_bsc5_catalog(filename: str) -> List[StarEntry]:
    """Read entire BSC5 binary catalog"""
    stars = []

    with open(filename, "rb") as f:
        # Read header
        num_stars = read_bsc5_header(f)

        # Read all star entries
        for i in range(num_stars):
            star = read_bsc5_entry(f)
            if star:
                stars.append(star)

    print(f"Successfully read {len(stars)} stars from BSC5")
    return stars


def filter_by_magnitude(
    stars: List[StarEntry], max_mag: float = 6.5
) -> List[StarEntry]:
    """Filter stars by maximum magnitude"""
    max_mag_scaled = int(max_mag * 100)
    filtered = [s for s in stars if s.magnitude <= max_mag_scaled]

    print(f"Filtered to magnitude {max_mag}: {len(filtered)} stars")
    return filtered


def sort_by_brightness(stars: List[StarEntry]) -> List[StarEntry]:
    """Sort stars by brightness (magnitude ascending)"""
    return sorted(stars, key=lambda s: s.magnitude)


def write_stm32_binary(stars: List[StarEntry], output_file: str):
    """
    Write optimized 16-byte binary format for STM32F407VET6

    Format per star (16 bytes total):
        float    ra_rad          (4 bytes)
        float    dec_rad         (4 bytes)
        int16_t  magnitude       (2 bytes)
        uint16_t star_id         (2 bytes)
        uint8_t  spectral_type   (1 byte)  - first char only
        uint8_t  flags           (1 byte)  - reserved
        uint16_t reserved        (2 bytes) - padding
    """

    with open(output_file, "wb") as f:
        for i, star in enumerate(stars):
            # Convert to single-precision float
            ra_float = float(star.ra_rad)
            dec_float = float(star.dec_rad)

            # Get first character of spectral type
            spec_char = ord(star.spectral_type[0]) if star.spectral_type else 0

            # Pack as 16-byte structure
            # Format: 2 floats (f), 3 uint16 (H), 2 uint8 (B)
            data = struct.pack(
                "<ffhHBBH",  # Little-endian, 16 bytes total
                ra_float,  # RA in radians
                dec_float,  # Dec in radians
                star.magnitude,  # Magnitude × 100
                i,  # Star ID (index in catalog)
                spec_char,  # Spectral type (first char)
                0,  # Flags (reserved)
                0,  # Reserved padding
            )

            f.write(data)

    print(f"Wrote {len(stars)} stars to {output_file}")
    print(f"Binary size: {len(stars) * 16} bytes ({len(stars) * 16 / 1024:.1f} KB)")


def write_c_header(
    stars: List[StarEntry], output_file: str, catalog_name: str = "STAR_CATALOG"
):
    """
    Generate C header file with catalog definition
    """

    with open(output_file, "w") as f:
        f.write(
            f"""/*
 * Auto-generated star catalog from BSC5
 * Generated from {len(stars)} stars
 * Format: 16 bytes per star (optimized for STM32F407VET6)
 */

#ifndef STAR_CATALOG_H
#define STAR_CATALOG_H

#include <stdint.h>

/* Star entry structure (16 bytes, 4-byte aligned) */
typedef struct __attribute__((aligned(4))) {{
    float    ra_rad;          /* Right Ascension (radians) */
    float    dec_rad;         /* Declination (radians) */
    uint16_t magnitude;       /* Visual magnitude × 100 */
    uint16_t star_id;         /* Catalog identifier */
    uint8_t  spectral_type;   /* Spectral type (first char) */
    uint8_t  flags;           /* Reserved flags */
    uint16_t reserved;        /* Alignment padding */
}} StarEntry;

/* Catalog constants */
#define NUM_STARS {len(stars)}
#define CATALOG_SIZE_BYTES (NUM_STARS * sizeof(StarEntry))

/* Memory location (adjust for your linker script) */
#define STAR_CATALOG_FLASH_ADDR 0x08060000

/* Access catalog from Flash */
#define STAR_CATALOG ((const StarEntry*)STAR_CATALOG_FLASH_ADDR)

/* Inline accessor functions */
static inline const StarEntry* get_star(uint16_t index) {{
    return (index < NUM_STARS) ? &STAR_CATALOG[index] : NULL;
}}

static inline float star_magnitude(uint16_t index) {{
    return (float)STAR_CATALOG[index].magnitude / 100.0f;
}}

#endif /* STAR_CATALOG_H */
"""
        )

    print(f"Wrote C header to {output_file}")


def print_statistics(stars: List[StarEntry]):
    """Print catalog statistics"""
    mags = [s.magnitude / 100.0 for s in stars]

    print("\n" + "=" * 50)
    print("Catalog Statistics")
    print("=" * 50)
    print(f"Total stars:        {len(stars)}")
    print(f"Brightest:          {min(mags):.2f} mag")
    print(f"Faintest:           {max(mags):.2f} mag")
    print(f"Mean magnitude:     {np.mean(mags):.2f} mag")
    print(f"Memory (binary):    {len(stars) * 16 / 1024:.1f} KB")
    print("Flash address:      0x08060000")
    print(f"End address:        0x{0x08060000 + len(stars) * 16:08X}")
    print("=" * 50)


def main():
    """Main conversion workflow"""

    # Configuration
    BSC5_FILE = "BSC5"  # Download from Harvard TDC
    MAX_MAGNITUDE = 6.5  # Brightness cutoff
    OUTPUT_BINARY = "star_catalog.bin"  # STM32 binary format
    OUTPUT_HEADER = "star_catalog.h"  # C header file
    work_dir = os.getcwd() + "/firmware/catalog/"
    bcs5_path = pathlib.Path(work_dir + BSC5_FILE)

    print("BSC5 to STM32 Star Catalog Converter")
    print("=" * 50)
    print()

    # Step 0: Download
    if not bcs5_path.exists():
        print("Step 0: Downloading BSC5 catalog...")
        download_bsc5("http://tdc-www.harvard.edu/catalogs/BSC5", bcs5_path)

    # Step 1: Read BSC5 binary catalog
    print("Step 1: Reading BSC5 catalog...")
    stars = read_bsc5_catalog(str(bcs5_path))

    # Step 2: Filter by magnitude
    print(f"\nStep 2: Filtering to magnitude {MAX_MAGNITUDE}...")
    stars = filter_by_magnitude(stars, MAX_MAGNITUDE)

    # Step 3: Sort by brightness
    print("\nStep 3: Sorting by brightness...")
    stars = sort_by_brightness(stars)

    # Step 4: Write STM32 binary
    print(f"\nStep 4: Writing STM32 binary ({OUTPUT_BINARY})...")
    write_stm32_binary(stars, work_dir + OUTPUT_BINARY)

    # Step 5: Write C header
    print(f"\nStep 5: Writing C header ({OUTPUT_HEADER})...")
    write_c_header(stars, work_dir + OUTPUT_HEADER)

    # Step 6: Print statistics
    print_statistics(stars)

    print("\n✅ Conversion complete!")
    print("\nNext steps:")
    print(f"1. Flash {OUTPUT_BINARY} to STM32 at address 0x08060000")
    print(f"2. Include {OUTPUT_HEADER} in your project")
    print("3. Access stars using: STAR_CATALOG[index]")


if __name__ == "__main__":
    main()
