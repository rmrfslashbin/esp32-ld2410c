# LD2410C Datasheets and Documentation

This directory contains official documentation for the HLK-LD2410C sensor.

## Official Documents

This directory contains both the official PDFs and AI-friendly Markdown conversions:

### Markdown References (AI-Optimized)
- **`LD2410C_Protocol.md`** - Protocol specification converted to Markdown
- **`HLK-LD2410C_Reference.md`** - Hardware datasheet converted to Markdown

These Markdown files are optimized for AI coders and provide searchable, structured documentation.

### PDF Originals

### 1. Serial Communication Protocol ✅
**Filename:** `LD2410C Serial communication protocol V1.00.pdf` (549 KB)
**Source:** Shenzhen Hi-Link Electronic Co., Ltd
**Date:** 2022-11-7
**Download:** https://www.hlktech.net/index.php?id=988
**Description:** Complete protocol specification including:
- Frame formats (command and data frames)
- Command reference (0xFF, 0xFE, 0x60, 0x61, 0x62, etc.)
- Data structure definitions
- Configuration parameters
- Default settings

### 2. Product Datasheet ✅
**Filename:** `HLK-LD2410C_datasheet.pdf` (1.6 MB)
**Source:** Hi-Link
**Download:** https://www.hlktech.net/index.php?id=988
**Alternative:** https://drive.google.com/file/d/1CYgZTUEJRasIE8QZA22C3LO2NItiWSYs/view
**Description:** Hardware specifications including:
- Electrical characteristics (5-12V power supply)
- Physical dimensions
- Detection range (0-6 meters typical)
- Antenna specifications
- Pin definitions

## Why Include These?

1. **Protocol Reference:** Developers can verify implementation against official spec
2. **Offline Development:** No internet required to check specifications
3. **Version Control:** Track which protocol version the driver supports
4. **Legal Clarity:** Shows proper attribution and reference sources
5. **Troubleshooting:** Users can cross-reference hardware behavior

## Copyright Notice

All datasheets are copyright Shenzhen Hi-Link Electronic Co., Ltd.
Included here under fair use for technical reference purposes.
See individual documents for full copyright and usage terms.

## File Size

Total: ~2.1 MB - Small enough for standard Git (no LFS needed)
