#!/usr/bin/env python3
"""Native CRC32 regression for the real src/flash_record_format.c, no MCU needed.

Verifies CRC against independent zlib reference over serialized metadata+payload,
checks bit-flip detection, and enforces memory-map linker guard statically.
"""
from pathlib import Path
import re
import shutil
import struct
import subprocess
import tempfile
import zlib

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "include" / "flash_record_format.h"
SRC = ROOT / "src" / "flash_record_format.c"
LDSCRIPT = ROOT / "ld" / "STM32F411CE_FLASH_256K.ld"
PIO = ROOT / "platformio.ini"


def main():
    assert shutil.which("gcc"), "Native host gcc missing"
    ld = LDSCRIPT.read_text()
    ini = PIO.read_text()
    assert re.search(r"ORIGIN\s*=\s*0x08000000\s*,\s*LENGTH\s*=\s*256K", ld)
    assert "board_build.ldscript = ld/STM32F411CE_FLASH_256K.ld" in ini
    assert "board_upload.maximum_size = 262144" in ini
    source = (ROOT / "src" / "flash_storage.c").read_text()
    assert "Flash_RecordCrc32" in source
    assert "if (Control_IsRunning() || !Motor_OutputsLocked())" in source
    assert "FLASH->CR &= ~FLASH_CR_PG;" in source
    assert "if (FLASH->SR & FLASH_ERR_MASK) return FLASH_ERR_CONTROLLER;" in source
    assert "REC_COMMIT_OFFSET" in source

    vectors = [
        (0x4D4D4631, 0x00010001, 1, bytes(range(64))),
        (0x4D4D4631, 0x00010002, 2, b"123456789"),
        (0x4D4D4631, 0x00010003, 127, bytes([0xEF]) * 999),
        (0x4D4D4631, 0x00010001, 0xFFFFFFFE, b"x"),
    ]
    cases = []
    for magic, ver_type, seq, payload in vectors:
        metadata = struct.pack("<IIII", magic, ver_type, seq, len(payload))
        expected = zlib.crc32(metadata + payload) & 0xFFFFFFFF
        values = ",".join(str(x) for x in payload)
        cases.append(f"""
    {{
        const uint8_t buf[] = {{{values}}};
        Flash_RecordHeader_t h = {{0x{magic:08X}UL, 0x{ver_type:08X}UL, {seq}UL, {len(payload)}U, 0U}};
        if (Flash_RecordCrc32(&h, buf) != 0x{expected:08X}UL) return 11;
        h.sequence ^= 1U;
        if (Flash_RecordCrc32(&h, buf) == 0x{expected:08X}UL) return 12;
    }}
""")
    program = '#include "flash_record_format.h"\n#include <stdint.h>\nint main(void) {\n' + ''.join(cases) + '\nreturn 0;\n}\n'
    with tempfile.TemporaryDirectory() as td:
        harness = Path(td) / "test.c"
        exe = Path(td) / "test"
        harness.write_text(program)
        subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-I", str(ROOT / "include"), str(SRC), str(harness),
                        "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
    print("PASS: actual C CRC32 (4 vectors/metadata flips) + linker + safety static gates")


if __name__ == "__main__":
    main()
