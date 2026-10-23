#!/usr/bin/env python3
"""Check actual C map generator and CRC against independent Python zlib and JS UI."""
from pathlib import Path
import importlib.util
import os
import shutil
import subprocess
import tempfile
import zlib

ROOT = Path(__file__).resolve().parents[1]


def main():
    assert shutil.which("gcc"), "gcc is needed for native C contract test"
    spec = importlib.util.spec_from_file_location(
        "flash_map_pipeline", ROOT/"tools"/"flash_map_pipeline.py")
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    code = r"""
#include "flash_map_protocol.h"
#include "flash_record_format.h"
#include <stdio.h>
int main(void) {
    uint8_t b[MAP_DIAG_MAX_BLOB];
    unsigned i,n,sz;
    for (sz=5U;sz<=16U;sz+=11U) {
        MapDiag_Make(b,sz);
        n=MapDiag_Length(sz);
        if (!MapDiag_Validate(b,n)) return 5;
        printf("%u,%u,%08lX,",sz,n,
              (unsigned long)Flash_DataCrc32(b,n));
        for (i=0;i<n;++i) printf("%02X",b[i]);
        putchar('\n');
        b[8]=0xFF;
        if (MapDiag_Validate(b,n)) return 6;
    }
    return 0;
}
"""
    with tempfile.TemporaryDirectory() as root:
        file = Path(root)/"test.c"
        exe = Path(root)/("test.exe" if os.name=="nt" else "test")
        file.write_text(code)
        subprocess.run(["gcc","-std=c11","-Wall","-Wextra","-Werror",
                        "-I",str(ROOT/"include"),
                        str(ROOT/"src"/"flash_map_protocol.c"),
                        str(ROOT/"src"/"flash_record_format.c"),
                        str(file),"-o",str(exe)],check=True)
        lines = subprocess.check_output([str(exe)],text=True).strip().splitlines()
        assert len(lines)==2
        for line,n in zip(lines,(5,16)):
            size,length,crc,raw=line.split(",")
            actual=bytes.fromhex(raw)
            assert int(size)==n and int(length)==len(actual)
            assert actual==tool.make_map(n), "C and host maps differ"
            assert int(crc,16)==zlib.crc32(actual)&0xFFFFFFFF
            assert len(actual)==8+n*n
            assert actual[0:4]==b"MAP1"
            assert all(x<=15 for x in actual[8:])
            print("PASS: C-generated map",n,"x",n,"payload",len(actual),"CRC",crc)
    for n in (5,16):
        payload=tool.make_map(n)
        lines=[
            "FM_CFG,w=%d,h=%d,len=%d,crc=%08X,src=RAM"
            % (n,n,len(payload),tool.crc32(payload)),
        ]
        for y in range(n):
            lines.append("FM_ROW,y=%d,hex=%s" % (
                y,"".join("%X" % b for b in payload[8+y*n:8+(y+1)*n])))
        lines.append("FM_DONE,rows=%d,crc=%08X,src=RAM" % (
            n,tool.crc32(payload)))
        assert tool.verify_uart(lines,n,"RAM")["status"]=="PASS"
        broken=list(lines)
        broken[1]=broken[1][:-1]+"F"
        if broken[1]==lines[1]: broken[1]=broken[1][:-1]+"0"
        try:
            tool.verify_uart(broken,n,"RAM")
        except AssertionError:
            pass
        else:
            raise AssertionError("UART corrupted row not detected")
    print("PASS: UART frame parser rejects corrupt map and accepts 5/16")


if __name__=="__main__":
    main()
