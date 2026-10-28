"""Host-only regression tests: no Windows COM, radio or tkinter needed."""
import sys
from pathlib import Path
import unittest
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from classic_spp_protocol import LineFramer, MazeState, numeric_fields
from classic_spp_ports import classify_port


def map_frames(n=5, corrupt=False):
    rows = []
    for y in range(n):
        rows.append("".join("%X" % ((1 if y == n-1 else 0)
                    | (4 if y == 0 else 0)
                    | (2 if x == n-1 else 0)
                    | (8 if x == 0 else 0)) for x in range(n)))
    raw = b"MAP1" + bytes((n, n, 0, 0)) + bytes(
        int(c, 16) for row in rows for c in row)
    crc = "%08X" % (zlib.crc32(raw) & 0xffffffff)
    cfg = "MAPCFG,fmt=1,w=%d,h=%d,len=%d,crc=%s,src=RAM" % (
        n, n, len(raw), crc)
    rowframes = ["MAPROW,y=%d,hex=%s" % (y, row) for y, row in enumerate(rows)]
    if corrupt:
        rowframes[-1] = rowframes[-1][:-1] + (
            "0" if rowframes[-1][-1] != "0" else "1")
    return [cfg, *rowframes, "MAPEND,rows=%d,crc=%s" % (n, crc)]


class ClassicProtocolTests(unittest.TestCase):
    def test_windows_com_direction(self):
        spp = r"BTHENUM\{00001101-0000-1000-8000-00805F9B34FB}_LOCALMFG&0000\7&31AD681E&0&000000000000_0000000C"
        self.assertEqual(classify_port(spp), ("incoming", None))
        spp = r"BTHENUM\{00001101-0000-1000-8000-00805F9B34FB}_LOCALMFG&0002\7&31AD681E&0&107100E9347B_C00000000"
        self.assertEqual(classify_port(spp), ("outgoing", "107100E9347B"))
        spp = r"BTHENUM\{00001101-0000-1000-8000-00805F9B34FB}_VID&0001004C_PID&2006\7&31AD681E&0&3CCD36769BEF_C00000000"
        self.assertEqual(classify_port(spp), ("outgoing", "3CCD36769BEF"))
        self.assertEqual(classify_port("PCI\\VEN_8086"), ("other", None))

    def test_fragmented_crlf(self):
        framer = LineFramer()
        self.assertEqual(framer.feed(b"MAPCFG,fmt=1\r"), [])
        self.assertEqual(framer.feed(b"\nCELL:1,2,3,0,FF\n"),
                         ["MAPCFG,fmt=1", "CELL:1,2,3,0,FF"])
        self.assertEqual(framer.feed(b"t=4 "), [])
        self.assertEqual(framer.feed(b"ms=3\n"), ["t=4 ms=3"])

    def test_bounded_partial(self):
        framer = LineFramer(max_pending=8)
        self.assertEqual(framer.feed(b"a" * 12), [])
        self.assertEqual(framer.discarded, 1)
        self.assertEqual(framer.pending, bytearray())

    def test_map_crc_pass_and_cell(self):
        maze = MazeState()
        frames = map_frames(5)
        for line in frames:
            result = maze.apply(line)
        self.assertTrue(result.startswith("MAP PASS"))
        self.assertEqual(maze.n, 5)
        self.assertTrue(maze.complete)
        self.assertEqual(len(maze.visited), 0)
        self.assertEqual(maze.walls[0][0] & 12, 12)
        self.assertTrue(maze.apply("CELL:2,2,3,1,FF").startswith("CELL"))
        self.assertEqual((maze.x, maze.y, maze.direction), (2, 2, 1))
        self.assertIn((2, 2), maze.visited)
        self.assertEqual(maze.walls[2][3] & 8, 8)
        self.assertEqual(maze.walls[3][2] & 4, 4)

    def test_map_corruption_cannot_replace_last_valid(self):
        maze = MazeState()
        for frame in map_frames(5):
            maze.apply(frame)
        original = [r[:] for r in maze.walls]
        for frame in map_frames(5, corrupt=True):
            result = maze.apply(frame)
        self.assertIn("FAIL", result)
        self.assertEqual(maze.walls, original)
        self.assertEqual(maze.source, "RAM")

    def test_missing_rows_rejected(self):
        maze = MazeState()
        frames = map_frames(5)
        maze.apply(frames[0])
        for frame in frames[1:-2]:
            maze.apply(frame)
        self.assertIn("FAIL", maze.apply(frames[-1]))
        self.assertFalse(maze.complete)

    def test_maze_legacy(self):
        maze = MazeState()
        cells = "0" * 256
        result = maze.apply("MAZE:2,3,1:" + cells)
        self.assertIn("MAZE", result)
        self.assertEqual((maze.x, maze.y, maze.direction), (2, 3, 1))

    def test_mz_wall_chunks(self):
        maze = MazeState()
        wh = "0" * 256
        self.assertIsNone(maze.apply("MZ0:2,3,1:" + wh[:64]))
        for i in (1, 2):
            self.assertIsNone(maze.apply("MZ%d:%s" % (i, wh[i * 64:(i + 1) * 64])))
        self.assertIn("MZ walls", maze.apply("MZ3:" + wh[192:]))
        self.assertEqual((maze.x, maze.y), (2, 3))

    def test_numeric_fields_and_wall_trace(self):
        f = numeric_fields("IR6,ms=120,l0=120,r0=300,yaw=-1.25")
        self.assertEqual(f["l0"], 120)
        self.assertEqual(f["yaw"], -1.25)
        self.assertNotIn("trace", numeric_fields("W,TRACE_END"))
        self.assertNotIn("bad", numeric_fields("bad=n/a"))

    def test_malformed_values_are_safe(self):
        maze = MazeState()
        self.assertIn("invalid", maze.apply("MAPCFG,fmt=1,w=16,h=17,len=264,crc=0,src=X"))
        self.assertIn("ignored", maze.apply("CELL:999,999,Z,7"))
        self.assertFalse(maze.complete)


if __name__ == "__main__":
    unittest.main()
