#!/usr/bin/env python3
"""Synthetic A/B/C BLE RX acceptance test, no MCU/BLE/COM port required."""
import unittest
from ble_rx_qualification import (parse_line, build_nonce, nonce_value,
                                  assess_challenges, decide, Capture)


def status(rx=0, ok=0, bad=0, fe=0, ore=0, ne=0, overflow=0):
    return dict(rx=rx, ok=ok, bad=bad, fe=fe, ore=ore, ne=ne,
                overflow=overflow, locked=1)


class RXQualification(unittest.TestCase):
    def test_nonce_commands_fit_default_20_byte_att_payload(self):
        for seq in (0, 1, 17, 9999):
            command=build_nonce(seq)
            self.assertEqual(len(command), 16)
            self.assertEqual(command[-1:], b"\n")
            self.assertEqual(command[2:6], f"{seq:04d}".encode())
            self.assertEqual(int(command[7:15],16), nonce_value(seq))

    def test_parse_uart_hex_value_all_digits(self):
        e=parse_line("RXACK,seq=17,value=12345678,ms=114")
        self.assertEqual(e["seq"],17)
        self.assertEqual(e["value"],"12345678")
        self.assertEqual(e["ms"],114)

    def test_corrupt_and_missing_ack_count(self):
        expected={1:nonce_value(1), 2:nonce_value(2),3:nonce_value(3)}
        data=[parse_line(f"RXACK,seq=1,value={expected[1]:08X}"),
              parse_line("RXACK,seq=2,value=00000000")]
        ble=[parse_line(f"RACK,seq=1,value={expected[1]:08X}")]
        r=assess_challenges(expected,{1,2,3},data,ble,[],
                            status(),status(rx=48,ok=2,bad=1))
        self.assertEqual(r["independent_uart_acks"],1)
        self.assertEqual(r["ble_echo_acks"],1)
        self.assertEqual(r["uart_missing_seq"],[2,3])
        self.assertEqual(r["uart_corrupted_seq"],[2])
        self.assertEqual(r["verdict"],"INTERMITTENT_OR_INCOMPLETE_RX")

    def test_complete_rx_without_ble_echo_is_rx_functional(self):
        expected={1:nonce_value(1),2:nonce_value(2)}
        uart=[parse_line(f"RXACK,seq={i},value={expected[i]:08X}")
              for i in expected]
        r=assess_challenges(expected,{1,2},uart,[],[],
                            status(),status(rx=32,ok=2))
        self.assertEqual(r["verdict"],"RX_FUNCTIONAL_IN_THIS_TEST")
        self.assertEqual(r["ble_echo_acks"],0)
        self.assertEqual(decide({"A":{"pass":True},"B":{"pass":True},
                                 "challenge_results":[r]}),
                         "RX_FUNCTIONAL_IN_TESTED_RANGE")

    def test_write_failed_and_no_ack_not_hardware_broken(self):
        r=assess_challenges({10:nonce_value(10)},set(),[],[],
                            ["FFE1 Unreachable"],status(),status())
        self.assertEqual(r["verdict"],"GATT_WRITE_NOT_TESTABLE")
        self.assertEqual(decide({"A":{"pass":True},"B":{"pass":True},
                                 "challenge_results":[r]}),
                         "NOT_TESTABLE_NO_GATT_WRITE")

    def test_uart_parser_handles_fragmented_lines(self):
        from io import StringIO
        pending=bytearray()
        ev=[]
        s=StringIO()
        Capture._parse(b"RXS,mode=A,rx=0\r",pending,ev,"UART1",s)
        self.assertFalse(ev)
        Capture._parse(b"\nRXACK,seq=1,value=12345678\r\n",
                       pending,ev,"UART1",s)
        self.assertEqual([e["type"] for e in ev],["RXS","RXACK"])
        self.assertEqual(ev[-1]["value"],"12345678")


if __name__=="__main__":
    unittest.main(verbosity=2)
