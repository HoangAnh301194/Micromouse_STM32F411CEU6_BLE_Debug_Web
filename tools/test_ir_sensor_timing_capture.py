import unittest

import ir_sensor_timing_capture as capture


class TimingCaptureTests(unittest.TestCase):
    def sample_records(self):
        lines = [
            "SYS_IRD,ms=1000,start=1000,done=1000,fail=0,dma_start=4000,"
            "dma_done=4000,dma_err=0,dma_to=0,adc_ovr=0,scan_us=700000,"
            "scan_max_us=720,dma_us=88000,dma_max_us=24",
            "SYS_IRD,ms=2000,start=2000,done=2000,fail=0,dma_start=8000,"
            "dma_done=8000,dma_err=0,dma_to=0,adc_ovr=0,scan_us=1400000,"
            "scan_max_us=724,dma_us=176000,dma_max_us=25",
            "SYS_IRC,ms=1000,tim_irq=10000,tim_cy=0,tim_maxcy=150,"
            "dma_irq=4000,dma_cy=0,dma_maxcy=250,adc_clk=24000000",
            "SYS_IRC,ms=2000,tim_irq=20000,tim_cy=960000,tim_maxcy=162,"
            "dma_irq=8000,dma_cy=0,dma_maxcy=299,adc_clk=24000000",
            "SYS_IR,ms=1000,new=1000,old=0,age_us=200,age_max_us=280",
            "SYS_IR,ms=2000,new=2000,old=0,age_us=210,age_max_us=293",
            "SYS_T,ms=1000,ticks=1000,late=0,gap=0,maxcy=1040",
            "SYS_T,ms=2000,ticks=2000,late=0,gap=0,maxcy=1046",
            "SYS_M,ms=1000,i2cok=1000,i2cerr=0,to=0",
            "SYS_M,ms=2000,i2cok=2000,i2cerr=0,to=0",
            "SYS_U,ms=1000,tx_drop=0,dma_err=0,rx_err=0,rx_ov=0",
            "SYS_U,ms=2000,tx_drop=0,dma_err=0,rx_err=0,rx_ov=0",
            "SYS_UQ,ms=1000,q=100,peak=500,accb=1000,doneb=900,"
            "prncy=1000,dmacy=100,rxcy=0,rxb=0",
            "SYS_UQ,ms=2000,q=0,peak=1049,accb=2000,doneb=2000,"
            "prncy=2000,dmacy=200,rxcy=0,rxb=0",
        ]
        values = (
            "age_us=200,ambient_reads=100,pair1_reads=100,pair2_reads=100,"
            "pair3_reads=100,a0=10,a1=20,a2=30,a3=40,a4=50,a5=60,"
            "l0=11,l1=21,l2=31,l3=41,l4=51,l5=61,"
            "d0=1,d1=1,d2=1,d3=1,d4=1,d5=1"
        )
        lines.extend((
            f"SYS_IRV,ms=1000,scan=100,{values}",
            f"SYS_IRV,ms=2000,scan=200,{values}",
        ))
        return [capture.parse_record(line) for line in lines]

    def test_complete_run_summarizes_throughput_and_load(self):
        summary = capture.summarize(self.sample_records(), 1.0, "COM7")

        self.assertTrue(summary["valid_capture"])
        self.assertEqual(summary["malformed_records"], 0)
        self.assertEqual(summary["SYS_IRD"]["scan_rate_hz"], 1000.0)
        self.assertEqual(
            summary["SYS_IRC"]["ir_isr_cpu_percent_of_nominal_sysclk"], 1.0
        )
        self.assertEqual(summary["SYS_IR"]["fresh_control_tick_percent"], 100.0)
        self.assertEqual(summary["SYS_UQ"]["uart_queue_peak_bytes"], 1049)

    def test_malformed_records_and_uart_drops_invalidate_run(self):
        records = self.sample_records()
        records.append(capture.parse_record("SYS_IRD,ms=3000,damaged"))
        malformed = capture.summarize(records, 1.0, "COM7")
        self.assertFalse(malformed["valid_capture"])
        self.assertEqual(malformed["malformed_records"], 1)

        records = self.sample_records()
        records[11]["tx_drop"] = 1
        dropped = capture.summarize(records, 1.0, "COM7")
        self.assertFalse(dropped["valid_capture"])
        self.assertFalse(dropped["uart_transport_clean"])

    def test_counter_deltas_handle_wraparound(self):
        self.assertEqual(
            capture.counter_delta({"count": 0xFFFFFFFE}, {"count": 1}, "count"),
            3,
        )


if __name__ == "__main__":
    unittest.main()
