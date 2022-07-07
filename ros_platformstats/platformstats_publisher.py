#*****************************************************************************
#
# Copyright (C) 2022 Xilinx, Inc.  All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
#******************************************************************************

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from std_msgs.msg import String
from platformstats import platformstats

class PlatformstatsPublisher(Node):

    def __init__(self):
        self.arr = DiagnosticArray()
        super().__init__('platformstats_publisher')
        self.publisher_ = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


    def timer_callback(self):
        self.ram_util = platformstats.get_ram_memory_utilization()
        msg = DiagnosticStatus(
            name='RAM Utilization: MemTotal',
            message='{memtotal} kB'.format(memtotal=self.ram_util[1])
        )
        self.arr.status = [msg]
        msg = DiagnosticStatus(
            name='RAM Utilization: MemFree',
            message='{memfree} kB'.format(memfree=self.ram_util[2])
        )
        self.arr.status += [msg]
        msg = DiagnosticStatus(
            name='RAM Utilization: MemAvailable',
            message='{memavail} kB'.format(memavail=self.ram_util[2])
        )
        self.arr.status += [msg]

        self.cpu_util = platformstats.get_cpu_utilization()
        for cpu in range(len(self.cpu_util)):
            msg = DiagnosticStatus(
                name='CPU Utilization: CPU{num}'.format(num=cpu),
                message='{util:0.2f}%'.format(util=self.cpu_util[cpu])
            )
            self.arr.status += [msg]

        self.swap_util = platformstats.get_swap_memory_utilization()
        msg = DiagnosticStatus(
            name='Swap Memory Utilization: SwapTotal',
            message='{swaptotal} kB'.format(swaptotal=self.swap_util[1])
        )
        self.arr.status += [msg]
        msg = DiagnosticStatus(
            name='Swap Memory Utilization: SwapFree',
            message='{swapfree} kB'.format(swapfree=self.swap_util[2])
        )
        self.arr.status += [msg]

        self.cma_util = platformstats.get_cma_utilization()
        msg = DiagnosticStatus(
            name='CMA Memory Utilization: CmaTotal',
            message='{cmatotal} kB'.format(cmatotal=self.cma_util[1])
        )
        self.arr.status += [msg]
        msg = DiagnosticStatus(
            name='CMA Memory Utilization: CmaFree',
            message='{cmafree} kB'.format(cmafree=self.cma_util[2])
        )
        self.arr.status += [msg]

        self.temp = platformstats.get_temperatures()
        msg = DiagnosticStatus(
            name='Temperature (LPD)',
            message='{lpdtemp:0.1f} C'.format(lpdtemp=self.temp[1]/1000)
        )
        self.arr.status += [msg]
        msg = DiagnosticStatus(
            name='Temperature (FPD)',
            message='{fpdtemp:0.1f} C'.format(fpdtemp=self.temp[2]/1000)
        )
        self.arr.status += [msg]
        msg = DiagnosticStatus(
            name='Temperature (PL)',
            message='{pltemp:0.1f} C'.format(pltemp=self.temp[3]/1000)
        )
        self.arr.status += [msg]

        self.volt = platformstats.get_voltages()
        msg = DiagnosticStatus(
            name='Voltage: VCC_PSPLL',
            message='{voltage} mV'.format(voltage=self.volt[1])
        )
        self.arr.status += [msg]
        msg = DiagnosticStatus(
            name='Voltage: PL_VCCINT',
            message='{voltage} mV'.format(voltage=self.volt[2])
        )
        self.arr.status += [msg]
        msg = DiagnosticStatus(
            name='Voltage: VOLT_DDRS',
            message='{voltage} mV'.format(voltage=self.volt[3])
        )
        self.arr.status += [msg]
        msg = DiagnosticStatus(
            name='Voltage: VCC_PSINTFP',
            message='{voltage} mV'.format(voltage=self.volt[4])
        )
        self.arr.status += [msg]
        msg = DiagnosticStatus(
            name='Voltage: VCC_PS_FPD',
            message='{voltage} mV'.format(voltage=self.volt[5])
        )
        self.arr.status += [msg]
        msg = DiagnosticStatus(
            name='Voltage: PS_IO_BANK_500',
            message='{voltage} mV'.format(voltage=self.volt[6])
        )
        self.arr.status += [msg]
        msg = DiagnosticStatus(
            name='Voltage: VCC_PS_GTR',
            message='{voltage} mV'.format(voltage=self.volt[7])
        )
        self.arr.status += [msg]
        msg = DiagnosticStatus(
            name='Voltage: VTT_PS_GTR',
            message='{voltage} mV'.format(voltage=self.volt[8])
        )
        self.arr.status += [msg]
        msg = DiagnosticStatus(
            name='Total Voltage',
            message='{voltage} mV'.format(voltage=self.volt[9])
        )
        self.arr.status += [msg]

        self.curr = platformstats.get_current()
        msg = DiagnosticStatus(
            name='Total Current',
            message='{curr} mV'.format(curr=self.curr[1])
        )
        self.arr.status += [msg]

        self.power = platformstats.get_power()
        msg = DiagnosticStatus(
            name='Total Power',
            message='{pwr} mV'.format(pwr=self.power[1]//1000)
        )
        self.arr.status += [msg]

        self.publisher_.publish(self.arr)

def main(args=None):
    rclpy.init(args=args)
    platformstats.init()

    platformstats_publisher = PlatformstatsPublisher()

    rclpy.spin(platformstats_publisher)

    platformstats_publisher.destroy_node()
    platformstats.deinit()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
