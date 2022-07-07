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

import os
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from std_msgs.msg import String

class PlatformstatsSubscriber(Node):

    def __init__(self):
        super().__init__('platformstats_subscriber')
        self.subscription = self.create_subscription(
            DiagnosticArray,
            'diagnostics',
            self.subscriber_callback,
            10)
        self.subscription  # prevent unused variable warning

    def subscriber_callback(self, msg):
        os.system('clear')
        for i in range(len(msg.status)):
            name = str(msg.status[i]).split("name='",1)[1].split("', message",1)[0]
            message = str(msg.status[i]).split("message='",1)[1].split("', hardware_id",1)[0]
            print(name + ": " + message)

def main(args=None):
    rclpy.init(args=args)

    platformstats_subscriber = PlatformstatsSubscriber()

    rclpy.spin(platformstats_subscriber)

    platformstats_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
