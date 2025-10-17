#! /usr/bin/env python3
import rclpy

def main(args=None):
    rclpy.init(args=args)
    print("Hello, Arm Robot!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()