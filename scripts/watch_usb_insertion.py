#!/usr/bin/env python3
"""Observe insertion truth/loads; optional exact /forces JSONL recording."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--record',type=Path)
    p.add_argument('--wall-seconds',type=float,default=0.)
    args,ros=p.parse_known_args()
    rclpy.init(args=ros)
    node=Node('watch_usb_insertion')
    stream=args.record.open('x',encoding='utf-8',buffering=1) if args.record else None
    previous=[None,0.]
    def observe(message):
        if stream:stream.write(message.data+'\n')
        s=json.loads(message.data).get('usb_insertion')
        if not s:return
        now=time.monotonic()
        if s['state']==previous[0] and now-previous[1]<.5:return
        previous[:]=[s['state'],now]
        o=s.get('observation',{})
        print(json.dumps(dict(state=s['state'],reason=s['reason'],depth_m=o.get('depth_m'),
            lateral_m=o.get('lateral_error_m'),angle_rad=o.get('orientation_error_rad'),
            resistance_N=o.get('resistance_N'),speed_m_s=s['speed_m_s'],
            insertion_success=s['insertion_success'],retention_active=s['retention_active'],
            grippers_released=s['grippers_released'],return_complete=s['return_complete'])),flush=True)
    sub=node.create_subscription(String,'/maniskill/forces',observe,10)
    start=time.monotonic()
    try:
        while rclpy.ok() and (not args.wall_seconds or time.monotonic()-start<args.wall_seconds):
            rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:pass
    finally:
        if stream:stream.close()
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
