#!/usr/bin/env python3
"""Offline versioned completion evidence; never ROS, device or execution authority."""
import argparse
import json
from pathlib import Path
import sys


def read_json(path,limit):
    with Path(path).open('rb') as stream:raw=stream.read(limit+1)
    if len(raw)>limit:raise ValueError('bounded input exceeds limit')
    def pairs(items):
        result={}
        for key,value in items:
            if key in result:raise ValueError('duplicate JSON key')
            result[key]=value
        return result
    value=json.loads(raw,object_pairs_hook=pairs)
    json.dumps(value,allow_nan=False)
    return value


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--response',required=True);parser.add_argument('--expected-context',required=True)
    parser.add_argument('--take-monotonic',required=True,type=float);parser.add_argument('--take-ros-ns',required=True,type=int)
    args=parser.parse_args(argv)
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2/src/so101_station_perception'))
    from so101_station_perception.completion_event import validate_status_event
    result=dict(schema='so101-completion-evidence-verification-v1',qualified=False,execution_enabled=False)
    try:
        response=read_json(args.response,16384);context=read_json(args.expected_context,8192)
        result.update(validate_status_event(response,context,received_mono=args.take_monotonic,received_ros_ns=args.take_ros_ns))
        result['observation_qualified']=False  # Evidence verification alone is not run/physical qualification.
        code=0
    except (ValueError,KeyError,TypeError,OSError) as error:
        result.update(historical_completion_valid=False,current_readiness=False,reason=str(error)[:256]);code=2
    print(json.dumps(result,allow_nan=False,sort_keys=True));return code


if __name__=='__main__':raise SystemExit(main())
