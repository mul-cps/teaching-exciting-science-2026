#!/usr/bin/env python3
"""Run a bounded program through the existing shared ROS program endpoint."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from workshop.blockly_client import run_program
from tools.run_blockly_station import strict_json


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('program',nargs='?',type=Path)
    parser.add_argument('--stdin-json',action='store_true')
    modes=parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--physical',action='store_true'); modes.add_argument('--simulation',action='store_true')
    parser.add_argument('--timeout',type=float,default=190.)
    args=parser.parse_args()
    if args.stdin_json:
        body=strict_json(sys.stdin.read(65537))
        if not isinstance(body,dict) or set(body)!={'program'}: parser.error('stdin must contain only program')
        program=body['program']
    elif args.program: program=strict_json(args.program.read_text())
    else: parser.error('provide a program JSON file or --stdin-json')
    try:
        result=run_program(program,physical=args.physical,timeout=args.timeout,wrapper_bridge=False)
        print(json.dumps(result,allow_nan=False))
    except BaseException as error:
        print(str(error),file=sys.stderr); return 1
    return 0


if __name__=='__main__': raise SystemExit(main())
