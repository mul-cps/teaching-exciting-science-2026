"""INSTRUCTOR ONLY. Requires ev3dev2 + RPyC classic service on a trusted LAN.

Hardware NOT validated: never use in a student station without bench testing.
"""
import argparse


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--host", required=True)
    p.add_argument("--motor", default="outA")
    p.add_argument("--seconds", type=float, default=0.3)
    p.add_argument("--i-understand-this-moves-hardware", action="store_true")
    a = p.parse_args()
    if not a.i_understand_this_moves_hardware:
        p.error("Requires explicit movement opt-in")
    if not 0 < a.seconds <= 1:
        p.error("Duration must be 0-1 seconds")
    import rpyc
    c = rpyc.classic.connect(a.host, port=18812)
    m = c.modules["ev3dev2.motor"].LargeMotor(a.motor)
    try:
        m.on_for_seconds(10, a.seconds, brake=True)
    finally:
        m.off(brake=True)
        c.close()


if __name__ == "__main__":
    main()
