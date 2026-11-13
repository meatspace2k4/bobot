"""python -m beni probe | record OUT.hevc [--secs N] | action NAME   [--host H --port P --bind IP]"""
import argparse, sys, time

from . import Beni, ACTIONS


def main():
    ap = argparse.ArgumentParser(prog="python -m beni")
    ap.add_argument("--host", default="192.168.7.1")
    ap.add_argument("--port", type=int, default=9007)
    ap.add_argument("--bind")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("probe", help="connect, report video rate for 3 s")
    r = sub.add_parser("record", help="save raw H.265")
    r.add_argument("out")
    r.add_argument("--secs", type=float, default=10)
    a = sub.add_parser("action", help="send one action pulse")
    a.add_argument("name", choices=sorted(ACTIONS))
    args = ap.parse_args()

    with Beni(args.host, args.port, args.bind) as b:
        if args.cmd == "probe":
            n = sum(1 for _ in b.frames(seconds=3))
            print(f"connected; {n / 3:.0f} video frames/s")
        elif args.cmd == "record":
            with open(args.out, "wb") as f:
                n = 0
                for au in b.frames(seconds=args.secs):
                    f.write(au); n += 1
            print(f"{n} frames -> {args.out}")
        elif args.cmd == "action":
            b.action(args.name)
            time.sleep(0.5)
            print(f"sent {args.name}")


if __name__ == "__main__":
    sys.exit(main())
