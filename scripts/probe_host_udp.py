#!/usr/bin/env python3
"""Small TCP-free UDP probe for Docker Desktop host-network validation."""

import argparse
import json
import socket
import sys
import time
from pathlib import Path


def listen(args):
    started = time.monotonic()
    result = {
        "bind": args.bind,
        "port": args.port,
        "received": False,
        "sender": None,
        "elapsed_seconds": None,
    }
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((args.bind, args.port))
        sock.settimeout(args.timeout)
        payload, sender = sock.recvfrom(65535)
        result["payload"] = payload.decode("utf-8", errors="replace")
        result["sender"] = [sender[0], sender[1]]
        result["received"] = payload == args.token.encode("utf-8")
    except socket.timeout:
        result["error"] = "timeout"
    except OSError as exc:
        result["error"] = str(exc)
    finally:
        sock.close()
        result["elapsed_seconds"] = round(time.monotonic() - started, 6)

    if args.result_file:
        output = Path(args.result_file)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result["received"] else 1


def send(args):
    payload = args.token.encode("utf-8")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for _ in range(args.repeat):
            sock.sendto(payload, (args.host, args.port))
            if args.interval > 0:
                time.sleep(args.interval)
    finally:
        sock.close()
    return 0


def make_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    listener = subparsers.add_parser("listen", help="listen for an exact token")
    listener.add_argument("--bind", default="0.0.0.0")
    listener.add_argument("--port", type=int, required=True)
    listener.add_argument("--token", required=True)
    listener.add_argument("--timeout", type=float, default=5.0)
    listener.add_argument("--result-file")
    listener.set_defaults(func=listen)

    sender = subparsers.add_parser("send", help="send an exact token")
    sender.add_argument("--host", default="127.0.0.1")
    sender.add_argument("--port", type=int, required=True)
    sender.add_argument("--token", required=True)
    sender.add_argument("--repeat", type=int, default=3)
    sender.add_argument("--interval", type=float, default=0.05)
    sender.set_defaults(func=send)
    return parser


def main():
    args = make_parser().parse_args()
    if not 1 <= args.port <= 65535:
        raise SystemExit("port must be in 1..65535")
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
