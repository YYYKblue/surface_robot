"""python3 -m under_control --serial-port /dev/serial/by-id/... --motion-enabled"""
import argparse
import asyncio
import logging
import signal

from .serial_worker import SerialWorker
from .server import Server


async def serve(args):
    worker = SerialWorker(args.serial_port, motion_enabled=args.motion_enabled)
    server = Server(worker, args.bind, args.port)
    stopped = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        asyncio.get_running_loop().add_signal_handler(sig, stopped.set)
    worker.start()
    try:
        await server.start()
        print(f"Underwater service listening on {args.bind}:{args.port}; motion={args.motion_enabled}", flush=True)
        await stopped.wait()
    finally:
        await server.close()
        await asyncio.to_thread(worker.close)


def main():
    parser = argparse.ArgumentParser(description="H7 underwater manual gateway (no ROS)")
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--serial-port", help="omit to inspect the UI without opening hardware")
    parser.add_argument("--motion-enabled", action="store_true")
    args = parser.parse_args()
    if args.motion_enabled and not args.serial_port:
        parser.error("--motion-enabled requires --serial-port")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    asyncio.run(serve(args))


if __name__ == "__main__":
    main()
