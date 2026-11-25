#!/usr/bin/env python3
"""Receive-only USB/UART -> raw UBX TCP fanout; never configures the receiver.

One process owns the serial port. Slow clients are disconnected instead of
accumulating old GPS timestamps. Read-only identity/RF/time-pulse polls expose
diagnostics; clients cannot write configuration or reset commands.
"""
import argparse
import json
import selectors
import signal
import socket
import time

import serial


def poll_message(cls, mid, data=b""):
    payload = bytes((cls, mid, len(data) & 255, len(data) >> 8)) + data
    a = b = 0
    for value in payload:
        a = (a + value) & 255
        b = (b + a) & 255
    return b'\xb5\x62' + payload + bytes((a, b))


# CFG-TP-* TIMEPULSE0 items, mirrored in player/ubx_receiver.cpp. ZED-F9K
# firmware NAKs CFG-TP5, so the player falls back to this RAM-layer readback.
TP1_CONFIG_KEYS = (
    0x30050001, 0x40050002, 0x40050003, 0x40050004, 0x40050005, 0x40050006,
    0x10050007, 0x10050008, 0x10050009, 0x1005000a, 0x1005000b, 0x2005000c,
    0x20050023, 0x40050024, 0x40050025, 0x5005002a, 0x5005002b, 0x20050030,
)


def diagnostic_polls(pps=False):
    """Read-only UBX queries: no configuration, reset, or pulse changes."""
    result = bytearray()
    for cls, mid in ((0x0a, 0x04), (0x27, 0x03), (0x0a, 0x38), (0x0d, 0x01)):
        result.extend(poll_message(cls, mid))
    if pps:
        # Readbacks only: TIMEPULSE0 configuration and current/future leaps.
        # CFG-TP5 precedes CFG-VALGET so a NAK arrives before the fallback.
        result.extend(poll_message(0x06, 0x31, b'\x00'))
        valget = bytes(4) + b"".join(k.to_bytes(4, "little") for k in TP1_CONFIG_KEYS)
        result.extend(poll_message(0x06, 0x8b, valget))
        result.extend(poll_message(0x01, 0x26))
    return bytes(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5019)
    parser.add_argument("--pps", action="store_true",
                        help="poll next-pulse labels at 10 Hz and TP5/VALGET/leap readbacks; no configuration writes")
    args = parser.parse_args()
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    clients = {}
    counters = dict(serial_bytes=0, connections=0, slow_clients=0)
    with serial.Serial(args.serial, args.baud, timeout=0, exclusive=True) as source, \
            socket.socket() as server, selectors.DefaultSelector() as selector:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.host, args.port))
        server.listen(4)
        server.setblocking(False)
        selector.register(server, selectors.EVENT_READ)
        selector.register(source.fileno(), selectors.EVENT_READ)

        def disconnect(client):
            selector.unregister(client)
            clients.pop(client)
            client.close()

        print(json.dumps(dict(event="ready", serial=args.serial,
                              endpoint=f"{args.host}:{args.port}",
                              pps_polling=args.pps,
                              receiver_configuration_changed=False)), flush=True)
        poll_at = 0
        pulse_poll_at = 0
        try:
            while running:
                if time.monotonic() >= poll_at:
                    source.write(diagnostic_polls(args.pps))
                    poll_at = time.monotonic() + 5
                if args.pps and time.monotonic() >= pulse_poll_at:
                    source.write(poll_message(0x0d, 0x01))
                    pulse_poll_at = time.monotonic() + .1
                for key, events in selector.select(0.1):
                    if key.fileobj is server:
                        client, _ = server.accept()
                        client.setblocking(False)
                        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                        clients[client] = bytearray()
                        selector.register(client, selectors.EVENT_READ)
                        counters["connections"] += 1
                    elif key.fileobj == source.fileno():
                        chunk = source.read(16384)
                        counters["serial_bytes"] += len(chunk)
                        for client, pending in list(clients.items()):
                            pending.extend(chunk)
                            if len(pending) > 65536:
                                counters["slow_clients"] += 1
                                disconnect(client)
                            else:
                                selector.modify(client, selectors.EVENT_READ |
                                                selectors.EVENT_WRITE)
                    else:
                        client = key.fileobj
                        if client not in clients:
                            continue
                        try:
                            if events & selectors.EVENT_READ:
                                # EOF or any client write closes this read-only stream.
                                client.recv(1024)
                                disconnect(client)
                                continue
                            if events & selectors.EVENT_WRITE:
                                pending = clients[client]
                                del pending[:client.send(pending)]
                                if not pending:
                                    selector.modify(client, selectors.EVENT_READ)
                        except (ConnectionError, OSError):
                            disconnect(client)
        finally:
            for client in list(clients):
                disconnect(client)
            print(json.dumps(dict(event="stopped", **counters)), flush=True)


if __name__ == "__main__":
    main()
