#!/usr/bin/env python3
"""Create an all-clear binary PGM matching the CIT 3F navigation map."""

import argparse
from pathlib import Path


def create_white_pgm(path: Path, width: int, height: int) -> None:
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    path.write_bytes(header + bytes([255]) * (width * height))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--width", type=int, default=4000)
    parser.add_argument("--height", type=int, default=4000)
    args = parser.parse_args()
    create_white_pgm(args.output, args.width, args.height)


if __name__ == "__main__":
    main()
