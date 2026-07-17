from pathlib import Path
import platform
import sys


def main() -> None:
    """Verify that the shared repository is visible inside Docker."""
    print("Crazyflie project repository is mounted correctly.")
    print(f"Working directory: {Path.cwd()}")
    print(f"Python version: {sys.version}")
    print(f"Machine architecture: {platform.machine()}")
    print(f"Operating system: {platform.platform()}")


if __name__ == "__main__":
    main()

print("Modified from the host computer.")
