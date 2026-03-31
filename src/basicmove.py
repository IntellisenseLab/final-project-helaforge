import contextlib
import select
import serial
import sys
import termios
import tty

HEADER = bytearray([0xAA, 0x55, 0x06, 0x01, 0x04])
PORT = "/dev/ttyUSB0"
BAUD = 115200
LINEAR_STEP = 200      # mm/s
ANGULAR_STEP = 200     # 0.1 rad/s equivalent per Kobuki docs


def build_command(linear: int, angular: int) -> bytearray:
	linear_bytes = int(linear).to_bytes(2, byteorder="little", signed=True)
	angular_bytes = int(angular).to_bytes(2, byteorder="little", signed=True)
	return HEADER + linear_bytes + angular_bytes


def stop_command() -> bytearray:
	return build_command(0, 0)


def instructions() -> str:
	return (
		"Teleop controls (press q to quit):\n"
		"  w: forward\n"
		"  s: backward\n"
		"  a: rotate left\n"
		"  d: rotate right\n"
		"  space: stop"
	)


def get_key(timeout: float = 0.1) -> str | None:
	ready, _, _ = select.select([sys.stdin], [], [], timeout)
	if ready:
		return sys.stdin.read(1)
	return None


@contextlib.contextmanager
def raw_stdin():
	fd = sys.stdin.fileno()
	old_settings = termios.tcgetattr(fd)
	tty.setcbreak(fd)
	try:
		yield
	finally:
		termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main() -> None:
	if not sys.stdin.isatty():
		print("Teleop requires an interactive terminal")
		return

	command_map = {
		"w": build_command(LINEAR_STEP, 0),
		"s": build_command(-LINEAR_STEP, 0),
		"a": build_command(0, ANGULAR_STEP),
		"d": build_command(0, -ANGULAR_STEP),
		" ": stop_command(),
	}

	with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
		print(f"Connected to Kobuki on {PORT} @ {BAUD} baud")
		ser.write(stop_command())
		print(instructions())

		with raw_stdin():
			while True:
				key = get_key()
				if not key:
					continue

				key = key.lower()
				if key == "q":
					break

				cmd = command_map.get(key)
				if cmd:
					ser.write(cmd)
					ser.flush()

		ser.write(stop_command())
		ser.flush()
		print("Stopped")


if __name__ == "__main__":
	try:
		main()
	except KeyboardInterrupt:
		print("\nInterrupted, stopping robot…")
		with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
			ser.write(stop_command())

