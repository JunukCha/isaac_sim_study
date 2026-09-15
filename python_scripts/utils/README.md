# Utilities

[한국어](README.ko.md) · [Back to root](../../README.md)

Shared helpers for launching livestream examples and diagnosing their network configuration.

## `live_stream.py`

Creates a headless `SimulationApp`, enables `omni.kit.livestream.app`, and configures a 1280 × 720 real-time path-traced stream. Import it before most other Isaac Sim modules:

```python
from utils.live_stream import simulation_app
```

Importing the module immediately launches `SimulationApp`.

## `check_port.py`

Prints environment entries containing `49100`, `47998`, or `PUBLIC_IP`:

```bash
python3 python_scripts/utils/check_port.py
```

Despite its name, this script does **not** inspect listening sockets. It helps verify whether livestream-related port or public-IP values are present in the process environment.

To continuously inspect the actual listening TCP/UDP sockets on Linux, run:

```bash
watch -d -n 0.1 "ss -lntup | grep -E '49100|47998'"
```

- `watch -n 0.1` reruns the command every 0.1 seconds.
- `-d` highlights changes between updates.
- `ss -lntup` lists listening TCP sockets and UDP sockets, including process information when permitted.
- `grep -E` keeps entries involving ports `49100` or `47998`.

If no output appears, neither port is currently shown by `ss`. Process details may require elevated permissions, and `watch`/`ss` must be installed in the Linux environment.
