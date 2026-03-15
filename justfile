# AutoLee build & test commands

# List available recipes
default:
    @just --list

# Install dependencies (PlatformIO + libraries)
setup:
    pipx install platformio
    pio pkg install -e esp32c6
    pio pkg install -e native

# Run all native unit tests
test:
    pio test -e native -v

# Run a single test suite (e.g. just test-one motor_fsm)
test-one suite:
    pio test -e native -f "test_native/test_{{suite}}" -v

# Build firmware for ESP32-C6
build:
    pio run -e esp32c6

# Clean build artifacts
clean:
    pio run -t clean

# Upload firmware to connected ESP32-C6
flash:
    pio run -e esp32c6 -t upload

# Open serial monitor (115200 baud)
monitor:
    pio device monitor -b 115200

# Build + flash + monitor
run: build flash monitor

# OTA upload to device on network (default: autolee.local)
ota host="autolee.local":
    pio run -e esp32c6 -t upload --upload-port {{host}}

# Show project file structure and line counts
files:
    @echo "=== Source ==="
    @wc -l AutoLee.ino include/*.h | sort -n
    @echo ""
    @echo "=== Tests ==="
    @wc -l test/test_native/*/test_*.cpp | sort -n

# Run mock web UI server (no hardware needed)
mock:
    python3 tools/mock_server.py

# Show git log for this project
log:
    git log --oneline -20
