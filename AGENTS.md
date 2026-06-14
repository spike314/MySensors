# AGENTS.md – MySensors Library

Home Automation IoT sensor mesh framework. Multi-platform Arduino library (nRF24, RFM69, RFM95, RS485 radios on AVR, ESP32, ESP8266, NRF5x, SAMD, STM32, Teensyduino).

**Active branch on this checkout**: Check `git branch` (may be feature branch, not main).

## Spy Node Project (This Branch)

**Purpose**: Modify SX126x radio driver to support "listen-only" Spy node mode for debugging/traffic analysis on SX126x networks. The node will report all received traffic via SX126x_Spy sketch.

**Not for production use** – debugging and analysis only.

### Task Scope

Modify the SX126x radio driver at `hal/transport/SX126x/` to:
1. **Receive all SX126x traffic** (including broadcast and addressed messages not destined for the node)
2. **Disable transmission** (Spy nodes are receive-only, not responders)
3. **Report raw packet data** via serial debug output for analysis
4. **Maintain RSSI/SNR metadata** for signal quality diagnostics

### Key Files for This Task

| File | Purpose |
|------|---------|
| `hal/transport/SX126x/MyTransportSX126x.cpp` | Transport layer interface (wraps driver calls) |
| `hal/transport/SX126x/driver/SX126x.cpp` | SX126x hardware driver (low-level radio control) |
| `hal/transport/SX126x/driver/SX126x.h` | SX126x driver public API |
| `examples/SX126x_Spy/SX126x_Spy.ino` | Test sketch (enable MY_RADIO_SX126x, use for serial output) |

### Implementation Pattern

The SX126x driver uses three key functions:
- `SX126x_handle()` – Polls radio state, updates internal status
- `SX126x_packetAvailable()` – Returns true if packet received
- `SX126x_getData()` – Reads received packet data into buffer

For Spy mode:
1. **Radio Configuration**: Set to RX-only continuous listening (no address filtering, accept all valid frames)
2. **Packet Capture**: Call `SX126x_getData()` for all packets regardless of destination address
3. **Debug Output**: Print raw packet bytes, RSSI, SNR via serial debug (`MY_DEBUG` output)
4. **Transmission Prevention**: Skip ACK generation and outbound transmission logic

### Build & Test

Compile test sketch for target SX126x board:
```bash
# Via Arduino IDE: Open examples/SX126x_Spy/SX126x_Spy.ino
# Edit to enable: #define MY_RADIO_SX126x
#                 #define MY_DEBUG (for verbose output)
# Select board, compile, upload

# Via Arduino CLI (CI):
arduino-cli compile -b stm32:stm32:stm32wle5xx_generic examples/SX126x_Spy/SX126x_Spy.ino
```

### Verification

After upload to SX126x-equipped board:
1. Open serial monitor at configured baud rate (default 115200 from SX126x_Spy.ino)
2. Observe debug messages showing captured radio traffic:
   - Packet source/destination addresses
   - RSSI and SNR values
   - Raw payload bytes
3. Confirm no transmission occurs (Spy node remains silent on air)

## Build & Test

### Linux Gateway Build
```bash
./configure --no-clean              # REQUIRED: generates Makefile.inc (auto-detects SoC)
make clean && make all              # Build gateway + Arduino libraries
make install --prefix=/usr/local    # Install gateway (systemd or sysvinit auto-configured)
```

**Critical**: `Makefile.inc` is generated and should never be committed. The configure script auto-detects Raspberry Pi SoC (BCM2835/2836/2837/2711) and CPU flags.

### Arduino Builds
- Uses Arduino CLI (`/opt/arduino-cli/arduino-cli` on CI; Arduino IDE locally)
- Test sketches (`.ino` files) in `tests/fast/` and `tests/nightly/` are **compilation only**
- No runtime execution in CI; hardware platforms are separate (external repos per platform)

### CI Pipeline
- **System**: Jenkins with Groovy (`.ci/pipeline.groovy`)
- **Stages**: Butler (PR style), Doxygen, Cppcheck, Linux gateway variants, Arduino platform builds (run in parallel)
- **PR Failures**: Slack + email to PR author
- **Branch Failures**: Slack + email to `builds@mysensors.org`

## Repository Structure

| Path | Purpose |
|------|---------|
| `core/` | Protocol, messaging, OTA, crypto, LED indication |
| `hal/architecture/` | MCU HAL (Linux, AVR, ESP32, NRF5, SAMD, STM32) |
| `hal/transport/` | Radio drivers (nRF24, RFM69, RFM95, RS485) |
| `hal/crypto/` | Hardware crypto (STM32, ATSHA204) and generic drivers |
| `drivers/` | Third-party deps (PubSubClient, SPIFlash, etc.) |
| `examples/` | User sketches |
| `examples_linux/` | Linux gateway examples |
| `tests/` | Compilation tests (fast/ and nightly/) |
| `.ci/` | Jenkins Groovy pipeline |
| `.mystools/` | Development tools (astyle, cppcheck) with git aliases |

## Code Style & Formatting

- **Tool**: `.mystools/bootstrap-dev.sh` sets up pre-commit hooks and git aliases
- **Run after setup**: `git astyle`, `git cppcheck` (or with `--cached` for staged files)
- **EditorConfig**: `.editorconfig` enforces tabs (2-char width), LF, UTF-8, final newline
- **Required**: Install `astyle ≥ 3.1` and `cppcheck ≥ 2.1` for pre-commit validation

## Build Quirks

### Gateway Configuration
- `--soc=BCM2835` (Pi Zero/1), `--soc=BCM2837` (Pi 3), `--soc=BCM2711` (Pi 4)
- `--my-gateway=serial|ethernet|mqtt` (default: ethernet)
- `--my-transport=rf24|rfm69|rfm95|rs485` (default: rf24)
- `--my-debug=enable|disable` (default: enable)
- Pass flags via `--extra-cxxflags` for compile-time options (e.g., `MY_RX_MESSAGE_BUFFER_SIZE`, `MY_RF24_DATARATE`)

### Arduino Multi-Config
- Test sketches use different feature combinations to catch incompatibilities
- Platform hardware repos cloned fresh on each CI run (not cached)

## Key Files & Entry Points

- `MySensors.h` – Main include (MCU sketch entry point)
- `MyConfig.h` – User-facing configuration template
- `core/MyMessage.h/.cpp` – Protocol format
- `hal/architecture/*/MyHw*.cpp` – Platform HAL implementations
- `hal/transport/*/MyTransport*.cpp` – Radio driver implementations
- `.ci/pipeline.groovy` – Build stage definitions

## Branches & Release Flow

- `master` – Stable releases
- `development` – Active development (target for PRs)
- Feature branches – Work locally, PR to `development` first, backport to `master` if needed

## Testing Prerequisites

- **Arduino CLI** for platform builds (CI uses `/opt/arduino-cli/arduino-cli`)
- **External hardware repos** cloned fresh per platform per CI run (ArduinoHwAVR, ArduinoHwSAMD, ArduinoHwNRF5, etc.)
- **Tests are compilation-only** (no hardware execution in CI)
- Nightly suite runs less frequently (longer times)

## No Codegen, Migrations, Dev Server

- Pure library; no database, background jobs, or runtime config loading
- No generated code at build time (only compile-time macros)
- Doxygen documentation generated from source comments
- All config via `MyConfig.h` or build flags

## Common Pitfalls for Agents

1. **Forgetting `./configure` before `make`** → Will fail with missing `Makefile.inc`
2. **Committing `Makefile.inc`** → Never commit generated build files
3. **Assuming tests run on hardware** → CI tests are compilation-only
4. **Missing `astyle` or `cppcheck`** → Pre-commit hooks fail without these tools
5. **Wrong SoC for gateway** → Use `./configure --soc=BCM2711` for Pi 4, `BCM2837` for Pi 3
6. **Not reading `.ci/pipeline.groovy`** → Source of truth for which platforms, stages, and parallelization
