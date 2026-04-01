# GNC-Airbrakes

Firmware for the IREC Airbrakes project, targeting Teensy 4.1.

## Prerequisites

- macOS, Linux, or Windows with WSL
- Git
- Homebrew (macOS) for installing dependencies
- [usbipd-win](https://github.com/dorssel/usbipd-win/releases) (Windows/WSL only, for USB passthrough)
- VS Code with C/C++ extension (optional, for IDE support)

## Quick Start

1. **Clone the repository**
   ```bash
   git clone https://github.com/XanLot/GNC-Airbrakes.git
   cd GNC-Airbrakes
   ```

2. **Install the toolchain**
   ```bash
   make install
   ```
   This downloads and installs:
   - ARM GNU Toolchain (v14.2.rel1) - cross-compiler for Cortex-M7
   - tycmd - command-line Teensy uploader

3. **Build the firmware**
   ```bash
   make build
   ```

4. **Upload to Teensy 4.1**

   ```bash
   make upload
   ```
   If upload fails, press the button on the Teensy and retry.

   **If you are on Windows Subsystem for Linux (WSL):** You must first install [usbipd-win](https://github.com/dorssel/usbipd-win/releases) and run `make wsl` to attach the Teensy USB device to WSL.

## Make Targets

| Target | Description |
|--------|-------------|
| `make install` | Install ARM toolchain and tycmd uploader |
| `make build` | Compile firmware and generate .hex file |
| `make wsl` | Attach Teensy USB to WSL (Windows/WSL only) |
| `make upload` | Build and flash to Teensy 4.1 |
| `make clean` | Remove all build artifacts |
| `make kill` | Stop running firmware (enter bootloader) |
| `make restart` | Restart the Teensy |
| `make cdb` | Generate compile_commands.json for clangd |
| `make help` | Show available targets |

## VS Code Setup

The project is configured for VS Code with the Microsoft C/C++ extension.

1. Install the [C/C++ extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
2. Run `make install` to download the ARM toolchain
3. Open the project folder in VS Code
4. IntelliSense will automatically configure using `.vscode/c_cpp_properties.json`

## Project Structure

```
GNC-Airbrakes/
├── Makefile                    # Build system
├── src/                        # Application source code
│   └── main.cpp                # Entry point (setup/loop)
├── teensy4/                    # Teensy 4.1 Arduino core
│   ├── Arduino.h
│   ├── imxrt1062_t41.ld        # Linker script
│   └── ...
├── libraries/                  # Third-party Arduino libraries
├── tools/
│   ├── install_compiler.sh     # ARM toolchain installer
│   └── install_tytools.sh      # tycmd installer
├── build/                      # Build outputs (generated)
└── .vscode/                    # VS Code configuration
    ├── settings.json
    └── c_cpp_properties.json
```

## Adding Libraries

Place Arduino-compatible libraries in the `libraries/` folder. They will be automatically included in the build.

## Compiler Settings

- **CPU**: Cortex-M7 @ 600MHz
- **C++ Standard**: GNU++23
- **Optimization**: -O2
- **Float ABI**: Hard (FPv5-D16)

## Troubleshooting

**Upload fails with "no device found"**
- Press the button on the Teensy to enter bootloader mode
- Ensure the Teensy is connected via USB
- Run `make upload` again

**IntelliSense shows errors**
- Ensure you've run `make install` to download the toolchain
- Reload VS Code window after installing

**Build fails with "compiler not found"**
- Run `make install` to download the ARM toolchain
