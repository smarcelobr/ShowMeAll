run# Build project
$ pio run

# Upload firmware
$ pio run --target upload

# Build specific environment
$ pio run -e nodemcuv2

# Upload firmware for the specific environment
$ pio run -e nodemcuv2 --target upload

# Clean build files
$ pio run --target clean