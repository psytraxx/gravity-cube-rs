# Hardware Setup Guide - Gravity Cube Firmware

## Pin Assignments

### BMI160 IMU (SPI Interface)

The BMI160 accelerometer/gyroscope sensor connects via SPI:

| Pin Function | ESP32 GPIO | Description |
|--------------|------------|-------------|
| **MOSI** (Master Out Slave In) | GPIO 23 | Data from ESP32 to BMI160 |
| **MISO** (Master In Slave Out) | GPIO 19 | Data from BMI160 to ESP32 |
| **SCLK** (SPI Clock) | GPIO 18 | SPI clock signal |
| **CS** (Chip Select) | GPIO 5 | BMI160 chip select (active low) |
| **VCC** | 3.3V | Power supply |
| **GND** | GND | Ground |

**SPI Configuration:**
- Frequency: 1 MHz
- Mode: Mode 0 (CPOL=0, CPHA=0)
- Bus: SPI2 (VSPI)

### WS2812B LEDs (RMT Interface)

The WS2812B addressable LEDs connect via the ESP32's RMT peripheral:

| Pin Function | ESP32 GPIO | Description |
|--------------|------------|-------------|
| **DATA** | GPIO 37 | WS2812B data signal |
| **VCC** | 5V | Power supply (external recommended) |
| **GND** | GND | Ground |

**LED Configuration:**
- Total LEDs: 384 (6 panels × 64 LEDs per panel)
- Protocol: WS2812B timing via RMT
- Data Rate: 800 kHz
- Color Order: GRB

## Wiring Diagram

```
ESP32-S3                    BMI160 IMU
┌─────────────┐            ┌──────────┐
│             │            │          │
│  GPIO 23 ───┼────────────┼─ MOSI   │
│  GPIO 19 ───┼────────────┼─ MISO   │
│  GPIO 18 ───┼────────────┼─ SCLK   │
│  GPIO 5  ───┼────────────┼─ CS     │
│  3.3V    ───┼────────────┼─ VCC    │
│  GND     ───┼────────────┼─ GND    │
│             │            │          │
└─────────────┘            └──────────┘

ESP32-S3                    WS2812B LED Strip
┌─────────────┐            ┌──────────┐
│             │            │          │
│  GPIO 37 ───┼────────────┼─ DIN    │
│  GND     ───┼────────────┼─ GND    │
│             │            │          │
└─────────────┘            │  5V ─────┼─ External PSU
                           │          │
                           └──────────┘
```

## Power Considerations

### ESP32-S3
- Operating Voltage: 3.3V
- Can be powered via USB or external regulator

### BMI160 IMU
- Operating Voltage: 2.4V - 3.6V (3.3V compatible)
- Current Draw: ~650μA (normal mode)
- Power from ESP32 3.3V pin: ✓ Safe

### WS2812B LEDs
- Operating Voltage: 4.5V - 5.5V
- Current Draw per LED: ~60mA (at full white)
- Total for 384 LEDs: Up to **23A** at full brightness!

**⚠️ IMPORTANT:** The WS2812B LEDs require a separate 5V power supply capable of delivering sufficient current. **Do NOT** power them from the ESP32's 5V pin!

**Recommended Power Setup:**
- Use a 5V 25-30A power supply for the LEDs
- Connect ESP32 GND to LED strip GND (common ground)
- Add a large capacitor (1000µF, 16V) across LED power rails
- Add a 470Ω resistor between ESP32 GPIO37 and LED DIN

## Software Configuration

The pin assignments are configured in `src/bin/main.rs`:

```rust
// BMI160 SPI pins
let spi_bus = Spi::new(peripherals.SPI2, Hertz(1_000_000))
    .with_sck(peripherals.GPIO18)     // Clock
    .with_mosi(peripherals.GPIO23)    // Data Out
    .with_miso(peripherals.GPIO19);   // Data In

let cs = Output::new(peripherals.GPIO5, Level::High);  // Chip Select

// WS2812 RMT pin
// GPIO 37 configured for RMT output
```

## Testing Individual Components

### Test 1: BMI160 IMU

To verify the IMU is working:

1. Flash the firmware
2. Monitor serial output
3. Look for: `✓ BMI160 IMU initialized successfully`
4. Check for gravity readings in the main loop

**Troubleshooting:**
- If initialization fails, check wiring
- Verify 3.3V power supply
- Ensure SPI pins are correctly connected
- Try adding pull-up resistors (4.7kΩ) on MOSI, MISO, SCLK

### Test 2: WS2812B LEDs

**Note:** Current implementation uses a stub driver. Full RMT implementation pending esp-hal-smartled compatibility.

When properly implemented, you should see:
- All 384 LEDs lighting up
- Particles responding to cube orientation
- Smooth animation at ~28 FPS (35ms delay)

##Firmware Status

### ✅ Implemented
- [x] Hexagonal architecture (Domain/Ports/Adapters)
- [x] BMI160 SPI driver integration
- [x] Physics simulation engine
- [x] Panel mapping (3D → 2D conversion)
- [x] Embassy async runtime
- [x] Pin assignments configured

### ⚠️ Pending
- [ ] Complete RMT driver for WS2812 (waiting for esp-hal-smartled v1.0 compatibility)
- [ ] Hardware testing and calibration
- [ ] IMU axis calibration for your specific mounting
- [ ] Color/brightness configuration

## Building and Flashing

1. **Install toolchain:**
   ```bash
   # Install Rust ESP toolchain
   espup install
   ```

2. **Build:**
   ```bash
   cd firmware
   cargo build --release
   ```

3. **Flash:**
   ```bash
   cargo run --release
   ```

4. **Monitor:**
   ```bash
   cargo run --release
   # Serial output will show initialization and loop status
   ```

## Next Steps

1. **Resolve esp-hal-smartled compatibility** - Update to matching version or implement custom RMT driver
2. **Test BMI160** - Verify accelerometer readings
3. **Calibrate IMU orientation** - Adjust axis swapping in `bmi160.rs` if needed
4. **Test LEDs** - Once RMT driver is working
5. **Tune parameters** - Adjust particle count, velocity, colors in `Config`

## Troubleshooting

| Issue | Solution |
|-------|----------|
| BMI160 not detected | Check SPI wiring, verify 3.3V power |
| LEDs not lighting | Verify 5V power, check data line, add level shifter if needed |
| Compilation errors | Update esp-hal-smartled to compatible version |
| Flickering LEDs | Add capacitor to power supply, check ground connections |
| Incorrect gravity direction | Adjust axis swapping in BMI160 adapter |

## References

- [ESP32-S3 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)
- [BMI160 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi160-ds000.pdf)
- [WS2812B Datasheet](https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf)
- [esp-hal Documentation](https://docs.esp-rs.org/esp-hal/)
