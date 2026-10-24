# Matter BMP280 validation report

Date: 2026-10-24
Target: ESP32-H2-DevKitM-1 (`/dev/ttyUSB0`)
Toolchain: ESP-IDF 5.3.2; esp-matter `release/v1.4.2` at `4cedd4ad40821933d6cc237e80ab6604f2953fb0`

## Result

The repaired normal image builds and links successfully. Its generated configuration contains SDA GPIO10 and SCL GPIO11. The exact artifact below was erased, flashed, hash-verified by esptool, and left running on the ESP32-H2 revision 0.1.

| Configuration | Size | SHA-256 |
|---|---:|---|
| Normal | 1,610,864 bytes (`0x189470`) | `9766d92ba97bfd517a6e56a527b9c65c9e2d7937a26886761f837a16e7d23e61` |
| OpenThread CLI + encrypted OTA | 1,718,192 bytes (`0x1a37b0`) | `8d8c7a980a5eb5add8729b61d5dd2558c3cd3ec7cbb404fa50c3db4b6ae55b49` |

## Reproduce

```bash
source "$HOME/.espressif/tools/activate_idf_v5.3.2.sh"
export PATH="$HOME/esp-matter/.host-python/bin:$PATH"
source "$HOME/esp-matter/export.sh"

cd "$HOME/IoT/matter_bmp280"
idf.py --ccache -B build-default \
  -DSDKCONFIG="$PWD/build-default/sdkconfig" \
  -DSDKCONFIG_DEFAULTS="$PWD/sdkconfig.defaults;$PWD/sdkconfig.defaults.esp32h2" \
  build
idf.py --ccache -B build-default \
  -DSDKCONFIG="$PWD/build-default/sdkconfig" \
  -DSDKCONFIG_DEFAULTS="$PWD/sdkconfig.defaults;$PWD/sdkconfig.defaults.esp32h2" \
  -p /dev/ttyUSB0 erase-flash flash monitor
```

For optional compile coverage, create an external throwaway RSA key, link it only for the build, and remove both afterward:

```bash
test_key="$(mktemp)"
openssl genrsa -out "$test_key" 2048
ln -s "$test_key" esp_image_encryption_key.pem
idf.py --ccache -B build-optional \
  -DSDKCONFIG="$PWD/build-optional/sdkconfig" \
  -DSDKCONFIG_DEFAULTS="$PWD/sdkconfig.defaults;$PWD/sdkconfig.defaults.esp32h2;$PWD/sdkconfig.defaults.optional" \
  build
unlink esp_image_encryption_key.pem
rm -f "$test_key"
```

The key filename is ignored by Git. Never commit a production OTA private key.

## Follow-up validation

- The default I2C wiring is SDA GPIO10 (J3 pin 4) and SCL GPIO11 (J3 pin 5). Both are exposed on the documented ESP32-H2-DevKitM-1 header and unused elsewhere in this application. Kconfig, generated normal/optional configurations, and README agree.
- Forced mode now initializes asleep, compares only persistent oversampling plus an allowed forced/sleep state, retriggers a conversion for every read, waits for completion, and then reads fresh raw values.
- I2C cleanup supports device-only, bus-only, fully initialized, and already-clean states. A handle is cleared only after ESP-IDF successfully releases that resource. Reinitialization retries unresolved cleanup and does not `memset()` retained handles away.
- With no BMP280 attached, four observed cycles logged GPIO10/GPIO11, created the I2C bus, received `ESP_ERR_NOT_FOUND`, successfully cleaned up the bus, left both Matter measurements null, and retried about ten seconds later. Matter and BLE commissioning remained active without a panic.

## Limits

No BMP280, Matter controller, or Thread border router was attached. Forced conversions, valid environmental values, sensor-only reset, deliberate partial-cleanup failure injection, final-fabric removal, and OTA transfer/decryption/rollback remain hardware/controller tests. Encrypted OTA validation here is build coverage only.

References: [ESP32-H2-DevKitM-1 header tables](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32h2/esp32-h2-devkitm-1/user_guide.html), [Bosch BMP280 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf).
