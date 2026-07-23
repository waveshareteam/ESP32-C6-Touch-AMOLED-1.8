# Contributing

Thank you for helping improve the ESP32-C6-Touch-AMOLED-1.8 examples.

This repository is customer-facing, so changes should keep the examples easy to build, easy to compare, and safe to run on the board.

## Before Opening a Pull Request

- Keep ESP-IDF examples compatible with the CI matrix unless an example documents a narrower requirement.
- Ensure the `Build Examples` workflow passes for changed ESP-IDF examples with target `esp32c6`.
- Keep Arduino changes in the matching V1 or V2 directory and ensure Arduino CI passes.
- Prefer managed ESP-IDF components in `main/idf_component.yml` instead of checking in component copies.
- Do not commit generated `build/`, `managed_components/`, `dependencies.lock`, local `sdkconfig`, or cache files.
- Update the example README when behavior, hardware requirements, menuconfig options, or expected output changes.

## Example Style

New ESP-IDF examples should be standalone projects under `examples/esp-idf/<number>_<name>/` and include:

- `README.md`
- `CMakeLists.txt`
- `main/CMakeLists.txt`
- `main/` source files
- `sdkconfig.defaults`

Use the learning order in [docs/EXAMPLES_GUIDE.md](docs/EXAMPLES_GUIDE.md): simple serial examples first, then board services, peripherals, audio, display, and LVGL. Keep board-specific low-level diagnostics in the `90_` range.

## Documentation Style

- Write public instructions with repository-relative paths.
- Avoid local machine paths, usernames, private network paths, or tool installation directories.
- Include the command sequence, required hardware, configuration notes, and expected result.
- Link to managed components or upstream projects when an example depends on them.

## CI

Pull requests that change files under `examples/esp-idf/` build the changed ESP-IDF examples. Changes to the workflow or discovery script build the full ESP-IDF example set. See [docs/CI.md](docs/CI.md) for details.
