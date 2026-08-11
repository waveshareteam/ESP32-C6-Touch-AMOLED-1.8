# Release tools

[简体中文](README_ZH.md)

## Download CI artifacts

Use download_artifacts.py with Python 3.10 or newer:

~~~bash
python3 releases/download_artifacts.py --run-id RUN_ID --clean
~~~

Omit --run-id to select the latest successful workflow run for the current
branch. Use --artifact repeatedly for exact artifact names or --pattern for a
glob. GitHub CLI authentication or GH_TOKEN can be used for private or
rate-limited requests.

Downloaded files are extracted below releases/downloads and are ignored by
Git.

## Package firmware

package_firmware.py is called by GitHub Actions after a successful build. For
ESP-IDF it reads flasher_args.json. For Arduino it reads exported binary files.
The output archive contains a manifest, flash arguments, flash helpers, and
the required binaries.

Generated archives are written below release-artifacts or releases/dist and
are ignored by Git.
