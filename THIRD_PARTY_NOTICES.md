# Third-party notices

BalaX is licensed under the GNU Affero General Public License v3.0 (see `LICENSE`). The firmware images in `web/fw/`, and any firmware or host preview built from this repository, also contain the third-party material below.

## ESP-IDF

Firmware images are built with [ESP-IDF v6.1.1](https://github.com/espressif/esp-idf/tree/v6.1.1). Espressif's source and binary libraries are licensed under the [Apache License 2.0](https://github.com/espressif/esp-idf/blob/v6.1.1/LICENSE). Bundled third-party components and their licenses are listed in [ESP-IDF's copyright notices](https://github.com/espressif/esp-idf/blob/v6.1.1/docs/en/COPYRIGHT.rst).

## Adafruit GFX Library 5×7 font

The status screen draws text with the 5×7 font from `esp32-tools/tools` (`components/gfx/include/gfx/font5x7.h`), which contains glyph data from `glcdfont.c` in the [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library).

```text
Software License Agreement (BSD License)

Copyright (c) 2012 Adafruit Industries.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
```
