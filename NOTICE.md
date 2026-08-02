# Notices

zaltz — Copyright (C) 2026 Eliyahu Moshe Leinkram.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU Affero General Public License as published by the Free
Software Foundation, either version 3 of the License, or (at your option) any
later version. It is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License in
[LICENSE](LICENSE) for more details.

## Ancestry

zaltz is a derivative work of **superdough**, the sampler/synthesizer layer of
the [Strudel](https://strudel.cc) project (© Strudel contributors,
AGPL-3.0-or-later). The engine re-implements superdough's control semantics in
C — voice parameters, envelopes, filters, orbit buses (reverb, delay, duck),
gain staging — and was developed against superdough's rendered output as the
reference oracle until the two matched. zaltz is therefore licensed
AGPL-3.0-or-later, the same terms as its ancestor.

Strudel itself is the JavaScript port of **TidalCycles** by Alex McLean and
contributors. zaltz contains no Tidal or Strudel pattern-language code — the
pattern layer stays upstream — but the control vocabulary it speaks descends
from that lineage, with gratitude.
