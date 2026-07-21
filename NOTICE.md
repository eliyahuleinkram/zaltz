# Notices

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
