# Vendored from dbce-wheel-mod-toolkit

`force_model.h` and `force_profile.h` are **copies**, not sources. They come from
`E:\Source\dbce-wheel-mod-toolkit` (`native/forcemodel/`) and are pinned by the
VERSION file beside them.

Do not edit them here. Fix a shaping bug in the toolkit, cut a release, then
re-vendor. The toolkit's conformance test pins these headers against a C#
implementation of the same maths, and editing this copy silently breaks that.

Cruis'n uses the **shaper only**. There is no force model to run: the arcade
board hands over a finished signed motor byte, so `Model` is never constructed.

## Licensing note

MAME is GPL-3 and this file is compiled into `vunit.exe`, which makes the
combined binary GPL-3. Both repositories have the same owner, so this is a
choice rather than a conflict - but it is a real one, and it is why
`docs/adr/0001-ffb-output-backends.md` in cruisn-collection originally kept
Cruis'n off the toolkit.
