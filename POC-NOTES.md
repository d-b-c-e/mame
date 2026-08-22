# POC branch notes (poc/quadlog)

This branch carries the Cruis'n POC: the DIJOYSTATE2 base patch plus the
renderer-replacement instrumentation and in-process GL renderer, all in
`src/mame/midway/midvunit_v.cpp` (+ one visibility change in `midvunit.h`).

**The project home is `E:\Source\cruisn-collection`** — read its
`.claude/session-notes.md`, `CLAUDE.md` and `results/RESULTS.md` before
working here. Key rules: `midvunit_gl_shaders.h` is GENERATED from
`cruisn-collection/gpu/renderer.py` (never hand-edit); build with the command in
cruisn-collection's CLAUDE.md (E:\msys64, OS=Windows_NT exported inside the shell);
after committing here, refresh `cruisn-collection/patch/vunit-poc-patches.patch`
via `git format-patch --stdout 6f55ed93..HEAD`. The FFB Arcade Plugin files
beside vunit.exe are untracked on purpose.
