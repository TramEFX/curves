# Curves

An EQ-based effect sandbox. Draw shapes on a parametric EQ; the shapes are
*alive* — governed by a physical string simulation, able to move, travel,
and respond to audio.

The core inversion: **normally parameters push the curve. Here, the string
pushes the parameters.** A physics engine owns a node array (the "control
string"); the filter bank reads it every block. Drawing, wind forces,
traveling shapes, and audio analysis are all just writers to the same nodes.

Simple in front, deep underneath — every advanced parameter exists under the
hood from day one, exposed only in an advanced submenu.

## Status

Design + string-physics prototype stage. See [ORCHESTRATION.md](ORCHESTRATION.md)
for the full architecture, decisions, and milestone plan (M0–M7).

- `prototype/` — Python proof of the string physics (validated)
- `src/` — plugin source (C++, JUCE + CLAP, in progress)

## License

AGPLv3 — see [LICENSE](LICENSE). Open source; credits for borrowed
open-source components are tracked in ORCHESTRATION.md §9.1 and logged at
adoption time.
