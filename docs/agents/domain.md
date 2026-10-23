# Domain docs

How to use the project's domain documentation when exploring or changing the code. Vehkernel is a single-context repository: one `GLOSSARY.md` at the root and one set of ADRs in `docs/adr/`.

## Before exploring, read these

- **`GLOSSARY.md`**: the project vocabulary.
- **`docs/adr/`**: the ADRs that touch the area you are about to work in. The index in `docs/adr/README.md` names each decision in one line.
- **`docs/specs/`**: the spec of the milestone the work belongs to, if any.

## Use the glossary's vocabulary

When your output names a domain concept (in an issue title, a refactor proposal, a hypothesis, a test name), use the term as `GLOSSARY.md` defines it. Do not drift to synonyms the glossary avoids.

If the concept you need is not in the glossary, either you are inventing language the project does not use (reconsider) or there is a real gap: note it, and add the term to the glossary once it is settled.

## Flag ADR conflicts

If your output contradicts an existing ADR, say so explicitly rather than silently overriding it:

> _Contradicts ADR-0008 (bus and work queues), but worth reopening because…_

## This repo's conventions

- All docs are in English (ADR-0021); use the glossary's English terms exactly as defined.
- ADRs are living records, edited in place; their format, editing rules and index are in `docs/adr/README.md`. Read it before writing or changing an ADR.
- Specs follow `docs/specs/README.md`.
