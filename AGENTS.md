# Vehkernel

## Language

Write everything in English: docs, ADRs, glossary, code, comments, diagnostics, commit messages, issues and pull requests, in the style of ArduPilot and PX4 documentation. Use the English terms defined in `GLOSSARY.md`. See `docs/adr/0021-english-project-language.md`.

## Contributing

The rules for every contributor, human or agent, are in `CONTRIBUTING.md`: the build and checks to run, the coding rules, and how decisions and milestones are recorded. This file adds what agents need on top.

## Agent skills

### Issue tracker

Issues live in GitHub Issues for `chimu0510/Vehkernel`, managed with the `gh` CLI. See `docs/agents/issue-tracker.md`.

### Triage labels

Default five-role vocabulary: `needs-triage`, `needs-info`, `ready-for-agent`, `ready-for-human`, `wontfix`. See `docs/agents/triage-labels.md`.

### Domain docs

Single-context: root `GLOSSARY.md` plus `docs/adr/`. See `docs/agents/domain.md`.

### Handoff

Progress lives in the issues and specs, not in agent memory. When starting a session, finishing a ticket or ending a session, follow `docs/agents/handoff.md`.
