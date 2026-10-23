# Specs

A spec describes one milestone: the problem, the solution, the user stories, the implementation and testing decisions, and what is out of scope. It lives twice: as a GitHub issue titled `Spec: ...` (published by `/to-spec`), and as `NNNN-slug.md` in this directory, numbered after the highest existing file. Its tickets are the issue's sub-issues (published by `/to-tickets`), with GitHub's native blocking edges; the last is usually the hardware check on the board, labelled `ready-for-human`. Issue handling is in `../agents/issue-tracker.md`.

## Status line

The line under the title gives the status, the issue and the tickets, and lives in the file only; the issue's open or closed state follows it:

```md
**Status**: in progress. Issue: [#61](https://github.com/chimu0510/Vehkernel/issues/61); tickets #62 to #67, with the hardware check in #67.
```

| Status | Meaning | What the status line adds |
|---|---|---|
| `in progress` | Tickets are being worked | The hardware check ticket |
| `deferred` | Paused by the maintainer with tickets still open | Who paused it and when, why, and what is left |
| `done` | Every ticket is closed and the spec issue with them | Where the hardware check confirmed it |

The tickets listed are the issue's sub-issues; where the two differ, the sub-issues win.

## Keeping a spec current

- **Until it is done**, the file and the issue body carry the same spec. A decision changed while implementing goes into both in the same edit, into every ADR it touches, and the commit message says why. The line under the status line reads: "Until this spec is done, this file and its issue carry the same spec; update both in the same edit."
- **Once it is done**, the spec is a record of what the milestone set out to build and is no longer edited, apart from its status line. Later decisions live in the ADRs and the code, which win where they differ. The line under the status line reads: "This spec is done and no longer edited; later decisions live in the ADRs and the code, which win where they differ."

## Tickets outside a spec

Small work that belongs to no milestone, such as a fix found on the bench, is a ticket with no parent issue, as #58 to #60 are. It follows the same closing rules as any ticket (`../agents/handoff.md`). If it changes what a spec in progress decides, that spec is updated as above.
