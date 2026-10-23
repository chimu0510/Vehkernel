# Handoff: progress between sessions

Progress lives where every session, agent and person can read it: the GitHub issues and the spec files (`issue-tracker.md`). A session leaves nothing that only it knows.

## Where each thing is recorded

| What | Where |
|---|---|
| A finished ticket | Its closing comment: `Done in <commit>.`, what was checked and the results (hardware readings, the tests that cover it), and any part moved to another ticket, named by number |
| A ticket in progress | A comment headed `Progress:` on it: the commits so far, what is left, the gotchas found, and how to get back to the same state |
| A decision taken while implementing that changes a spec | The spec file and its issue body, in the same edit; the commit message says why |
| A decision that is hard to reverse, surprising and a real trade-off | An ADR (`domain.md`) |
| Commits, push state, test counts | Git and CI: look them up, never copy them |
| Agent memory (outside the repo, private to one agent and machine) | Only what the repo and the tracker cannot hold: the maintainer's preferences and corrections, gotchas of the host and the bench, external sources, and one pointer to the active spec issue |

## Ending a session

1. Close every finished ticket with its closing comment. Done when no finished ticket is open and every closed one carries its results.
2. Leave a `Progress:` comment on the ticket still in progress. Done when a session that reads only that comment and the commits can carry on.
3. Bring each touched spec file and its issue body in step with the tickets: the status line, and the decisions changed while implementing. Done when the spec, its issue and the code describe the same behaviour.
4. Push, or say in the `Progress:` comment which commits are local and why.
5. Prune agent memory: add only facts the tracker and the repo cannot hold, delete what they now answer or what went stale, and point to the active spec issue.

## Starting a session

1. From the memory pointer, open the active spec issue and its sub-issues; the next ticket is the first open one with no open blocker (`issue-tracker.md`, frontier query).
2. Read the latest `Progress:` comment on it, `git log --oneline -10` and `git status`.
3. Check the host and bench gotchas in memory before building or flashing.
