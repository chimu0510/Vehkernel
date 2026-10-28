# Delegation: implementation through agy

An agent that has the [Antigravity CLI](https://antigravity.google/docs/cli-using) (agy) and the `agy-delegate` wrapper of the Antigravity plugin for Claude Code splits a ticket between two roles: the main agent decides and accepts, agy implements. An agent without them does both roles itself, by the same rules.

## Running agy

agy runs as a shell command in the background, not through the agy-bridge MCP server: an MCP tool call gives up after about a minute in the desktop app, before any ticket is done, and returns no session id while agy keeps working (found on 2026-10-29). A shell command has no such limit, and each one is a separate process, so several tickets can run at once.

Each ticket keeps its files in a directory beside its worktree, `../vk-<ticket>.agy/`, outside the repository: the prompt and report of each round `n`, and agy's log. From the worktree:

```bash
agy-delegate --yolo --timeout 60m -d "$PWD" - < ../vk-<ticket>.agy/prompt-<n>.md \
  > ../vk-<ticket>.agy/report-<n>.md 2> ../vk-<ticket>.agy/agy-<n>.log
```

- `--yolo` lets agy edit files and run commands without asking, as the bridge did; the limits in **What agy must not do** hold only because the prompt states them.
- The first round starts a new agy conversation. Its id is the `conversation_id` in the `AGY_USAGE` line of `agy-1.log`.
- Every later round on the ticket adds `--conversation <id>`, so agy continues with what it already knows about the ticket.
- The exit code says how it ended: 0 done, 2 agy failed, 10 quota, 11 not logged in, 12 timed out (the report then holds agy's partial answer). On 12, the next round resumes the same conversation.
- The default model tier (`flash`) suits most tickets; `--tier pro` is for a ticket that is mostly algorithm, such as a filter's mathematics.

## Roles

| Role | Owns |
|---|---|
| Main agent | Decisions, specs and ADRs, publishing issues, claiming tickets, worktrees, delegating, final acceptance, merges, commits and pushes, `Progress:` and closing comments, agent memory |
| Ticket subagent (optional) | Driving one ticket's agy conversation in the background for the main agent: delegating, rerunning the checks, a first review against the acceptance criteria, and a short report |
| agy | Implementing one ticket in its worktree, running the checks `CONTRIBUTING.md` requires, reporting the results |

agy never touches the issue tracker, the specs or the ADRs, and never commits or pushes. A ticket subagent never commits, pushes, merges or writes to the issue tracker either: it reports to the main agent, which owns acceptance.

## What to delegate

Delegate a ticket when its implementation is larger than the cost of writing the prompt and reviewing the result: a new module, driver or tool with its tests, or a change across many files. The main agent does the rest itself:

- small changes, such as a fix of a few lines, a parameter or a renamed term;
- changes that are mostly judgement, such as an ADR conflict, a design choice left open by the spec, or tuning against measurements;
- hardware checks, which go to a person (`ready-for-human`).

## One issue, one agy conversation

- Each ticket starts its own agy conversation. Every later round on that ticket resumes it with `--conversation <id>`, so fixes continue from what agy already knows about the ticket.
- A conversation is never resumed for another ticket, and never given two tickets at once. Each delegation then starts from the repository and its prompt alone, with nothing carried over from other work.
- When the ticket is closed, its conversation is finished and not resumed again.
- A conversation id belongs to agy on one machine. If it is lost, or the conversation cannot be resumed, start a new one on the same ticket from its `Progress:` comment and the worktree's diff.

## One ticket, step by step

1. The main agent picks the ticket from the frontier and claims it, as in `handoff.md`, **Starting a session**.
2. The main agent creates the ticket's worktree on a new branch from `main`: `git worktree add ../vk-<ticket> -b ticket/<ticket> main`, and runs `git submodule update --init` in it. The worktree is the `cwd` of every agy call on the ticket; its build trees stay inside it (`build/<preset>`).
3. The main agent writes the prompt to `../vk-<ticket>.agy/prompt-1.md` and runs agy from the worktree, as **Running agy** says. The prompt holds:
   - the ticket number and its acceptance criteria, copied from the issue, not paraphrased;
   - the files to read first: the spec, the ADRs the ticket touches, `GLOSSARY.md` and `CONTRIBUTING.md`;
   - the limits in **What agy must not do**, written out in full;
   - the report format in step 5.
4. agy implements and runs the checks in the container, never on the host (ADR-0016): `vk cmake --preset test`, `vk cmake --build --preset test`, `vk ctest --preset test`, a build of every firmware preset it touched, and the fuzz presets when it changed the Link's decoders ([Building](../guide/building.md#development-environment) defines `vk`; run it from the worktree).
5. agy reports: the files it changed, each check it ran with its result, and anything left undone or in doubt.
6. The main agent, or the ticket's subagent, reviews:
   - reads the whole diff against `main` (`git diff main`, `git status` for new files);
   - checks that agy changed nothing outside the ticket's purpose: not `.devcontainer/`, `.github/`, `uv.lock`, `pyproject.toml` or the submodules, unless the ticket asks for it, and nothing outside the worktree;
   - checks that no existing test was removed, skipped or loosened, and that no tolerance, threshold or number in the acceptance criteria was changed;
   - reruns the checks itself in a fresh container (`vk` starts one per command), from a clean build tree (`rm -rf build/<preset>` first); numbers from agy's report are never copied into a closing comment;
   - checks the change against the ticket's acceptance criteria and the spec;
   - checks the coding rules in `CONTRIBUTING.md`: the C++20 subset, modules talking only through the bus, no hand-edited generated files, the SPDX line, the ADR reference at the top of each file, the glossary's terms.
7. On a rejection the reviewer writes the concrete findings as the next round's prompt, without resending the context, resumes the ticket's conversation with `--conversation <id>`, and returns to step 6.
8. On acceptance the main agent, never a subagent, reads the diff itself, splits the change into logical commits as `CONTRIBUTING.md` asks, brings the branch onto the current `main` (rebase, then the checks again if `main` moved), fast-forwards `main`, pushes, closes the ticket with its closing comment (`handoff.md`), and removes the worktree, its branch and its `../vk-<ticket>.agy/` directory (`git worktree remove`, `git branch -d`).

Keep each round small enough to finish within `--timeout`: as a rule, one module or tool and its tests. Split a larger ticket before delegating it.

## Several tickets at once

The main agent may run up to three tickets at the same time when all of these hold:

- each is on the frontier, and none blocks another;
- the files each is expected to change do not overlap, counting the shared and generated inputs: `schema/`, `Kconfig`, the `params.toml` files, the `CMakeLists.txt` files, `CMakePresets.json` and `REUSE.toml`;
- each has its own worktree, branch and agy conversation.

The main agent hands each ticket to a background ticket subagent, which runs steps 3 to 7 and returns only a short report: the files changed, the checks it reran and their results, each acceptance criterion with its state, and its doubts. agy's output and the review rounds stay in the subagent, which keeps the main agent's context small while the tickets run.

The main agent then accepts the tickets one at a time, in frontier order, as in step 8. Only the implementation runs in parallel: the first accepted ticket lands on `main`, and the next is rebased onto it and checked again before it lands. If two tickets turn out to touch the same file, the later one waits for the earlier to land.

A firmware image built in a worktree carries no git hash: the worktree's `.git` points outside the directory `vk` mounts, so `finalize_image.py` writes 0. Images for flashing are built on `main`.

## What agy must not do

`--yolo` runs agy with full shell access and its permission prompts skipped, so these limits hold only because the prompt states them:

- No `git commit`, `git push` or other history change, no branch or worktree changes, and no `gh` write.
- No edits under `docs/specs/` or `docs/adr/`, and no changes to issues.
- No edits outside the worktree given as `cwd`.
- No changes to the development environment to make a check pass: not `.devcontainer/`, `.github/`, `uv.lock`, `pyproject.toml`, the submodules or installed packages, unless the ticket asks for it.
- No removing, skipping or loosening an existing test, and no changing a tolerance, threshold or number the acceptance criteria give. If one seems wrong, stop and report it.
- No `cmake` or `ctest` on the host; every build and test runs in the container.
- No flashing, SWD, serial or other access to hardware. A check that needs the bench goes back to the main agent, which hands it to a person (`ready-for-human`).
- No reading or quoting of anything under `vendor/`, and no credentials.
- When the spec looks wrong or the change would contradict an ADR, stop and report it instead of changing the design.

## Other agy-bridge tools

The agy-bridge MCP server's short tools fit within the MCP call limit. The main agent may use `adversarial_review` for a second opinion on a spec or ADR draft and on a change before it is pushed, and `web_lookup` and `deep_search` for external documentation and git history. Their answers are unverified: check every finding against the code before acting on it, and flag an ADR conflict as `domain.md` says.

## Handoff

An agy conversation id never goes into an issue or into memory. When a session of the main agent ends with a delegation unfinished, the `Progress:` comment records what is done and what is left, and the name of the ticket's branch; the worktree's changes are committed to that branch first, as a work-in-progress commit that is never pushed to `main`. The next session continues in that worktree with a new agy conversation, as **One issue, one agy conversation** says. Prompts to agy, the ticket subagents' reports and agy's reports are written in English, with the glossary's terms.
