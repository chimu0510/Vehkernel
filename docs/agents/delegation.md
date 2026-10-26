# Delegation: implementation through agy-bridge

An agent that has the [agy-bridge](https://github.com/sshahzaiib/agy-bridge) MCP server splits a ticket between two roles: the main agent decides and accepts, the Antigravity CLI (agy) implements. An agent without agy-bridge does both roles itself, by the same rules.

## Roles

| Role | Owns |
|---|---|
| Main agent | Decisions, specs and ADRs, publishing issues, claiming tickets, delegating, acceptance, commits and pushes, `Progress:` and closing comments, agent memory |
| agy | Implementing one ticket in the working tree, running the checks `CONTRIBUTING.md` requires, reporting the results |

agy never touches the issue tracker, the specs or the ADRs, and never commits or pushes.

## One ticket, step by step

1. The main agent picks the ticket from the frontier and claims it, as in `handoff.md`, **Starting a session**.
2. The main agent delegates with `delegate`, `cwd` set to the absolute path of the repository root. The prompt holds:
   - the ticket number and its acceptance criteria, copied from the issue, not paraphrased;
   - the files to read first: the spec, the ADRs the ticket touches, `GLOSSARY.md` and `CONTRIBUTING.md`;
   - the limits in **What agy must not do**, written out in full;
   - the report format in step 4.
3. agy implements and runs the checks in the container, never on the host (ADR-0016): `vk cmake --preset test`, `vk cmake --build --preset test`, `vk ctest --preset test`, a build of every firmware preset it touched, and the fuzz presets when it changed the Link's decoders ([Building](../guide/building.md#development-environment) defines `vk`).
4. agy reports: the files it changed, each check it ran with its result, and anything left undone or in doubt.
5. The main agent accepts or rejects:
   - reads the whole diff (`git diff`, `git status` for new files);
   - reruns the checks in the container itself; numbers from agy's report are never copied into a closing comment;
   - checks the change against the ticket's acceptance criteria and the spec;
   - checks the coding rules in `CONTRIBUTING.md`: the C++20 subset, modules talking only through the bus, no hand-edited generated files, the SPDX line, the ADR reference at the top of each file, the glossary's terms.
6. On a rejection the main agent sends the concrete findings with `follow_up` and the same `session_id`, without resending the context, and returns to step 5.
7. On acceptance the main agent splits the change into logical commits as `CONTRIBUTING.md` asks, pushes, and closes the ticket with its closing comment (`handoff.md`).

Keep each delegation to one ticket small enough to finish within the MCP client's timeout; split a larger ticket before delegating it.

## What agy must not do

The bridge runs agy with full shell access and its permission prompts skipped, so these limits hold only because the prompt states them:

- No `git commit`, `git push` or other history change, and no `gh` write.
- No edits under `docs/specs/` or `docs/adr/`, and no changes to issues.
- No `cmake` or `ctest` on the host; every build and test runs in the container.
- No flashing, SWD, serial or other access to hardware. A check that needs the bench goes back to the main agent, which hands it to a person (`ready-for-human`).
- No reading or quoting of anything under `vendor/`, and no credentials.
- When the spec looks wrong or the change would contradict an ADR, stop and report it instead of changing the design.

## Other agy-bridge tools

The main agent may use `adversarial_review` for a second opinion on a spec or ADR draft and on a change before it is pushed, and `web_lookup` and `deep_search` for external documentation and git history. Their answers are unverified: check every finding against the code before acting on it, and flag an ADR conflict as `domain.md` says.

## Handoff

A `session_id` is private to one machine and session; it never goes into an issue or into memory. When a session ends with a delegation unfinished, the `Progress:` comment records what is done and what is left, and the next session delegates again. Prompts to agy and its reports are written in English, with the glossary's terms.
