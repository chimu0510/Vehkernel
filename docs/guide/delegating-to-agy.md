# Delegating to agy

An agent working on Vehkernel can hand the implementation of a ticket to the Antigravity CLI (agy), as [`docs/agents/delegation.md`](../agents/delegation.md) describes. agy runs with its permission prompts skipped, so it must not be able to reach anything but the ticket's own worktree. This guide sets up the sandbox that enforces that on a development host, and shows how to run agy in it and how to check the sandbox.

With this setup, agy only writes code. It cannot build, test, commit or push: the agent that delegated runs the checks in the container and reviews the result.

## What the sandbox allows

agy runs inside [bubblewrap](https://github.com/containers/bubblewrap) (`bwrap`), which gives it its own view of the file system:

| agy can | agy cannot |
|---|---|
| Read and write its ticket's worktree | Write to disk anywhere else: the main checkout, other worktrees and the rest of the home directory are hidden or read-only |
| Read the system (`/usr`, `/etc`, ...), read-only | See the home directory: `~/.ssh`, `~/.config/gh`, `~/.claude.json`, the keyring, `~/.docker` are hidden |
| Read the repository's history (`git log`, `git diff`, `git status`) | Write to `.git`: no commit, branch, tag or push |
| Read and write its own state and login (`~/.gemini`, `~/.config/antigravity`) | Reach Docker, D-Bus or other services under `/run` |
| Use the network | |

Outside its worktree and its own state, everything agy writes lands in the sandbox's own memory (`/tmp`, and the empty home directory with the directories leading to the worktree) and is gone when agy exits.

The network stays open, because agy talks to Gemini over it. What agy can read, it could send: its worktree, which is public source, and its own login and state.

agy's own `--sandbox` option is not a substitute. On this host (agy 1.4.9, Linux) it made the workspace read-only and left the rest of the file system writable, and agy's file-editing tool ignored it altogether. The Antigravity plugin's README reports the same on macOS.

## Requirements

- A Linux host, linux/amd64, as for [building](building.md#development-environment). Tested on Ubuntu 24.04.
- The Claude desktop app, or another program allowed to create user namespaces (see [User namespaces on Ubuntu](#user-namespaces-on-ubuntu)).
- The development image `vehkernel-dev`, for the checks the delegating agent runs ([Building](building.md#development-environment)).

Tested with these versions:

| Tool | Version | Source |
|---|---|---|
| bubblewrap | 0.9.0 (Ubuntu package `bubblewrap`) | [containers/bubblewrap](https://github.com/containers/bubblewrap) |
| Antigravity CLI (`agy`) | 1.4.9 | [Antigravity CLI documentation](https://antigravity.google/docs/cli-using) |
| Antigravity plugin for Claude Code | 0.30.2 (commit `c5fda28`) | [yuting0624/antigravity-for-claude-code](https://github.com/yuting0624/antigravity-for-claude-code) |

## Installing

### bubblewrap

```bash
sudo apt install bubblewrap
bwrap --version
```

### The Antigravity CLI

Install agy as its [documentation](https://antigravity.google/docs/cli-using) describes. This guide expects the executable at `~/.local/bin/agy`; if yours is elsewhere, change `AGY_BIN` in the shell function below. Check it:

```bash
agy --version
```

Do not log in yet: the login is made inside the sandbox ([Logging in](#logging-in-inside-the-sandbox)).

### The Antigravity plugin for Claude Code

The plugin provides `agy-delegate`, the wrapper that runs agy headless, resumes a conversation and reports how a run ended. In Claude Code:

```text
/plugin marketplace add yuting0624/antigravity-for-claude-code
/plugin install antigravity@antigravity-for-claude-code
```

Claude Code installs it under `~/.claude/plugins/cache/antigravity-for-claude-code/antigravity/<version>/`, and `bin/agy-delegate` is the wrapper. The plugin's `/antigravity:setup` checks agy outside the sandbox and does not apply here.

The plugin also offers the agy-bridge MCP server's way of delegating. Do not use it for tickets: in the Claude desktop app an MCP tool call gives up after about a minute and returns no session id while agy keeps working.

## User namespaces on Ubuntu

bubblewrap needs an unprivileged user namespace. Ubuntu 24.04 restricts them through AppArmor:

```bash
sysctl kernel.apparmor_restrict_unprivileged_userns
```

When this prints `1`, `bwrap` started from an ordinary terminal fails with:

```text
bwrap: setting up uid map: Permission denied
```

A program with an AppArmor profile that allows user namespaces is not restricted, and the programs it starts inherit that. The Claude desktop app has one (`/etc/apparmor.d/claude-desktop`), so its agent shells and its Terminal panel can run `bwrap`. Run the sandbox from there. This guide does not change the system setting.

## The worktree directory

Each ticket's worktree, and the directory beside it with the ticket's prompts and agy's reports (`vk-<ticket>.agy/`), go in one directory of their own, outside the main checkout and away from other projects. Set it in the shell that creates worktrees and runs agy, for example in `~/.bashrc`:

```bash
export VK_WORKTREES="$HOME/WorkSpace/SubAgents"
mkdir -p "$VK_WORKTREES"
```

A ticket's worktree is then `$VK_WORKTREES/vk-<ticket>`, and its files `$VK_WORKTREES/vk-<ticket>.agy/`. The sandbox needs nothing else from this directory: it binds the worktree it is started in and the repository's `.git`, wherever they are.

## The sandbox

Define this shell function, in the shell that runs agy (for example in `~/.bashrc`). Run it from inside the ticket's worktree: it finds the worktree and the repository's `.git` from there.

```bash
AGY_BIN="$HOME/.local/bin/agy"
AGY_PLUGIN="$HOME/.claude/plugins/cache/antigravity-for-claude-code"

agy_sandboxed() {
    local wt common
    wt="$(git rev-parse --show-toplevel)" || return
    common="$(git rev-parse --path-format=absolute --git-common-dir)" || return
    mkdir -p "$HOME/.gemini" "$HOME/.config/antigravity"
    bwrap \
        --ro-bind / / --dev /dev --proc /proc \
        --tmpfs /tmp --tmpfs /run \
        --ro-bind /run/systemd/resolve /run/systemd/resolve \
        --tmpfs "$HOME" \
        --ro-bind "$AGY_BIN" "$AGY_BIN" \
        --ro-bind "$AGY_PLUGIN" "$AGY_PLUGIN" \
        --bind "$HOME/.gemini" "$HOME/.gemini" \
        --bind "$HOME/.config/antigravity" "$HOME/.config/antigravity" \
        --ro-bind "$common" "$common" \
        --bind "$wt" "$wt" \
        --die-with-parent --unshare-pid \
        -- "$@"
}
```

What each option does:

| Option | Why |
|---|---|
| `--ro-bind / /` | The whole system, read-only: agy and the tools it calls run as usual, but write nothing. |
| `--dev /dev --proc /proc` | A private `/dev` and `/proc`. |
| `--tmpfs /tmp` | An empty, private `/tmp`. |
| `--tmpfs /run` | Hides the Docker socket, the D-Bus session bus and the other local services under `/run`. Through D-Bus a process could start programs outside the sandbox (`systemd-run --user`). |
| `--ro-bind /run/systemd/resolve ...` | Puts the DNS configuration back: `/etc/resolv.conf` links into it. Without it, name lookups go to `[::1]:53` and fail. |
| `--tmpfs "$HOME"` | Hides the whole home directory, with its keys, tokens and other projects. |
| `--ro-bind "$AGY_BIN" ...`, `--ro-bind "$AGY_PLUGIN" ...` | Puts back agy and the plugin's `agy-delegate`, read-only. |
| `--bind "$HOME/.gemini" ...`, `--bind "$HOME/.config/antigravity" ...` | agy's state, conversations and login, writable. |
| `--ro-bind "$common" ...` | The repository's `.git`, read-only, so that `git log`, `git diff` and `git status` work in the worktree. |
| `--bind "$wt" ...` | The ticket's worktree, writable. It comes last, so it is writable even inside a hidden directory. |
| `--die-with-parent --unshare-pid` | agy and everything it starts stop with the shell that started them, and see no other process. |

Run it from the main checkout only to log in: the main checkout is then the writable directory.

## Logging in inside the sandbox

agy keeps its login in the system keyring, which it reaches over D-Bus. The sandbox hides D-Bus, so a login made outside it is not seen inside. When agy cannot reach the keyring, it falls back to a file, so log in once inside the sandbox:

1. In the Claude desktop app's Terminal panel, from any worktree or the main checkout, start agy in the sandbox:

   ```bash
   agy_sandboxed "$AGY_BIN"
   ```

2. Choose **Google OAuth**. agy cannot open a browser inside the sandbox; open the address it shows in your own browser and sign in.
3. When agy's prompt appears, leave it (`/exit` or Ctrl+D).

The login is now in `~/.gemini/antigravity-cli/antigravity-oauth-token`, which the sandbox can read. agy outside the sandbox keeps using the keyring. Check the login inside the sandbox:

```bash
agy_sandboxed "$AGY_BIN" models
```

## Running agy

From the ticket's worktree, with the prompt in `../vk-<ticket>.agy/prompt-<n>.md` (`delegation.md`, **Running agy**):

```bash
AGY_DELEGATE="$AGY_PLUGIN/antigravity/0.30.2/bin/agy-delegate"
agy_sandboxed "$AGY_DELEGATE" --yolo --timeout 60m -d "$PWD" - \
    < ../vk-<ticket>.agy/prompt-<n>.md \
    > ../vk-<ticket>.agy/report-<n>.md 2> ../vk-<ticket>.agy/agy-<n>.log
```

- The redirections are made by the shell outside the sandbox, so the prompt, report and log live in `../vk-<ticket>.agy/`, which agy cannot read or change.
- `--yolo` lets agy edit files and run commands without asking; the sandbox is what limits it.
- The `conversation_id` in the `AGY_USAGE` line of the log identifies the conversation. The next round on the same ticket adds `--conversation <id>`.
- The `duration_seconds` in the same line is the time since the conversation's first round started, idle time between rounds included, not the length of this round (seen with agy 1.4.9 on 2026-10-29: a 12-minute second round reported 9,081 s). A round's own length is the time from its log's creation to its report's last write, `stat -c '%W %Y' agy-<n>.log report-<n>.md`.
- The exit code says how it ended: 0 done, 2 agy failed, 10 quota, 11 not logged in, 12 timed out.
- An agent runs this as a background command, so that a long round is not cut off by a tool call time limit.

## Watching a running conversation

Do not open a conversation in another agy (`agy`, then `/resume`) while a delegation runs in it: agy allows a conversation to be open in one place at a time, and opening it a second time interrupts the delegation's round (seen on 2026-10-29: the round stopped producing and had to be stopped and resumed).

`tools/agy/agy_watch.py` shows what agy is doing instead, without touching the conversation. It runs outside the sandbox, from the main checkout:

```bash
python3 tools/agy/agy_watch.py <conversation-id> --last 10
python3 tools/agy/agy_watch.py <conversation-id> --follow
```

Each step is printed with its index, kind and status: `prompt` for a message sent to agy, `agy` for agy's reasoning and the tools it calls, the tool's name (`ReadFile`, `RunCommand`, `FileEdit`, ...) for its arguments and result, and `system` for agy's own notices. The status is `done`, `running` or `stopped`. `--follow` reads again every 5 s (`--interval`) and prints new steps, and a step again when its status changes; `--full` prints whole texts instead of one line per field.

What makes it safe:

- It never runs agy and never opens agy's files for writing. It copies the conversation's database and its write-ahead log into a private temporary directory, reads the copy and deletes it.
- It takes no lock and leaves agy's presence files (`presence/<id>.lock`) alone, which is how agy notices a second instance.
- It accepts only a conversation id, never a path.

The steps are stored as protobuf messages with no published schema; the tool reads the fields it knows from agy 1.4.9 and skips the rest, so after an agy update it may show less, and needs its field numbers updated.

## Checking the sandbox

Check the sandbox after setting it up, after changing the function, and after updating agy, the plugin or the system. Run a plain shell in it from a worktree, not agy: agy may refuse a prompt that probes its sandbox, and the sandbox does not depend on what runs inside it. Each line prints `ALLOWED` or `blocked` and changes nothing. The main checkout's own files must not be visible: only its `.git` is.

```bash
agy_sandboxed bash -c '
t() { if eval "$1" >/dev/null 2>&1; then echo "ALLOWED  $1"; else echo "blocked  $1"; fi; }
wt=$(git rev-parse --show-toplevel); common=$(git rev-parse --path-format=absolute --git-common-dir)
t "test -w $wt"
t "ls $(dirname "$common")/CMakeLists.txt"
t "ls $HOME/.ssh/."
t "ls $HOME/.config/gh/."
t "test -e $HOME/.claude.json"
t "ls $HOME/.local/share/keyrings/."
t "docker ps"
t "git -C $wt status --short"
t "test -w $common/objects"
t "test -w $common/refs/heads"
t "getent hosts generativelanguage.googleapis.com"'
```

Expected:

```text
ALLOWED  test -w <worktree>
blocked  ls <repository>/CMakeLists.txt
blocked  ls <home>/.ssh/.
blocked  ls <home>/.config/gh/.
blocked  test -e <home>/.claude.json
blocked  ls <home>/.local/share/keyrings/.
blocked  docker ps
ALLOWED  git -C <worktree> status --short
blocked  test -w <repository>/.git/objects
blocked  test -w <repository>/.git/refs/heads
ALLOWED  getent hosts generativelanguage.googleapis.com
```

Any other result means the sandbox does not hold; do not delegate until it does.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `bwrap: setting up uid map: Permission denied` | Ubuntu restricts user namespaces for this terminal | Run from the Claude desktop app ([User namespaces on Ubuntu](#user-namespaces-on-ubuntu)) |
| Login fails with `lookup oauth2.googleapis.com on [::1]:53 ... connection refused` | No DNS configuration in the sandbox | Keep `--ro-bind /run/systemd/resolve /run/systemd/resolve` |
| `authentication failed or timed out`, exit 12 at once | No login the sandbox can read; the log shows `Failed to load stored token from keyring, falling back to file` | [Log in inside the sandbox](#logging-in-inside-the-sandbox) |
| Exit 11 | The login expired | Log in inside the sandbox again |
| `fatal: not a git repository` inside the sandbox | The repository's `.git` is not bound | Run `agy_sandboxed` from inside the worktree, so that it finds `.git` |
| A `target/logs/` directory appears in the worktree | An MCP server configured in agy (for example a Discord one) writes its log in agy's working directory | `target/` is ignored through `.git/info/exclude`; or remove that MCP server from agy |
| An agent's call to `agy-delegate` is refused by Claude Code's auto mode | The permission check judges an agent that skips its permission prompts | The person decides: approve the call or add a permission rule; an agent does not change permission settings |
