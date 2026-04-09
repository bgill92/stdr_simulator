# Agent Delegation

**Blocking requirement:** Always delegate code writing and modification to the agent pipeline. Never write, edit, or refactor code directly in the top-level conversation. This is not optional.

## Required Pipeline

Any task that involves writing or modifying code must go through the full pipeline in order:

1. **`coder`** — writes/implements the code changes
2. **`pre-commit`** — run `pixi run pre-commit run --files <changed-files>` on all files the coder touched, then `git add` any auto-fixed formatting. This prevents formatting-only noise from leaking into the final diff.
3. **`code-reviewer`** — reviews the changes for correctness and standards compliance
4. **`test-runner`** — builds the package and runs tests to verify nothing is broken

All four steps are mandatory. Do not skip pre-commit, the reviewer, or test-runner, even for "simple" changes. Do not present coder output to the user without review and build verification.

If the user explicitly asks for only specific agent(s) (e.g., "Run the test-runner"), delegate only to those requested.

## What Does Not Require Agents

Only these tasks may be done directly without agents: answering questions, reading files, git operations, running skills/slash-commands, and other tasks that produce zero code changes.
