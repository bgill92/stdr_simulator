# Git Workflow

## Commit Messages

- Follow [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) format: `type(scope): description`
- Allowed types: `build`, `bump`, `chore`, `ci`, `docs`, `feat`, `fix`, `perf`, `refactor`, `revert`, `style`, `test`
- Scope is optional but encouraged (e.g., `feat(behavior): add new gripper plugin`)
- Use present tense ("Add feature" not "Added feature").
- Be concise but descriptive — one line summarizing the change.
- Enforced by commitizen via a `commit-msg` pre-commit hook.

## Destructive Commands

Before running any rollback, reset, or overwrite command (`git reset`, `git checkout -- <file>`, `git show ... > file`, `git clean`), explicitly confirm with the user the exact files and revisions to be changed. If there is any ambiguity, stop and ask — never assume.

## Before Committing

1. `pre-commit run -a` — must pass
2. Squashing to a single commit is recommended. A series of 2–3 well-organized commits is also fine if the user prefers. CI enforces a max of 3 commits per PR.
3. Force push: always `--force-with-lease` with explicit branch name

## PR Titles

- Always capitalize the first word.
- Bug fixes: start with "Fix: " (e.g., "Fix: Crash when loading empty config").
- Documentation changes: start with "Docs: " (e.g., "Docs: Add setup guide for new users").
- New features: start with "Add " (e.g., "Add support for custom gripper profiles").
- Claude workflow changes: start with "Claude: " (e.g., "Claude: Updated CLAUDE.md with better git process").

## Pull Requests

- Always create PRs as **draft** (`gh pr create --draft`). A human must transition it to "Ready for review."
- Commit messages must contain meaningful content.
- CMake changes for non-ROS code must not include Ament macros.
- Never create custom milestones, labels, types, or other metadata fields on PRs or issues. Only select from existing ones. Creating new metadata is reserved for users.
- When filing bug reports, set the issue **Type** to "Bug" (GitHub's native issue type field). Do not use `label: Bug`.
