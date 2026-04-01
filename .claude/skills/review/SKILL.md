---
name: review
description: "Review recent code changes against coding standards"
context: fork
---

# Review

## Context

- Current branch: !`git branch --show-current`
- Diff to review: !`git diff HEAD`
- Staged changes: !`git diff --cached`

## Task

Review the code changes shown above. Check against:

1. **CLAUDE.md** — project-wide rules
2. **`.claude/rules/cpp-style.md`** — C++ style, GCC 11 gotchas, error handling
3. **`.claude/rules/testing.md`** — test patterns, benchmarks

## Output Format

**SUMMARY**: 1-2 sentence overview

**REQUIRED CHANGES**: Blocking issues with priority (Critical/High/Medium)

**SUGGESTED IMPROVEMENTS**: Non-blocking enhancements

Focus on things that would cause bugs, CI failures, or violate project conventions. Skip obvious style comments that `clang-format` handles.
