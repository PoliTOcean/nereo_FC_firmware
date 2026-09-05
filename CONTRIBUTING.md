# Contributing to Nereo FC Firmware

This document states the working agreements this repository is held to for
the current hardening milestone. It cites the team's style and git guides as
authority rather than restating them — read the guides for the full rules,
use this file as the review-time checklist.

## Code Style

Naming, formatting, and file-organisation rules are maintained in the team's
`R&D` planning workspace, alongside this repository, at
`Rules latex/c_guidelines.tex` (C) and `Rules latex/cpp_guidelines.tex`
(C++, applies to `freertos.cpp` and other `.cpp` sources). Those documents
are authoritative; this is a quick-reference checklist for PR review:

- [ ] snake_case for variables and functions
- [ ] UPPER_CASE for constants and macros
- [ ] TitleCase for structs and enums
- [ ] 80-column lines
- [ ] K&R brace style (opening brace on the same line)
- [ ] Include guards on every header (`UPPER_SNAKE_CASE` matching the file
      path, e.g. `CORE_INC_NAVIGATION_H_`)

## Git Workflow

Commit and branching rules are maintained at
`Rules latex/r&d_software_github_rules.ltx`. In short:

- Commits are atomic and single-purpose — one logical change per commit.
- Work happens on a descriptive branch per feature or fix, never directly
  on `main`.
- A PR is opened only after the change has been tested. For anything
  touching the thruster/command path, that means working through the
  bench-then-wet verification ladder before requesting review, not after.

## This Milestone's Standing Rule

No PR touches more than one audited `stabilize_mode.c` defect (STD-03).
`stabilize_mode.c`'s defects are deliberately split across separate PRs so
that a regression discovered during a wet test stays bisectable to a single
change. This is the rule most likely to be violated under deadline pressure
and the most expensive one to get wrong, which is why the PR template makes
it an explicit checkbox rather than an unwritten norm.

## Documentation

New or changed public interfaces (function declarations in headers) document
their parameters and return values with a Doxygen `@param`/`@return` block,
matching the existing style in `Core/Inc/interpolations.h`:

```c
/**
 * @brief One-line summary of what the function does.
 *
 * @param foo What foo is.
 * @return What is returned.
 */
```

## What This Milestone Deliberately Does Not Do Yet

- **No CI style gate.** There is no `clang-format` check or lint step
  enforcing any of the above mechanically. Enforcement is a human review
  gate at PR time for now; mechanising it is deferred, tracked work.
- **No rollout to the other two repos yet.** This checklist applies to
  `nereo_FC_firmware` only. It will be rolled out to `nereo_ros2_code` and
  `ros2_controller_tuning_aid` when those repos are first touched in a
  later phase of this milestone, not pre-emptively here.
