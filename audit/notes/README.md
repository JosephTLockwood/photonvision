# audit/notes — per-check working notes

One file per check: `<stage>-<short-topic>.md`. Each note should record:

- The check ID from `PROGRESS.md` (e.g. `1.1`)
- The iteration timestamp it was performed at
- The git commit(s) of any file(s) read
- The derivation / trace / verification
- Outcome: **finding** (with severity, linked to FINDINGS.md entry) or **clean** (with a
  one-paragraph rationale).

Notes for already-PR'd issues use the prefix `prefound-` and just cite the PR number.
