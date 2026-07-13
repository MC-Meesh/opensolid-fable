# Merge Pipeline

1. A polecat works its assigned bead on an isolated git worktree branch.
2. Running `gt done` pushes that branch and submits a merge-request (MR) wisp.
3. The MR wisp enters the Refinery merge queue.
4. Refinery runs the verification gates (build, tests, clippy) on the branch.
5. On green gates, Refinery merges the branch to `main` on origin — no direct pushes to main.
