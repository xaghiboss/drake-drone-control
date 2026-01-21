# Quadcopter Simulation (simulation/quadcopter_sim)

Short description
- Small Drake + MeshCat based quadcopter simulation (C++ / Bazel).
- This directory contains the simulation source and Bazel build files to build and run the demo.

Why two README files and versioning?
- Keep one canonical `README.md` with usage, build, and developer notes.
- Keep a separate `CHANGELOG.md` (versioned notes) so release history is clear and concise.
- This makes it easy for users to get started from `README.md` while letting maintainers record version changes in `CHANGELOG.md`.

Repository layout (this folder)
- BUILD.bazel — Bazel target for `quadcopter`.
- MODULE.bazel, MODULE.bazel.lock — Bzlmod module configuration and lock.
- main.cc, quadcopter_*.cc/.h, keyboard_input.* — source files.
- build.sh, run.sh — helper scripts that make builds reproducible locally (use a writable Bazel output root).
- .gitignore — ignores build outputs and local caches.
- CHANGELOG.md — versioned changelog / release notes.

Prerequisites
- Bazel (tested with Bazel 9.0.0); newer Bazel should work but may need minor adjustments.
- Drake available at the path specified in `MODULE.bazel` (this repository uses a local path override):
  - In this repo we point Drake to `~/drake` via `local_path_override` in `MODULE.bazel`.
  - Ensure Drake is downloaded and available there, or update `MODULE.bazel` accordingly.
- A modern C++ toolchain (g++/clang with C++20).
- A web browser for MeshCat visualization (MeshCat opens on localhost).

Quick start (recommended, copy/paste)
1. From project root (this directory):
   ```
   cd ~/drake_projects/drake-drone-control/simulation/quadcopter_sim
   chmod +x build.sh run.sh
   ./build.sh
   ./run.sh
   ```
   - `./build.sh` uses a Bazel output root outside the repo (by default `$HOME/.bazel_output_quadcopter`) so the build does not require changing system cache mounts.
   - `./run.sh` starts the simulation and prints the MeshCat URL (e.g. `http://localhost:7000`). Open that in your browser.

Notes about Bazel output root and caches (important)
- On some systems `~/.cache/bazel` is a symlink to a shared mount that can be read-only (for example `/mnt/drake/bazel-cache`). If Bazel cannot write there, builds fail.
- The included `build.sh` forces Bazel to use an output root outside the workspace and outside such mounts:
  - OUT_ROOT="$HOME/.bazel_output_quadcopter"
- This folder is not tracked (included in `.gitignore`). You can change location by editing `build.sh`/`run.sh` if desired.

If you prefer to run Bazel manually:
- One-off build using local output root:
  ```
  bazel --output_user_root="$HOME/.bazel_output_quadcopter" build //:quadcopter --cxxopt='-std=c++20'
  ./bazel-bin/quadcopter
  ```

What I changed recently (so you understand the repo)
- Added `build.sh` and `run.sh` that use `$HOME/.bazel_output_quadcopter`.
- Added `.gitignore` to avoid committing bazel outputs and editor files.
- Added `bazel_dep(name = "rules_cc", version = "0.2.14")` to `MODULE.bazel` so Bazel knows where to find C++ rules under Bzlmod.
- Updated `BUILD.bazel` to `load("@rules_cc//cc:defs.bzl", "cc_binary")` before declaring the `cc_binary` target.

Common commands (git, build, debugging)
- Stage & commit:
  ```
  git add .
  git commit -m "Your message"
  git push origin main
  ```
- Create archive or share:
  ```
  git archive -o quadcopter_sim.zip HEAD
  ```
- If Bazel reports "repo contents cache inside the main repo":
  - Make sure output_user_root is outside the workspace (the provided `build.sh` does this).
- If Bazel complains about missing `cc_binary` rule:
  - Ensure `MODULE.bazel` contains `bazel_dep(name = "rules_cc", version = "0.2.14")` (or another compatible version).
  - Ensure `BUILD.bazel` contains the `load("@rules_cc//cc:defs.bzl", "cc_binary")` line.

Building on another machine / CI
- CI should:
  - Check out the repo.
  - Ensure Drake is available (either from local path or modify MODULE.bazel to use a public drake module).
  - Use `bazel --output_user_root="$(pwd)/.bazel_output" build //:quadcopter` or call `./build.sh` after setting `$HOME` appropriately.
- Commit the `MODULE.bazel.lock` file so module versions are reproducible for others.

Troubleshooting (short)
- Build fails with permission errors writing to `/mnt/drake`:
  - Use the supplied `build.sh` (it avoids /mnt/drake).
  - Or fix the mount (requires admin); not recommended for casual use.
- MeshCat not visible:
  - Confirm the printed URL and port (default 7000). If firewall blocks, open the port or use the browser on the same host.
- Git push fails:
  - Run `git pull --rebase origin main` then `git push`.
  - If you must overwrite remote, use `git push --force-with-lease origin main` (dangerous if you collaborate).

How to tag a release (versioning)
- After committing changes for a release:
  ```
  git tag -a v1.1 -m "Release v1.1: add build/run scripts, rules_cc support"
  git push origin v1.1
  ```
- Update `CHANGELOG.md` with the release notes (see `CHANGELOG.md`).

Contributing
- Please open PRs for changes. Keep the `MODULE.bazel.lock` updated when module deps change.
- Prefer a small, focused commit per logical change.

License
- Add your preferred license file (LICENSE). If none present, add one before public distribution.

Contact / maintainer
- Repository owner: xaghiboss (GitHub).
- For build-specific issues, paste the last 50 lines of the failing Bazel output when asking for help.

Enjoy! If you want, I can add a short CI workflow that runs `./build.sh` on push (GitHub Actions).
