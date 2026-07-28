# Contributing to ax-ros2-ref

First off — thank you for taking the time to contribute! 🎉

> ⚠️ **Please note:** `ax-ros2-ref` is an **experimental reference implementation**, not
> an Axelera product. It is maintained on a best-effort basis with no guarantee of
> response times or long-term support. Contributions are very welcome, but please set
> expectations accordingly: reviews may take time, and not every change will be merged.

This document explains how to contribute and the rules that all contributions must follow.

## Contents

- [Developer Certificate of Origin (DCO)](#developer-certificate-of-origin-dco)
- [How to sign off your commits](#how-to-sign-off-your-commits)
- [Code of Conduct](#code-of-conduct)
- [Ways to contribute](#ways-to-contribute)
- [Reporting bugs](#reporting-bugs)
- [Suggesting enhancements](#suggesting-enhancements)
- [Development setup](#development-setup)
- [Pull request process](#pull-request-process)
- [Commit message guidelines](#commit-message-guidelines)
- [Coding style](#coding-style)
- [Testing](#testing)
- [Reporting security issues](#reporting-security-issues)
- [License](#license)

---

## Developer Certificate of Origin (DCO)

This project uses the [Developer Certificate of Origin (DCO)](DCO) to certify that
contributors have the right to submit the code they are contributing.

**By contributing to this project, you agree to and certify the terms of the DCO
(version 1.1), the full text of which is in the [`DCO`](DCO) file in this repository.**
In short, the DCO is a statement that you wrote the contribution yourself, or otherwise
have the right to submit it under the project's [open source license](LICENSE).

We **do not** use a separate Contributor License Agreement (CLA). Instead, we require
that **every commit is signed off** to indicate agreement with the DCO. Pull requests
that contain commits without a valid sign-off cannot be merged.

## How to sign off your commits

Signing off is done by adding a `Signed-off-by` line to your commit message that matches
the author of the commit. Git can add it for you automatically with the `-s` (or
`--signoff`) flag:

```bash
git commit -s -m "Fix duck detector HSV thresholds"
```

This appends a line like the following to the end of your commit message:

```
Signed-off-by: Jane Developer <jane.developer@example.com>
```

A few important points:

- The name and email in the sign-off **must match** your Git author identity. Set them
  once with:

  ```bash
  git config --global user.name "Jane Developer"
  git config --global user.email "jane.developer@example.com"
  ```

- Use your **real name** and a valid email address. Anonymous or pseudonymous
  contributions cannot satisfy the DCO.

### Fixing a missing sign-off

If you forgot to sign off, you can fix it before pushing:

```bash
# Amend the most recent commit
git commit --amend -s --no-edit

# Sign off every commit on your branch (rebasing off the default branch)
git rebase --signoff main
```

After amending or rebasing you will need to force-push your branch:

```bash
git push --force-with-lease
```

> 💡 **Tip:** Configure a Git alias or a `prepare-commit-msg` hook, or simply get into the
> habit of always using `git commit -s`, so you never forget the sign-off.

## Code of Conduct

Be respectful, inclusive, and constructive. Harassment, discrimination, and other
unwelcome behavior will not be tolerated. Assume good intent, keep discussions technical,
and help make this a welcoming space for contributors of all backgrounds and experience
levels. Maintainers may remove, edit, or reject contributions and comments that do not
align with these principles.

## Ways to contribute

There are many ways to help, and not all of them involve writing code:

- Reporting bugs and reproducible issues
- Improving documentation (typos, clarifications, new guides, examples)
- Suggesting or prototyping enhancements
- Testing on different hardware (Metis M.2/PCIe, host platforms, robot arms)
- Reviewing open pull requests
- Answering questions from other users

## Reporting bugs

Bugs, questions, and usage help are handled on the **[Axelera developer
community](https://community.axelera.ai/)**, not through GitHub issues. See
[SUPPORT.md](SUPPORT.md) for details.

Before posting:

1. **Search the [community](https://community.axelera.ai/)** to avoid duplicates and see
   if your question has already been answered.
2. Make sure you are on a supported configuration (see
   [`docs/quickstart.md`](docs/quickstart.md) — Ubuntu 24.04 + ROS2 Jazzy).

A good bug report includes:

- A clear, descriptive title.
- The exact steps to reproduce, ideally as a minimal example.
- What you **expected** to happen versus what **actually** happened.
- Relevant logs, error output, and stack traces (as text, not screenshots where possible).
- Your environment: OS, ROS2 distribution, Voyager SDK version, Metis hardware
  (M.2/PCIe), and any robot hardware in use.

If you already have a fix, you are welcome to open a pull request directly (see
[Pull request process](#pull-request-process)).

## Suggesting enhancements

For non-trivial changes, **please raise the idea on the [Axelera developer
community](https://community.axelera.ai/) before you start coding.** This avoids wasted
effort on changes that may not fit the direction of this reference implementation.
Describe the use case and motivation, not just the proposed solution — it helps others
understand the problem you are solving.

## Development setup

See [`docs/quickstart.md`](docs/quickstart.md) for the full toolchain and workspace setup
(Voyager SDK, ROS2 Jazzy, and building the `ros2_ws` workspace with `colcon`).

Typical contributor workflow:

1. **Fork** the repository and clone your fork.
2. Create a topic branch off `main`:

   ```bash
   git checkout -b my-feature main
   ```

3. Make your changes in small, focused commits.
4. Build and test locally (see [Testing](#testing)).
5. Push to your fork and open a pull request.

Keep your branch up to date with `main` by rebasing rather than merging where practical,
so history stays linear and easy to review.

## Pull request process

1. Ensure your branch **builds cleanly** (`colcon build`) and existing tests pass.
2. Ensure **every commit is signed off** (see [above](#how-to-sign-off-your-commits)).
3. Keep pull requests **focused and reasonably small** — one logical change per PR is far
   easier to review than a large, mixed changeset.
4. Update **documentation** (README, `docs/`) and any relevant launch files or parameters
   when behavior changes.
5. Fill in the PR description: explain **what** changed and **why**, and link any related
   [community](https://community.axelera.ai/) discussion.
6. Be responsive to review feedback. Maintainers may request changes; push follow-up
   commits (or amend and force-push) as needed.
7. By opening a PR you confirm your contribution complies with the
   [DCO](#developer-certificate-of-origin-dco).

## Commit message guidelines

Well-formed commit messages make history easier to read and `git blame` more useful:

- Use a short (≤ ~72 character) **imperative** summary line:
  *"Add pose smoothing filter"*, not *"Added"* or *"Adds"*.
- Leave a blank line, then a body explaining **what** and **why** (not just how) when the
  change is non-trivial.
- Reference any related [community](https://community.axelera.ai/) discussion in the body
  or footer.
- End every commit with your `Signed-off-by` line (added automatically by `git commit -s`).

Example:

```
Add HSV threshold tuning for red-toy detector

The default thresholds missed darker red objects under warm lighting.
Widen the hue band and expose the range as a launch parameter so it can
be tuned per environment without recompiling.

Signed-off-by: Jane Developer <jane.developer@example.com>
```

## Coding style

- **C++**: follow the [ROS2 / ament C++ style](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).
  Run `ament_uncrustify` / `ament_cpplint` where available and keep nodes composable.
- **Python**: follow [PEP 8](https://peps.python.org/pep-0008/) and the ROS2 Python
  conventions; `ament_flake8` and `ament_pep257` should pass.
- Match the style of the surrounding code — naming, formatting, and comment density.
- Keep changes minimal and avoid unrelated reformatting in the same PR.

## Testing

- Build the workspace and confirm it compiles without new warnings:

  ```bash
  cd ros2_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

- Run available tests:

  ```bash
  colcon test && colcon test-result --verbose
  ```

- For behavioral changes, describe in the PR how you verified them (hardware used, launch
  command, observed result). Include new or updated tests where practical.

## Reporting security issues

**Please do not report security vulnerabilities publicly** — not on the
[developer community](https://community.axelera.ai/) and not through GitHub. Instead,
report them privately via the [Axelera support channels](https://support.axelera.ai) or
by contacting the maintainers directly, and allow reasonable time for a response before
public disclosure. Note that, as an experimental reference implementation, this project
carries no security-support guarantees.

## License

By contributing, you agree that your contributions will be licensed under the license
covering the code you are changing, and you certify the
[Developer Certificate of Origin](DCO) as described above.

This repository is not licensed uniformly. The root [LICENSE](LICENSE) is MIT, but each
ROS2 package declares its own license in its `package.xml` (mostly Apache-2.0), and
`ros2_ws/src/ned-ros2-driver/` is a bundled BSD-3-Clause copy of a third-party project.
Check the license applying to the files you touch — see the licensing table in the
[README](README.md#licensing) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

Changes to `ros2_ws/src/ned-ros2-driver/` are best sent
[upstream](https://github.com/NiryoRobotics/ned-ros2-driver) where they are not specific
to this reference implementation.
