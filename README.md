# wheeltec_ros2

This repository manages the `/mnt/wheeltec_ros2` ROS2 workspace with Git.

## Scope

The initial repository scope is intentionally conservative:

- Track source code under `src/`
- Track workspace-level docs and conventions
- Ignore build products, logs, virtual environments, recordings, and local auxiliary assets

## Workflow

- Keep changes small and topic-focused
- Prefer commit messages in `type(scope): summary` format
- Avoid editing generated content under `build/`, `install/`, and `log/`
- Use static checks when build/test commands are restricted

## Recommended Commit Types

- `feat`: new feature
- `fix`: bug fix
- `refactor`: code restructuring without behavior change
- `docs`: documentation update
- `config`: parameter or configuration change
- `chore`: repository maintenance

## Notes

This workspace has local constraints documented in `AGENTS.md` and `.ai_agent_constraints.md`.
