# chassis_robot

Source-only snapshot of the Wheeltec ROS 2 workspace used by the chassis robot.

The live workspace source tree is stored under `src/`. Runtime maps, ROS bags,
recordings, generated previews, caches, local backup files, ML/speech models,
meshes, media, and precompiled libraries are intentionally excluded. Those
artifacts must be provisioned separately when required by a package.

Credential literals found in the live tree are not copied. The affected source
uses `WHEELTEC_SSH_PASSWORD` and `QWEN_API_KEY` environment variables instead.
