## What and why

<!-- What this changes and why. Link the issue: Closes #NN -->

## How it was checked

<!-- Tests added or run (ctest, SITL scenarios), presets built, and hardware checks with their results. -->

## Checklist

- [ ] `ctest --preset test` passes and every touched firmware preset builds
- [ ] Docs, ADRs and the spec affected by this change are updated in this pull request
- [ ] New topics are defined in `schema/msg/`; no generated file is edited by hand
- [ ] Changes that move actuators were checked with the wheels off the ground and output power limited, or say why not
- [ ] No vendor datasheets, schematics, firmware or flash dumps are added
