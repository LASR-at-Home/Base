# Command generator data (lab)

Data directory for the [RoboCup@Home CommandGenerator](https://github.com/RoboCupAtHome/CommandGenerator),
in the same layout as the competition repos (e.g. [Incheon2026](https://github.com/RoboCupAtHome/Incheon2026)),
keeping only the four files the generator reads:

```
names/names.md           <- config/people.yaml
maps/room_names.md       <- config/locations.yaml (locations with a polygon)
maps/location_names.md   <- config/locations.yaml (other locations; (p) = placeable: true)
objects/objects.md       <- config/objects.yaml (grouped by category)
```

**Do not edit these files by hand.** Change `config/*.yaml` and regenerate:

```bash
python3 scripts/export_command_generator_data.py            # from tasks/GPSR
```

Locations with `exclude_from_commands: true` (e.g. the instruction point) are left out.

## Generating commands

```bash
uv pip install git+https://github.com/RoboCupAtHome/CommandGenerator
athome-generator -d command_generator_data          # interactive
athome-generator -d command_generator_data -g       # 5000 commands (sanity check)
athome-generator-gpsr-ui -d command_generator_data  # web UI
```
