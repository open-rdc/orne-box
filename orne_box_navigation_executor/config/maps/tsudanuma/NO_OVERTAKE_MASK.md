# No-overtake mask

`cit_3f_map_no_overtake.pgm` uses the same 4000 x 4000 geometry, 0.05 m
resolution, and origin as `cit_3f_map.yaml`.

- White (`0` occupancy): normal overtaking behavior.
- Black (`100` occupancy): no-overtake behavior.

Keep the image dimensions and YAML metadata unchanged when editing the mask.
Add approximately 0.3-0.5 m of padding at zone entrances and exits to avoid
state flicker caused by localization uncertainty.
