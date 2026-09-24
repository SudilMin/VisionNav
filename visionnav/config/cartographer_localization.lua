-- Localization in a saved map (laptop_brain.launch.py loads <map>.pbstream with this file).
-- Same sensor settings as mapping; the saved map stays frozen and only the last few submaps of
-- the current walk are kept, so the map does not grow or drift while the wearer is at home.
include "cartographer_config.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
-- Find the wearer in the saved map quickly (defaults: every 90 nodes, 0.003 of them, after 10 s).
-- There is no initial pose, so the first fix comes from this global search; a house-sized map keeps
-- it cheap. On a standing rig a node is added only every 0.5 s, so it must search often.
POSE_GRAPH.optimize_every_n_nodes = 10
POSE_GRAPH.global_sampling_ratio = 0.3
POSE_GRAPH.global_constraint_search_after_n_seconds = 2.

return options
