# Preserved local RViz configuration edits

The two imported LBR checkouts had different local `moveit.rviz` edits. Each is
saved separately, with base revision and modified-file hashes in manifest.json.
Temporary-index reconstruction reproduced every modified file byte for byte
before the checkout was restored to its recorded version. No robot command or
hardware connection was made. These are configuration snapshots, not reviewed
changes for upstream.

To restore an experiment, confirm its checkout/base revision from the manifest,
then use `git -C <checkout> apply <absolute-path-to-patch>`. Do not apply both
variants to the same checkout.
