# Third-Party Notices

The simulator imports the repositories below at the pinned revisions in `simulator/dependencies.lock.yaml`.
They remain separate source checkouts and are not committed to this repository.

## EUFS repositories

`eufs_sim2`, `state_lib`, `map_lib`, `eufs_msgs`, `eufs_gmock_matchers`, and `eufs_logger` are licensed under MIT.
Their pinned `LICENSE` files name Edinburgh University Formula Student as the copyright owner.

- Evidence: the corresponding `license_evidence` URL in `simulator/dependencies.lock.yaml`.

`vehicle_models` declares MIT in its pinned `package.xml`.
That metadata identifies Cameron Matthew as maintainer but does not state a copyright owner.

- Evidence: https://gitlab.com/eufs/public/vehicle_models/-/blob/3508bec2c3d77e0ff16f08794675d4f7b52479b7/package.xml#L5-L8

## Open Car Dynamics

`open_car_dynamics` is licensed under Apache-2.0 at revision `94f8fb187fb0ed22bba1d809bd74f66d1ff75af4`.
Its pinned `LICENSE` contains the Apache License 2.0 text but leaves the example copyright notice unfilled.
The pinned `CITATION.cff` names the project authors, including Simon Sagmeister, Simon Hoffmann, Georg Jank, and Panagiotis Kounatidis, but does not assert a copyright owner.

- License evidence: https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/LICENSE
- Author evidence: https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/CITATION.cff
