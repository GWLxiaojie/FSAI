# Third-Party Notices

The simulator imports the repositories below at the pinned revisions in `simulator/dependencies.lock.yaml`.
They remain separate source checkouts and are not committed to this repository.

## IMechE FS-AI official visual and track assets

The optional ten-lap scenario fetches data directly from `FS-AI/FS-AI_IMechE_ADS-DV_HiL`
at revision `1fe77bed411ba621a3187bc2eb9ca13c7d371832`.
The pinned paths and Git blob hashes are in `simulator/official_assets.lock.json`.
Original GLB/OBJ/Road/TestRun files and all derived geometry remain under the ignored
`.dependencies` directory; they are not redistributed with this source repository.
The official HiL repository does not declare an open-source license for these assets.
The separate official CAD repository states that its CAD remains Copyright IMechE
and must not be re-uploaded to other public locations.
The importer uses the HiL's native visual meshes, not a copy of the full CAD or the
commercial CarMaker dynamics engine.

- HiL source: https://github.com/FS-AI/FS-AI_IMechE_ADS-DV_HiL/tree/1fe77bed411ba621a3187bc2eb9ca13c7d371832
- CAD source/use notice: https://github.com/FS-AI/FS-AI_ADS-DV_CAD/blob/958fdbf336fcc6cbd60535c2e7812d51a977d267/README.md

## EUFS repositories

### eufs_sim2

Revision: `9f5df79a03725ea7d10542fc2ce8224d90836560`.
License: MIT.
Copyright owner: Edinburgh University Formula Student.
Evidence: https://gitlab.com/eufs/public/eufs_sim2/-/blob/9f5df79a03725ea7d10542fc2ce8224d90836560/LICENSE

### vehicle_models

Revision: `3508bec2c3d77e0ff16f08794675d4f7b52479b7`.
License: MIT.
Copyright owner: not stated in pinned package metadata.
Evidence: https://gitlab.com/eufs/public/vehicle_models/-/blob/3508bec2c3d77e0ff16f08794675d4f7b52479b7/package.xml#L5-L8

### state_lib

Revision: `ec83a141f188e8a4c39a381f4666485d8cc83e20`.
License: MIT.
Copyright owner: Edinburgh University Formula Student.
Evidence: https://gitlab.com/eufs/public/state_lib/-/blob/ec83a141f188e8a4c39a381f4666485d8cc83e20/LICENSE

### map_lib

Revision: `1919b36062850c9ba4553d1833a9b517c61c2e86`.
License: MIT.
Copyright owner: Edinburgh University Formula Student.
Evidence: https://gitlab.com/eufs/public/map_lib/-/blob/1919b36062850c9ba4553d1833a9b517c61c2e86/LICENSE

### eufs_msgs

Revision: `9e918686c9e9292c613f321e6fd85e3a5d87cd87`.
License: MIT.
Copyright owner: Edinburgh University Formula Student.
Evidence: https://gitlab.com/eufs/public/eufs_msgs/-/blob/9e918686c9e9292c613f321e6fd85e3a5d87cd87/LICENSE

### eufs_gmock_matchers

Revision: `7ef83d030746c6a31bcf4f888d4121fcf4b7e8a9`.
License: MIT.
Copyright owner: Edinburgh University Formula Student.
Evidence: https://gitlab.com/eufs/public/eufs-gmock-matchers/-/blob/7ef83d030746c6a31bcf4f888d4121fcf4b7e8a9/LICENSE

### eufs_logger

Revision: `375ea1d8f8885af66809129e444624ba13353fa7`.
License: MIT.
Copyright owner: Edinburgh University Formula Student.
Evidence: https://gitlab.com/eufs/public/eufs-logger/-/blob/375ea1d8f8885af66809129e444624ba13353fa7/LICENSE

## pybind11_conversions

Revision: `6c97133c717133b9c8f2ab42c602677b36a9a658` (upstream master, not its stale default branch).
License: not declared at this pinned revision; package metadata contains a TODO placeholder.
Copyright owner: not stated in pinned package metadata.
Required by the pinned map_lib Python bindings.
Source remains a separate checkout and is not redistributed in this repository.
Evidence: https://gitlab.com/eufs/public/pybind11_conversions/-/blob/6c97133c717133b9c8f2ab42c602677b36a9a658/package.xml

## Open Car Dynamics

`open_car_dynamics` is licensed under Apache-2.0 at revision `94f8fb187fb0ed22bba1d809bd74f66d1ff75af4`.
Its pinned `LICENSE` contains the Apache License 2.0 text but leaves the example copyright notice unfilled.
The pinned `CITATION.cff` names the project authors, including Simon Sagmeister, Simon Hoffmann, Georg Jank, and Panagiotis Kounatidis, but does not assert a copyright owner.

- License evidence: https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/LICENSE
- Author evidence: https://github.com/TUMFTM/Open-Car-Dynamics/blob/94f8fb187fb0ed22bba1d809bd74f66d1ff75af4/CITATION.cff
