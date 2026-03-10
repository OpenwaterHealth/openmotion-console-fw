# SBOM

This folder contains a lightweight SPDX tag-value Software Bill of Materials (SBOM) for the repository.

- `spdx-2.2.spdx`: high-level SPDX tag-value SBOM created by inspection.

Notes and next steps:

- To generate a more comprehensive, machine-verified SBOM (including licenses, checksums, transitive dependencies), run an automated scanner such as `syft` (Anchore) or `cyclonedx-bom` and add the output here.

Example command (recommended locally):

```bash
# generate CycloneDX JSON with syft
syft . -o cyclonedx-json > SBOM/cyclonedx.json

# or generate SPDX with syft
syft . -o spdx-json > SBOM/spdx-full.json
```

If you want, I can run an automated SBOM generation step (CycloneDX or SPDX) and add the results here — tell me which format and whether you want me to attempt to run `syft` in this environment.
