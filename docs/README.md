# Building the API documentation

The repository ships a static Doxygen site for the public `lems::data` API,
the `compat/vo` forwarding adapters, and the migration guides. The docs build
does not configure CMake, compile C++, find Eigen/OpenCV/yaml-cpp, or require
an installed `lems-data` library.

## Local build

From the repository root, run:

```sh
python3 tools/build_docs.py --output ../lems-data-docs-site
python3 tools/check_docs.py --site ../lems-data-docs-site/html
```

The default output is `../lems-data-docs-site` next to the checkout, which is
outside the source tree. Set `LEMS_DOCS_OUTPUT` or pass `--output` to choose a
different location. The command finds `doxygen` on `PATH`; set `DOXYGEN` or
pass `--doxygen /path/to/doxygen` when it is installed elsewhere. If Doxygen
is not supplied, the build stops with an actionable error; it does not install
packages or access the network.

The generated `html/` directory is a self-contained site. `xml/` is generated
as well for local tooling and link audits. Open `html/index.html` directly;
the native Doxygen index and browser search remain enabled. The local theme
files in `docs/site/assets/` are copied into the site, with no external CDN,
font, analytics, or telemetry dependency.

## API coverage and navigation contract

The Doxygen input is deliberately limited to the actual public headers under
`include/lems/data` and `compat/vo`, plus Markdown guides under `docs/` and
`docs/site/`. The landing page is `docs/site/index.md` and is used as the
Doxygen main page.

The stable documentation group IDs are:

- `datasets` — readers, configuration, and iteration
- `cameras` — camera, calibration, pose, and frame metadata
- `edges` — 2-D/3-D edges and matching records
- `utilities` — geometry and image utilities
- `pipeline` — data-only pipeline containers
- `compatibility` — Brown-LEMS compatibility adapters

`tools/check_docs.py` checks generated local links, the native Doxygen search
index, and member coverage for `Edge`, `Frame`, `CameraCalibration`,
`DatasetIterator`, and `Utility`. It intentionally validates generated HTML,
not a separate hand-written header parser.

## GitHub Pages deployment

`.github/workflows/docs.yml` builds and checks the site on pushes to `main` or
`codex/cohesive-edge-library`, and on manual `workflow_dispatch`. It installs
Doxygen on the hosted runner, writes generated files under the runner's
ephemeral temporary directory, validates the generated HTML, and uploads only
the generated `html/` tree through the official GitHub Pages artifact and
deployment actions. No generated files are committed to this repository.

Before the first deployment, an administrator must open **Settings → Pages**
and set **Source** to **GitHub Actions**. The workflow does not select or
change that repository setting; without it, a build or artifact upload does
not publish a Pages site. The repository's Actions policy must also allow the
official Pages actions named in the workflow. The deploy job uses the
`github-pages` environment and requires the standard `pages: write` and
`id-token: write` permissions.

This documentation site is intentionally public. A private repository alone
would not make a GitHub Pages site private, so do not publish sensitive
content on that assumption. Private Pages visibility requires an eligible
GitHub Enterprise Cloud organization with Pages access control enabled. If
that access control is unavailable, treat every published page as public.
