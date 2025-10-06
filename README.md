# OSRM-output

Small utility that uses OSRM to compute pairwise travel times and distances between a list of coordinates.

This repository contains a small C++ program (CMake-based) that:

- Loads a list of coordinates (or samples them inside a small Belgium area if none provided).
- Boots an OSRM engine (using a prebuilt `.osrm` dataset) and queries pairwise routes.
- Produces two CSV outputs: travel_distances and travel_times, plus (optionally) a coordinates file.
- Contains a `DockerImage/Dockerfile` that builds an image with the runtime and creates an `/app/results` folder inside the container.

## Quick overview of the code

- `include/OSRMParameters.h` — struct `osrm_params` containing configuration, coordinates storage, and helpers for loading/sampling/saving coordinates.
- `src/OSRM_Engine.cpp` — the main OSRM logic: sampling (if used), starting the OSRM engine, calculating pairwise matrices (distance/time), writing CSVs.
- `src/main.cpp` — CLI entrypoint: parses arguments, loads or samples coordinates, starts the engine and runs calculations.
- `DockerImage/Dockerfile` — Dockerfile to build a container with system dependencies and compile the app.

## Coordinate format

The program expects a simple whitespace-separated text file where each line contains one coordinate pair. Use the format:

```
<longitude> <latitude>
```

Example:
```
4.3517 50.8466
3.7038 51.0370
```

Note: internally coordinates are used as `(longitude, latitude)` to match how the OSRM types are constructed in the code.

## Output files

By default outputs are written to the `results/` directory (created automatically by the repository and Dockerfile):

- `results/travel_distances.csv` — CSV matrix of distances (meters). Row = from-index, column = to-index.
- `results/travel_times.csv` — CSV matrix of travel times (seconds). Same indexing as distances.
- `results/coordinates.txt` — when sampling is used, the sampled coordinates written as `longitude latitude` per line.

## Build & run (native)

Prerequisites
- CMake
- A C++17-capable compiler (g++/clang)
- An OSRM dataset (.osrm files) for the region you want to query (e.g. Belgium)

Build

```sh
mkdir -p build
cd build
cmake ..
cmake --build . --parallel 8
```

Run

```sh
# Provide an OSRM dataset and optional coordinates file
./build/osrm --osrm-path /full/path/to/belgium-latest.osrm --coordinates-path /full/path/to/coords.txt

# If you omit --coordinates-path the program samples 100 points inside a small central-Belgium bounding area
./build/osrm --osrm-path /full/path/to/belgium-latest.osrm
```

Notes:
- If `--coordinates-path` is provided, the program uses the file you pass. If omitted, it randomly samples locations inside Belgium (small central polygon) and writes the sampled coordinates to `results/coordinates.txt`.
- After the run you should find the CSV matrices in `results/`.

## Docker image

The `DockerImage/Dockerfile` bundles the dependencies and builds both OSRM and this application in the image.

Build the image (from repository root):

```sh
docker build -f DockerImage/Dockerfile -t osrm-output:latest .
```

Run an instance and mount host folders (recommended)

The container expects the OSRM dataset under `/app/OSM` (you can mount your host dataset there). It writes outputs to `/app/results`, which you should mount to persist results on the host.

Example (mount host `OSM/` and `results/`):

```sh
docker run --rm \
  -v /host/path/to/OSM:/app/OSM \
  -v $(pwd)/results:/app/results \
  osrm-output:latest \
  --osrm-path /app/OSM/belgium-latest.osrm \
  --coordinates-path /app/results/coordinates.txt
```

Notes on entrypoint:
- The Dockerfile defines the binary as the image `ENTRYPOINT`, but you can (and in the example do) provide additional arguments to the container command line. They will be appended to the entrypoint command.
- If you prefer to pass a different executable or fully override entrypoint, use `--entrypoint` in `docker run`.

## Configuring where outputs go

- Host: the program writes to `results/` in the repository root when running locally.
- Container: the Dockerfile creates `/app/results`. Mount a host dir into `/app/results` when running the container to persist outputs.

## TBB / destructor note (macOS)

You may have noticed a crash during program exit referencing `libtbbmalloc` or `libtbb` on some macOS setups. This is a destructor-order issue that occurs in certain environments when TBB static destructors run during process teardown.

Workarounds the code includes:

1. The program performs an explicit cleanup: it resets the OSRM engine pointer and frees allocated matrices before exit. This helps reduce the chance of TBB-related crashes.
2. If you still see a crash on teardown, the code uses `std::_Exit(0)` after cleanup to avoid running the remaining C++ static destructors. This is a pragmatic workaround (it skips any further C++ destructors) — acceptable here because the program explicitly releases its OSRM/TBB resources first.

If you prefer to debug the destructor-order issue instead of using `std::_Exit`, try running under a debugger or removing the early exit and instrumenting static destructors to discover which object triggers TBB access during teardown.

## Troubleshooting

- "coordinates-path" not being read: make sure you pass the option with leading dashes: `--coordinates-path /full/path/coords.txt` (launch configs or scripts that omit `--` will not register it as an option).
- Wrong coordinate order: the file must be `longitude latitude` per line. If your data is (lat lon), swap columns.
- OSRM dataset not found: ensure the `.osrm` files and related files are in the path you pass to `--osrm-path`.

## Development notes

- The code uses simple in-memory `int**` matrices for distances and times. For very large location sets this will consume O(n^2) memory. Consider streaming, chunking, or using a disk-backed matrix for larger scale.
- The sampling routine currently uses a small, manually-specified polygon for central Belgium. Replace with a GeoJSON polygon if you need precise national boundaries.

## License

See `LICENSE` in this repository.

---

If you want, I can:
- Add a short example `coords.txt` in `results/` for testing.
- Add CI task snippets to build the Docker image and run a quick smoke test that verifies CSV files are created.
- Improve the coordinates loader to accept both `lat lon` and `lon lat` if you want auto-detection.
