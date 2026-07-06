#!/usr/bin/env bash
#
# run.sh — automate the OSRM-output Docker workflow.
#
# Usage:
#   ./run.sh -r belgium -c ./coordinates.txt
#   ./run.sh -u https://download.geofabrik.de/europe/belgium-latest.osm.pbf -c ./coordinates.txt
#   ./run.sh -r belgium            # no coords -> sampling mode
#
# What it does:
#   1. Resolves an .osm.pbf (downloads from Geofabrik if not already cached in ./OSM)
#   2. Copies your coordinates file into DockerImage/results/coordinates.txt (if given)
#   3. Builds the docker image only if it doesn't already exist (or --force-build)
#   4. Runs the container, mounting:
#        - ./OSM       -> raw + processed OSRM data (cached across runs)
#        - ./results   -> CSV outputs
#
set -euo pipefail

IMAGE_NAME="app/osrm"
DOCKER_DIR="DockerImage"
OSM_DIR="$(pwd)/OSM"
RESULTS_DIR="$(pwd)/results"

REGION=""
PBF_URL=""
COORDS=""
FORCE_BUILD=false

usage() {
  echo "Usage: $0 [-r region] [-u pbf_url] [-c coordinates.txt] [--force-build]"
  echo
  echo "  -r  Geofabrik region name, e.g. 'belgium', 'france', 'germany'"
  echo "      (resolves to https://download.geofabrik.de/europe/<region>-latest.osm.pbf)"
  echo "  -u  Direct URL to an .osm.pbf file (overrides -r)"
  echo "  -c  Path to a coordinates.txt file (longitude latitude per line)"
  echo "      If omitted, the app samples points inside the region."
  echo "  --force-build   Rebuild the docker image even if it already exists"
  exit 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -r) REGION="$2"; shift 2 ;;
    -u) PBF_URL="$2"; shift 2 ;;
    -c) COORDS="$2"; shift 2 ;;
    --force-build) FORCE_BUILD=true; shift ;;
    -h|--help) usage ;;
    *) echo "Unknown argument: $1"; usage ;;
  esac
done

if [[ -z "$REGION" && -z "$PBF_URL" ]]; then
  echo "Error: you must provide either -r <region> or -u <pbf_url>"
  usage
fi

mkdir -p "$OSM_DIR" "$RESULTS_DIR"

# --- 1. Resolve the .osm.pbf -------------------------------------------------
if [[ -n "$PBF_URL" ]]; then
  PBF_FILE="$OSM_DIR/region.osm.pbf"
  REGION_LABEL="$(basename "$PBF_URL")"
else
  PBF_FILE="$OSM_DIR/region.osm.pbf"
  REGION_LABEL="$REGION"
  PBF_URL="https://download.geofabrik.de/europe/${REGION}-latest.osm.pbf"
fi

# Cache: only download if not already present, or if a different region was requested
CACHE_MARKER="$OSM_DIR/.last_region"
NEED_DOWNLOAD=true
if [[ -f "$PBF_FILE" && -f "$CACHE_MARKER" ]]; then
  if [[ "$(cat "$CACHE_MARKER")" == "$REGION_LABEL" ]]; then
    NEED_DOWNLOAD=false
  fi
fi

if $NEED_DOWNLOAD; then
  echo ">> Downloading $REGION_LABEL from $PBF_URL"
  curl -L --fail -o "$PBF_FILE" "$PBF_URL"
  echo "$REGION_LABEL" > "$CACHE_MARKER"
else
  echo ">> Using cached $PBF_FILE for region '$REGION_LABEL'"
fi

# --- 2. Coordinates -----------------------------------------------------------
if [[ -n "$COORDS" ]]; then
  if [[ ! -f "$COORDS" ]]; then
    echo "Error: coordinates file '$COORDS' not found"
    exit 1
  fi
  DEST_COORDS="$RESULTS_DIR/coordinates.txt"
  COORDS_ABS="$(cd "$(dirname "$COORDS")" && pwd)/$(basename "$COORDS")"
  if [[ "$COORDS_ABS" == "$DEST_COORDS" ]]; then
    echo ">> Coordinates already at $DEST_COORDS"
  else
    cp "$COORDS" "$DEST_COORDS"
    echo ">> Copied $COORDS -> $DEST_COORDS"
  fi
else
  echo ">> No coordinates file given — app will sample points."
  rm -f "$RESULTS_DIR/coordinates.txt"
fi

# --- 3. Build the image (skip if it already exists, unless --force-build) ----
IMAGE_EXISTS=$(docker images -q "$IMAGE_NAME" 2>/dev/null || true)
if [[ -z "$IMAGE_EXISTS" || "$FORCE_BUILD" == true ]]; then
  echo ">> Building docker image ($IMAGE_NAME)"
  cp "$PBF_FILE" "$DOCKER_DIR/OSM/region.osm.pbf" 2>/dev/null || {
    mkdir -p "$DOCKER_DIR/OSM"
    cp "$PBF_FILE" "$DOCKER_DIR/OSM/region.osm.pbf"
  }
  docker build "$DOCKER_DIR" -t "$IMAGE_NAME"
else
  echo ">> Image $IMAGE_NAME already exists, skipping build (use --force-build to override)"
fi

# --- 4. Run the container ------------------------------------------------------
echo ">> Running container"
docker run --rm \
  -v "$RESULTS_DIR:/app/results" \
  "$IMAGE_NAME"

echo ">> Done. Results in $RESULTS_DIR"
