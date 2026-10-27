#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE_NAME="${IMAGE_NAME:-carla-0.9.15:dtt-map-import}"
CARLA_BRANCH="${CARLA_BRANCH:-0.9.15}"
UNREAL_BRANCH="${UNREAL_BRANCH:-carla}"
BUILD_DIR="${BUILD_DIR:-$SCRIPT_DIR/carla-build}"
OUTPUT_DIR="${OUTPUT_DIR:-$SCRIPT_DIR/carla-output}"
RUNTIME_BASE="${RUNTIME_BASE:-nvidia/vulkan:1.1.121-cuda-10.1--ubuntu18.04}"
PACKAGE_CONFIG="${PACKAGE_CONFIG:-Shipping}"
BASE_IMAGE="${IMAGE_NAME}-base"
BUILD_CONTAINER="carla-package-build-$$"
NO_CACHE=0
FROM_TARBALL=0
LOG_FILE=""

usage() {
  cat <<'USAGE'
Build a runtime-only CARLA 0.9.15 Docker image with the fixed Unreal Engine.

The image contains a packaged CARLA (server binary, engine, content, Python
API, Import/) built from the CARLA-patched Unreal Engine fork
(CarlaUnreal/UnrealEngine, branch 'carla') with the Digital Twin Tool C++
patch applied. The fork contains the distance-field fix (commit a288ae07)
that is missing from the prebuilt carlasim/carla:0.9.15 image and that
makes Digital Twin Tool maps crash on load (see
carla-simulator/carla#8696). The DTT C++ patch
(docker/patches/carla-0.9.15-digital-twin.patch, runtime C++ subset) makes
single-level DTT maps simulatable: without it the server segfaults on the
first tick (ALargeMapManager puts actors to sleep on tile-less maps and
ASensor::EndPlay dereferences the never-started sensor's stream token).
The final image contains only the packaged CARLA: no Unreal Engine source,
editor, or build toolchain.

Required environment variables (only when the base image must be built):
  EPIC_USER   GitHub username for CarlaUnreal/UnrealEngine (your account
              must be a member of the Epic Games GitHub organization)
  EPIC_PASS   GitHub token/password with access to the repository

Optional environment variables:
  IMAGE_NAME       Tag of the final runtime image
                    default: carla-0.9.15:dtt-map-import
  CARLA_BRANCH     CARLA git branch/tag            default: 0.9.15
  UNREAL_BRANCH    CARLA Unreal Engine branch      default: carla
  BUILD_DIR        Host folder used as CARLA's Build/ directory (third-party
                    dependencies, kept on the host and reused on retries)
                    default: <script folder>/carla-build
  OUTPUT_DIR       Host folder for the release tarball, the log and the
                    staging area (default: <script folder>/carla-output)
  RUNTIME_BASE     Base image of the runtime container.
                    default: nvidia/vulkan:1.1.121-cuda-10.1--ubuntu18.04
                    (same base as the official carlasim/carla image; it provides
                    a Vulkan loader/ICD stack compatible with the host NVIDIA
                    driver - a plain ubuntu:18.04 base makes the server exit
                    at startup with no Vulkan device)
  PACKAGE_CONFIG   Package configuration: Shipping, Development or Debug
                    default: Shipping

Options:
  --no-cache       Disable the Docker layer cache for the base image. Required
                   to pick up new commits of the Unreal Engine fork branch
                   (a cached shallow clone does not pull new commits).
  --from-tarball   Skip phases 1-2 and rebuild the runtime image from the
                   newest CARLA_*.tar.gz already in OUTPUT_DIR (e.g. to tweak
                   the runtime image or switch RUNTIME_BASE without
                   re-packaging).
  --help           Show this help

How it works:
  Phase 1 (docker build): Unreal Engine, CARLA source, DTT C++ patch
           (runtime subset of docker/patches/carla-0.9.15-digital-twin.patch),
           Content download and URL patches -> <IMAGE_NAME>-base. Fully
           layer-cached. SKIPPED when the base image already exists (also,
           then no EPIC credentials are needed).
  Phase 2 (docker run):   `make setup` and `make package` run in a container
           with BUILD_DIR mounted as CARLA's Build/ folder. This cooks all
           maps with the fixed engine, builds the CARLA server/client
           binaries and the Python API, and produces the release tarball.
           Expect one to three hours.
  Phase 3 (docker build): the tarball is extracted and copied into a small
           runtime image (<RUNTIME_BASE> + SDL2/Vulkan/OpenMP) that contains
           only the packaged CARLA under /home/carla.

Build/ (third-party dependencies) and the release tarball stay on the host
in BUILD_DIR and OUTPUT_DIR; neither is part of the final image.

Run the result with:
  bash docker/start-carla-docker.sh [path/to/cooked_map.tar.gz]
USAGE
}

die() { echo "ERROR: $*" >&2; exit 1; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-cache)
      NO_CACHE=1
      shift
      ;;
    --from-tarball)
      FROM_TARBALL=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ "$PACKAGE_CONFIG" == "Shipping" || "$PACKAGE_CONFIG" == "Development" || "$PACKAGE_CONFIG" == "Debug" ]] \
  || die "PACKAGE_CONFIG must be Shipping, Development or Debug"

command -v docker >/dev/null 2>&1 || die "docker is not installed or not in PATH."
docker info >/dev/null 2>&1 || die "Docker daemon is not reachable."
docker buildx version >/dev/null 2>&1 || die "Docker Buildx support is required."

# Free space in KiB of the filesystem holding PATH.
free_kib() { df -Pk "$1" | awk 'NR==2 {print $4}'; }
have_base_image() { docker image inspect "$BASE_IMAGE" >/dev/null 2>&1; }

TMP_DIR="$(mktemp -d -t carla-dtt-build.XXXXXXXX)"
NETRC_FILE="$TMP_DIR/netrc"
cleanup() {
  rm -rf "$TMP_DIR"
  docker rm -f "$BUILD_CONTAINER" >/dev/null 2>&1 || true
}
trap cleanup EXIT HUP INT TERM

# Phase 1 build inputs (generated up front, because the Dockerfile hash is
# needed below to tell whether an existing base image is stale).
DTT_PATCH="$SCRIPT_DIR/patches/carla-0.9.15-digital-twin.patch"
[[ -f "$DTT_PATCH" ]] || die "DTT C++ patch not found: $DTT_PATCH"
cp -f "$DTT_PATCH" "$TMP_DIR/carla-dtt.patch"

# This helper patches only Update.sh's download function.
cat > "$TMP_DIR/patch_update.py" <<'PY_UPDATE'
from pathlib import Path

p = Path("Update.sh")
s = p.read_text()
old = """  if hash aria2c 2>/dev/null; then
    echo -e \"${CONTENT_LINK}\\n\\tout=Content.tar.gz\" > .aria2c.input
    aria2c -j16 -x16 --input-file=.aria2c.input
    rm -f .aria2c.input
  else
    wget -c ${CONTENT_LINK} -O Content.tar.gz
  fi
"""
new = """  CACHE_DIR=\"${CARLA_CONTENT_CACHE:-${HOME}/.cache/carla-content}\"
  CACHE_ARCHIVE=\"${CACHE_DIR}/${CONTENT_ID}.tar.gz\"
  mkdir -p \"${CACHE_DIR}\"
  if [[ -f \"${CACHE_ARCHIVE}\" ]] && tar -tzf \"${CACHE_ARCHIVE}\" >/dev/null 2>&1; then
    echo \"Using cached CARLA content archive: ${CACHE_ARCHIVE}\"
    cp -f \"${CACHE_ARCHIVE}\" Content.tar.gz
  elif hash aria2c 2>/dev/null; then
    echo -e \"${CONTENT_LINK}\\n\\tout=Content.tar.gz\" > .aria2c.input
    aria2c -j16 -x16 --input-file=.aria2c.input
    rm -f .aria2c.input
    tar -tzf Content.tar.gz >/dev/null
    cp -f Content.tar.gz \"${CACHE_ARCHIVE}\"
  else
    wget -c ${CONTENT_LINK} -O Content.tar.gz
    tar -tzf Content.tar.gz >/dev/null
    cp -f Content.tar.gz \"${CACHE_ARCHIVE}\"
  fi
"""
if old not in s:
    raise SystemExit("Update.sh download block not found")
p.write_text(s.replace(old, new, 1))
PY_UPDATE

# NOTE: this heredoc is quoted, so its content is literal. Inside the sed
# commands below, a single backslash (\.) is what sed needs. Do not double it.
cat > "$TMP_DIR/Dockerfile" <<'EOF_DOCKERFILE'
# syntax=docker/dockerfile:1.7

FROM ubuntu:20.04

ARG CARLA_BRANCH=0.9.15
ARG UNREAL_BRANCH=carla

USER root
ENV DEBIAN_FRONTEND=noninteractive

# CARLA 0.9.15 Ubuntu 20.04 toolchain. Kept identical to the previous build
# script so Docker can reuse its cached layer.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ca-certificates \
      wget \
      software-properties-common \
      gnupg \
      dirmngr \
      apt-transport-https && \
    add-apt-repository ppa:ubuntu-toolchain-r/test && \
    wget -qO /usr/share/keyrings/apt-llvm.asc https://apt.llvm.org/llvm-snapshot.gpg.key && \
    echo "deb [signed-by=/usr/share/keyrings/apt-llvm.asc] http://apt.llvm.org/focal/ llvm-toolchain-focal-10 main" \
      > /etc/apt/sources.list.d/llvm-10.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      clang-10 \
      lld-10 \
      g++-7 \
      cmake \
      ninja-build \
      libvulkan1 \
      python3 \
      python3-dev \
      python3-pip \
      python-is-python3 \
      libpng-dev \
      libtiff5-dev \
      libjpeg-dev \
      tzdata \
      sed \
      curl \
      unzip \
      autoconf \
      libtool \
      rsync \
      libxml2-dev \
      git \
      aria2 && \
    python3 -m pip install --no-cache-dir -Iv setuptools==47.3.1 && \
    python3 -m pip install --no-cache-dir distro wheel auditwheel && \
    update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-10/bin/clang++ 180 && \
    update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-10/bin/clang 180 && \
    rm -rf /var/lib/apt/lists/*

RUN useradd -m -u 1000 -s /bin/bash carla
USER carla
WORKDIR /home/carla
ENV HOME=/home/carla
ENV UE4_ROOT=/home/carla/UE4.26

# ------------------------------------------------------------------------------
# Stage 1: CARLA-patched Unreal Engine 4.26 (expensive, cached)
# ------------------------------------------------------------------------------
RUN --mount=type=secret,id=github_netrc,target=/home/carla/.netrc,uid=1000,gid=1000,mode=0400 \
    git clone --depth 1 --branch "${UNREAL_BRANCH}" \
      https://github.com/CarlaUnreal/UnrealEngine.git "${UE4_ROOT}" && \
    cd "${UE4_ROOT}" && \
    ./Setup.sh && \
    ./GenerateProjectFiles.sh && \
    make

# ------------------------------------------------------------------------------
# Stage 2: CARLA source
# ------------------------------------------------------------------------------
RUN git clone --depth 1 --branch "${CARLA_BRANCH}" \
      https://github.com/carla-simulator/carla.git /home/carla/carla

WORKDIR /home/carla/carla

# ------------------------------------------------------------------------------
# Stage 2b: Digital Twin Tool C++ patch (single-level / pre-cooked maps)
# ------------------------------------------------------------------------------
# Applies only the runtime C++ changes from the DTT workflow (Carla plugin +
# LibCarla). Map-creation tooling (Streetmap, CarlaTools, HoudiniEngine,
# build scripts, default-map configs) is intentionally NOT applied. The key
# fix: ALargeMapManager no longer puts actors to sleep on maps without tiles;
# stock 0.9.15 crashes on the first tick of a single-level DTT map
# (ConvertActiveToDormantActors -> PutActorToSleep -> ASensor::EndPlay
# dereferencing the sensor's empty stream token).
COPY carla-dtt.patch /tmp/carla-dtt.patch
RUN git apply --include='LibCarla/*' \
              --include='Unreal/CarlaUE4/Plugins/Carla/Source/*' \
              /tmp/carla-dtt.patch && \
    git status --short

# ------------------------------------------------------------------------------
# Stage 3: CARLA content download (patched URL), isolated and cached
# ------------------------------------------------------------------------------
COPY patch_update.py /tmp/patch_update.py
RUN sed -i \
      's#http://carla-assets\.s3\.amazonaws\.com/#https://carla-assets.s3.us-east-005.backblazeb2.com/#' \
      Update.sh && \
    ! grep -qF 'carla-assets.s3.amazonaws.com' Update.sh && \
    grep -qF 'backblazeb2.com' Update.sh && \
    python3 /tmp/patch_update.py

ENV CARLA_CONTENT_CACHE=/home/carla/.cache/carla-content
RUN --mount=type=cache,id=carla-content-0.9.15,target=/home/carla/.cache/carla-content,uid=1000,gid=1000,sharing=locked \
    ./Update.sh

# ------------------------------------------------------------------------------
# Stage 4: dependency/build patches (each one is verified, because sed -i
# silently does nothing when a pattern does not match)
# ------------------------------------------------------------------------------
RUN sed -i \
      's#https://boostorg\.jfrog\.io/artifactory/main/release/#https://archives.boost.io/release/#' \
      Util/BuildTools/Setup.sh && \
    sed -i \
      's#https://carla-releases\.s3\.eu-west-3\.amazonaws\.com/Backup/#https://carla-releases.s3.us-east-005.backblazeb2.com/Backup/#' \
      Util/BuildTools/Setup.sh && \
    sed -i \
      's#https://sourceforge\.net/projects/libpng/files/libpng16/${LIBPNG_VERSION}/#https://sourceforge.net/projects/libpng/files/libpng16/older-releases/${LIBPNG_VERSION}/#' \
      Util/BuildTools/Setup.sh && \
    sed -i \
      's#https://github\.com/NixOS/patchelf/archive/${PATCHELF_VERSION}\.tar\.gz#https://github.com/NixOS/patchelf/archive/refs/tags/${PATCHELF_VERSION}.tar.gz#' \
      Util/BuildTools/Setup.sh && \
    sed -i 's/^    git fetch$/    git fetch origin ${CURRENT_STREETMAP_COMMIT} --depth=1/' Util/BuildTools/BuildUE4Plugins.sh && \
    sed -i '/^project(CARLA)$/a set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)' CMakeLists.txt && \
    ! grep -qF 'boostorg.jfrog.io' Util/BuildTools/Setup.sh && \
    ! grep -qF 'carla-releases.s3.eu-west-3.amazonaws.com' Util/BuildTools/Setup.sh && \
    ! grep -qF 'libpng16/${LIBPNG_VERSION}/' Util/BuildTools/Setup.sh && \
    ! grep -qF 'archive/${PATCHELF_VERSION}.tar.gz' Util/BuildTools/Setup.sh && \
    ! grep -qxF '    git fetch' Util/BuildTools/BuildUE4Plugins.sh && \
    grep -qF 'CMAKE_BUILD_WITH_INSTALL_RPATH' CMakeLists.txt

# Stage 4b: extra build tools missing from the stock CARLA package list
USER root
RUN apt-get update && \
    apt-get install -y --no-install-recommends automake && \
    rm -rf /var/lib/apt/lists/*
USER carla

ENV PATH="/home/carla/UE4.26/Engine/Binaries/Linux:/home/carla/carla/Util:${PATH}"
WORKDIR /home/carla/carla

ENTRYPOINT ["/bin/bash"]
EOF_DOCKERFILE

# A base image is stale when the phase-1 build inputs (Dockerfile, the
# Update.sh patch helper, the DTT patch) differ from the ones it was built
# with (e.g. a new patch stage or a changed DTT patch). The inputs' hash is
# stamped into the base image as a label at build time; a missing label
# (images built before this check existed) also counts as stale. Rebuilding a
# stale base reuses Docker's layer cache, so the expensive engine layer is
# not rebuilt.
EXPECTED_INPUTS_SHA="$(cat "$TMP_DIR/Dockerfile" "$TMP_DIR/patch_update.py" "$TMP_DIR/carla-dtt.patch" | sha256sum | cut -d' ' -f1)"
BASE_INPUTS_SHA=""
if have_base_image; then
  BASE_INPUTS_SHA="$(docker inspect --format '{{index .Config.Labels "build_inputs_sha256"}}' "$BASE_IMAGE" 2>/dev/null || true)"
fi

NEED_PHASE_1=0
STALE_BASE=0
if [[ "$FROM_TARBALL" -eq 0 ]]; then
  if ! have_base_image; then
    NEED_PHASE_1=1
  elif [[ "$NO_CACHE" -eq 1 ]]; then
    NEED_PHASE_1=1
  elif [[ "$BASE_INPUTS_SHA" != "$EXPECTED_INPUTS_SHA" ]]; then
    NEED_PHASE_1=1
    STALE_BASE=1
  fi
fi
if [[ "$NEED_PHASE_1" -eq 1 ]]; then
  # Phase 1: the expensive engine layer lives in Docker's storage, not on the
  # filesystem below.
  DOCKER_ROOT="$(docker info --format '{{.DockerRootDir}}' 2>/dev/null || true)"
  if [[ -n "$DOCKER_ROOT" ]]; then
    AVAIL="$(free_kib "$DOCKER_ROOT")"
    NEED=$((150 * 1024 * 1024))
    if [[ "$AVAIL" -lt "$NEED" ]]; then
      die "Building the base image needs ~150 GB of free space on the Docker
storage filesystem ($DOCKER_ROOT); only $((AVAIL / 1024 / 1024)) GB are free."
    fi
  fi
  if [[ "$STALE_BASE" -eq 1 && -z "${EPIC_USER:-}${EPIC_PASS:-}" ]]; then
    # A stale-base rebuild is served from the layer cache: every layer up to
    # the changed stage (including the engine clone that uses the credentials)
    # is a cache hit, so no real credentials are needed.
    echo "NOTE: EPIC_USER/EPIC_PASS are not set. The stale-base rebuild relies"
    echo "on the Docker layer cache; if the cache is cold, the build will fail"
    echo "at the Unreal Engine clone stage."
    EPIC_USER="unused"
    EPIC_PASS="unused"
  else
    : "${EPIC_USER:?EPIC_USER must be set (GitHub user with access to CarlaUnreal/UnrealEngine)}"
    : "${EPIC_PASS:?EPIC_PASS must be set (GitHub token with access to CarlaUnreal/UnrealEngine)}"
  fi
fi

# Host-side folders. The container user (uid 1000) must be able to write to
# them even if your host uid differs, hence the permissive mode.
mkdir -p "$BUILD_DIR" "$OUTPUT_DIR"
chmod 777 "$BUILD_DIR" "$OUTPUT_DIR"
AVAIL="$(free_kib "$OUTPUT_DIR")"
NEED=$((60 * 1024 * 1024))
if [[ "$AVAIL" -lt "$NEED" ]]; then
  die "Phases 2-3 need ~60 GB of free space in $OUTPUT_DIR (release tarball,
staging area, build context); only $((AVAIL / 1024 / 1024)) GB are free."
fi

# Keep Ctrl-C working when run from a terminal; always allocate a pseudo-TTY
# so the build output is unbuffered, i.e. live.
TTY_ARGS=(-t)
if [[ -t 0 ]]; then
  TTY_ARGS=(-i -t)
fi

# ------------------------------------------------------------------------------
# Phase 1: base image (UE4, CARLA source, content, patches) - layer cached
# ------------------------------------------------------------------------------
if [[ "$FROM_TARBALL" -eq 1 ]]; then
  echo "Phase 1/3: skipped (--from-tarball)"
  echo
elif [[ "$NEED_PHASE_1" -eq 0 ]]; then
  echo "Phase 1/3: base image ${BASE_IMAGE} already exists, skipping"
  echo
else
  cat > "$NETRC_FILE" <<EOF_NETRC
machine github.com
  login ${EPIC_USER}
  password ${EPIC_PASS}
EOF_NETRC
  chmod 600 "$NETRC_FILE"

  BUILD_ARGS=(
    --build-arg "CARLA_BRANCH=${CARLA_BRANCH}"
    --build-arg "UNREAL_BRANCH=${UNREAL_BRANCH}"
    --secret "id=github_netrc,src=${NETRC_FILE}"
    --label "build_inputs_sha256=${EXPECTED_INPUTS_SHA}"
    --progress=plain
    --load
    -t "${BASE_IMAGE}"
    -f "$TMP_DIR/Dockerfile"
  )

  if [[ "$NO_CACHE" -eq 1 ]]; then
    BUILD_ARGS+=(--no-cache)
  fi

  if [[ "$STALE_BASE" -eq 1 ]]; then
    echo "Phase 1/3: base image ${BASE_IMAGE} is stale (built from a different"
    echo "Dockerfile); rebuilding it. The layer cache is reused, so the"
    echo "Unreal Engine layer is NOT rebuilt."
  else
    echo "Phase 1/3: building base image ${BASE_IMAGE}"
    echo
    echo "The Unreal Engine build takes hours. Do NOT use --no-cache for normal retries."
  fi
  echo "  CARLA branch  : ${CARLA_BRANCH}"
  echo "  UE branch     : ${UNREAL_BRANCH}"
  echo "  Content cache : carla-content-0.9.15 (BuildKit cache mount)"
  echo

  docker buildx build "${BUILD_ARGS[@]}" "$TMP_DIR"
  echo
fi

# ------------------------------------------------------------------------------
# Phase 2: make setup + make package with host-mounted Build/
# ------------------------------------------------------------------------------
if [[ "$FROM_TARBALL" -eq 1 ]]; then
  echo "Phase 2/3: skipped (--from-tarball)"
  echo
else
echo "Phase 2/3: make setup + make package (config: ${PACKAGE_CONFIG})"
echo "  Host Build dir: ${BUILD_DIR}"
echo "  (mounted at /home/carla/carla/Build, reused on retries)"
echo "  This cooks all CARLA maps with the fixed engine and builds the server"
echo "  and client binaries; expect one to three hours."
echo

# Script that runs inside the container. Quoted heredoc: nothing is expanded
# here; PACKAGE_CONFIG arrives as an environment variable from `docker run -e`.
cat > "$TMP_DIR/package.sh" <<'EOF_PACKAGE'
set -Eeuo pipefail

cd /home/carla/carla

make setup
make package ARGS="--config=${PACKAGE_CONFIG}"

# Collect the newest release tarball (a previous run may have left one behind).
TARBALL="$(ls -1t Dist/CARLA_*.tar.gz | head -n1)"
echo "==> Release tarball: ${TARBALL##*/}"
cp -f "$TARBALL" /output/
chmod a+r "/output/${TARBALL##*/}" || true
echo "==> Copied to /output: ${TARBALL##*/}"
echo "==> Done (container phase)"
EOF_PACKAGE
chmod +x "$TMP_DIR/package.sh"

LOG_FILE="$OUTPUT_DIR/carla-package-$(date +%Y%m%d-%H%M%S).log"

docker rm -f "$BUILD_CONTAINER" >/dev/null 2>&1 || true
set +e
docker run --name "$BUILD_CONTAINER" \
  "${TTY_ARGS[@]}" \
  --entrypoint /bin/bash \
  -e GIT_DISCOVERY_ACROSS_FILESYSTEM=1 \
  -e "PACKAGE_CONFIG=${PACKAGE_CONFIG}" \
  -v "${BUILD_DIR}:/home/carla/carla/Build" \
  -v "${OUTPUT_DIR}:/output" \
  -v "$TMP_DIR/package.sh:/package.sh:ro" \
  "${BASE_IMAGE}" /package.sh 2>&1 | tee "$LOG_FILE"
STATUS="${PIPESTATUS[0]}"
set -e

# The pseudo-TTY adds carriage returns; remove them from the saved log.
sed -i 's/\r$//' "$LOG_FILE" 2>/dev/null || true

if [[ "$STATUS" -ne 0 ]]; then
  echo
  if grep -qE 'DistanceFieldVolumeTexture|DistanceFieldAtlas' "$LOG_FILE" 2>/dev/null; then
    echo "The log contains the distance-field crash signature from carla-simulator/carla#8696."
    echo "This means the Unreal Engine fork inside ${BASE_IMAGE} predates the fix"
    echo "(commit a288ae07 on the CarlaUnreal/UnrealEngine 'carla' branch)."
    echo "Rebuild the base image from the current 'carla' branch: rerun with --no-cache."
    echo
  fi
  echo "FAILED (exit code ${STATUS}). Full log: ${LOG_FILE}" >&2
  exit "$STATUS"
fi
fi

# ------------------------------------------------------------------------------
# Phase 3: extract the release and build the slim runtime image
# ------------------------------------------------------------------------------
echo
echo "Phase 3/3: building the runtime image ${IMAGE_NAME} (base: ${RUNTIME_BASE})"

shopt -s nullglob
TARBALLS=("$OUTPUT_DIR"/CARLA_*.tar.gz)
[[ ${#TARBALLS[@]} -gt 0 ]] || die "No CARLA_*.tar.gz found in ${OUTPUT_DIR} (phase 2 produces it; for --from-tarball, run a full build first)"
# Newest first, in case an earlier run left a tarball with the same name.
TARBALL="$(ls -1t "${TARBALLS[@]}" | head -n1)"
echo "  Release tarball: ${TARBALL}"

STAGING="$OUTPUT_DIR/staging"
echo "  Extracting to ${STAGING}"
rm -rf "$STAGING"
mkdir -p "$STAGING"
tar -xzf "$TARBALL" -C "$STAGING"
[[ -f "$STAGING/CarlaUE4.sh" ]] || die "CarlaUE4.sh not found in the extracted build - is this a valid CARLA package?"
for entry in ImportAssets.sh Import CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping; do
  [[ -e "$STAGING/$entry" ]] || die "'$entry' missing from the extracted build"
done

# Record which commits the packaged CARLA was built from, so the fix can be
# verified later with: docker inspect --format '{{index .Config.Labels ...}}'
LABELS=()
if have_base_image; then
  UE_COMMIT="$(docker run --rm --entrypoint /bin/bash "$BASE_IMAGE" -c 'git -C /home/carla/UE4.26 rev-parse HEAD')"
  CARLA_COMMIT="$(docker run --rm --entrypoint /bin/bash "$BASE_IMAGE" -c 'git -C /home/carla/carla rev-parse HEAD')"
  LABELS+=(--label "carla_ue4_commit=${UE_COMMIT}" --label "carla_commit=${CARLA_COMMIT}")
else
  UE_COMMIT="(base image ${BASE_IMAGE} not present, label omitted)"
  CARLA_COMMIT="(base image ${BASE_IMAGE} not present, label omitted)"
fi

# When the base image was (re)built in this invocation, the package was
# definitely built from the DTT-patched CARLA source, so record the patch hash.
# (With a pre-existing base image or --from-tarball the provenance is unknown,
# so no label is added.)
DTT_PATCH_SHA=""
if [[ "$NEED_PHASE_1" -eq 1 && -f "$SCRIPT_DIR/patches/carla-0.9.15-digital-twin.patch" ]]; then
  DTT_PATCH_SHA="$(sha256sum "$SCRIPT_DIR/patches/carla-0.9.15-digital-twin.patch" | cut -d' ' -f1)"
  LABELS+=(--label "carla_dtt_patch_sha256=${DTT_PATCH_SHA}")
fi

# Dockerfile for the runtime container: only the packaged CARLA, no engine
# source, no editor, no toolchain.
cat > "$TMP_DIR/Dockerfile.carla" <<'EOF_CARLA'
# Runtime-only image for a CARLA release built from the fixed Unreal fork.
# Build context: the extracted CARLA release folder (see build script).
ARG RUNTIME_BASE=nvidia/vulkan:1.1.121-cuda-10.1--ubuntu18.04
FROM ${RUNTIME_BASE}

# Same runtime libraries as the official CARLA release Dockerfile (which also
# fetches the NVIDIA apt key first, because the nvidia/vulkan base has the
# CUDA apt repo configured but its signing key is not installed; the || true
# keeps this working for plain ubuntu RUNTIME_BASE overrides). Plus
# xdg-user-dirs: UE's UserDir() calls `xdg-user-dir DOCUMENTS` at startup and
# prints an error line when the binary is missing.
RUN (apt-key adv --fetch-keys "https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/3bf863cc.pub" || true) && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      libsdl2-2.0 \
      xserver-xorg \
      libvulkan1 \
      libomp5 \
      xdg-user-dirs && \
    rm -rf /var/lib/apt/lists/*

RUN useradd -m -u 1000 -s /bin/bash carla
COPY --chown=carla:carla . /home/carla
USER carla
WORKDIR /home/carla
ENV CARLA_ROOT=/home/carla

# No ENTRYPOINT: commands passed to `docker run` must be executed verbatim,
# exactly like the official carlasim/carla image (start-carla-docker.sh relies
# on this, e.g. `docker run <image> /bin/bash -c "..."`).
CMD ["./CarlaUE4.sh"]
EOF_CARLA

docker build \
  --build-arg "RUNTIME_BASE=${RUNTIME_BASE}" \
  "${LABELS[@]}" \
  -t "$IMAGE_NAME" \
  -f "$TMP_DIR/Dockerfile.carla" \
  "$STAGING"

# Filesystem smoke test (no GPU needed). The real test is starting the server.
docker run --rm --entrypoint /bin/bash "$IMAGE_NAME" -c \
  'test -x /home/carla/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping && \
   test -f /home/carla/CarlaUE4.sh && \
   test -f /home/carla/ImportAssets.sh && \
   test -d /home/carla/Import'

rm -rf "$STAGING"

echo
echo "Build complete: ${IMAGE_NAME}"
echo "  UE fork commit  : ${UE_COMMIT}  (contains the #8696 distance-field fix)"
echo "  CARLA commit    : ${CARLA_COMMIT}"
if [[ -n "$DTT_PATCH_SHA" ]]; then
  echo "  DTT C++ patch   : sha256:${DTT_PATCH_SHA:0:16}... (runtime subset applied)"
fi
echo "  Release tarball : ${TARBALL}  (safe to delete)"
echo "  Build dir       : ${BUILD_DIR}"
if [[ -n "$LOG_FILE" ]]; then
  echo "  Log             : ${LOG_FILE}"
fi
echo
echo "Start it (optionally importing a cooked Digital Twin map package):"
echo "  bash docker/start-carla-docker.sh [path/to/cooked_map.tar.gz]"
echo
echo "Notes:"
echo "  - The final image contains only the packaged CARLA (no UE source/editor)."
echo "  - CARLA source = stock ${CARLA_BRANCH} + the DTT C++ patch (runtime"
echo "    subset of docker/patches/carla-0.9.15-digital-twin.patch) + fixed"
echo "    engine fork. Rebuilding the base image is needed whenever the"
echo "    patch or the engine fork changes."
