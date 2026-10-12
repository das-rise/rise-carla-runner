#!/bin/bash
# Download PCLA pretrained weights and runtime assets.
#
# Must run inside the rirun conda environment (needs huggingface_hub, requests, tqdm):
#   conda run -n rirun bash build-scripts/install-weights.sh
#
# Optionally download only specific agent base names (top-level keys of
# src/rirun/PCLA/agents.json, e.g. minddrive orion tfv4):
#   conda run -n rirun bash build-scripts/install-weights.sh minddrive orion tfv4
set -e

# Run relative to the repository root regardless of caller location.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$(dirname "$SCRIPT_DIR")"

if [ "$#" -gt 0 ]; then
    echo "→ Downloading pretrained weights for: $*"
    python src/rirun/PCLA/pcla_functions/download_weights.py --agents "$@"
else
    echo "→ Downloading all pretrained weights (this may take a while)..."
    python src/rirun/PCLA/pcla_functions/download_weights.py --all
fi

echo "→ Downloading runtime assets (BEV maps, speed limits, town pickles)..."
python src/rirun/PCLA/pcla_functions/download_assets.py

echo "→ PCLA weights and assets installed"
