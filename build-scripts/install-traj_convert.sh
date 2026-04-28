#!/usr/bin/env bash
set -euo pipefail

# Require three positional args: CONDA ENV_NAME PIP
if [ "$#" -ne 3 ]; then
	>&2 echo "Usage: $0 CONDA ENV_NAME PIP"
	>&2 echo "Example: $0 /path/to/conda my-env pip"
	exit 1
fi

CONDA="$1"
ENV_NAME="$2"
PIP="$3"

# Make the repo directory a variable
OSI_DIR="osi-gen"

# Remove the directory if it exists
if [ -d "$OSI_DIR" ]; then
	rm -rf "$OSI_DIR"
fi

git clone git@github.com:das-rise/osi-gen.git "$OSI_DIR"
cd "$OSI_DIR"/python && "$CONDA" run -n "$ENV_NAME" "$PIP" install . || exit 1
cd -
rm -rf "$OSI_DIR"
exit 0