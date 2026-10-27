#!/usr/bin/env bash
force=false
if [ "$1" = "--force" ]; then
  force=true
fi

# Only match this project's server images, not e.g. carla-editor containers.
containerid=$(docker ps --format '{{.ID}} {{.Image}}' \
    | awk '$2 ~ /carla-0\.9\.15|carlasim\/carla|carla-synergies/ {print $1}')
if [ -z "$containerid" ]; then
  echo "No running Carla container found."
else
  if [ "$force" = true ]; then
    docker rm -f "$containerid" > /dev/null 2>&1
    echo "Container $containerid killed."
  else
    trap 'echo; echo "Kill aborted."; exit 130' INT
    printf "Found container: %s. Kill? [enter or Ctrl+C] " "$containerid"
    read -s -r
    echo
    docker rm -f "$containerid" > /dev/null 2>&1
    echo "Container $containerid killed."
  fi
fi
