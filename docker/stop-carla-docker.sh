#!/usr/bin/env bash
force=false
if [ "$1" = "--force" ]; then
  force=true
fi

containerid=$(docker ps | grep carla | awk ' { print $1 } ')
if [ -z "$containerid" ]; then
  echo "No running Carla container found."
else
  if [ "$force" = true ]; then
    docker kill "$containerid" > /dev/null 2>&1
    echo "Container $containerid killed."
  else
    trap 'echo; echo "Kill aborted."; exit 130' INT
    printf "Found container: %s. Kill? [enter or Ctrl+C] " "$containerid"
    read -s -r
    echo
    docker kill "$containerid" > /dev/null 2>&1
    echo "Container $containerid killed."
  fi
fi