containerid=$(docker ps | grep carla | awk ' { print $1 } ')
if [ -z "$containerid" ]; then
  echo "No running Carla container found."
else
  trap 'echo; echo "Kill aborted."; exit 130' INT
  printf "Found container: %s. Kill? [enter or Ctrl+C] " "$containerid"
  read -s -r  # -s = silent, input is not echoed
  echo    # add newline
  docker kill "$containerid" > /dev/null 2>&1
  echo "Container $containerid killed."
fi

