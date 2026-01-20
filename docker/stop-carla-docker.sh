containerid=$(docker ps | grep carla | awk ' { print $1 } ')
if [ -z "$containerid" ]; then
  echo "No running Carla container found."
else
  echo "Found container: ${containerid}. Kill? [enter]"
  read
  docker kill ${containerid}
fi

