xhost +

# Check if password.env file exists and set the env_file_option variable accordingly
env_file_option=""
if [ -f ./password.env ]; then
  env_file_option="--env-file ./password.env"
fi

# Default action is to build
action="pull"

# Parse flags
while [[ "$#" -gt 0 ]]; do
  case $1 in
    --pull)
      action="pull"
      ;;
    --build)
      action="build"
      ;;
    --push)
      action="push"
      ;;
    *)
      echo "Usage: $0 [--pull] [--build] [--push]"
      exit 1
      ;;
  esac
  shift
done

# Perform the action
if [ "$action" == "pull" ]; then
  docker compose $env_file_option -f ./docker-compose.yml pull
  docker compose $env_file_option -f ./docker-compose.yml up --detach
elif [ "$action" == "push" ]; then
  docker compose $env_file_option -f ./docker-compose.yml build
  docker compose $env_file_option -f ./docker-compose.yml push
  docker compose $env_file_option -f ./docker-compose.yml up --detach
else
  docker compose $env_file_option -f ./docker-compose.yml build
  docker compose $env_file_option -f ./docker-compose.yml up --detach
fi

docker exec -it docker_isaac bash