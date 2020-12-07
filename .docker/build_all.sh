#!/usr/bin/env bash

branch=$(git branch --show-current)

echo -e "Building amd and arm64 for $branch\n WARNING: this script must be run from the root of the repo not from within the .docker folder"

docker build --pull --rm -f ./.docker/Dockerfile  -t gdwyer/ram:$branch-amd64 .
docker build --pull --rm -f ./.docker/arm64/Dockerfile  -t gdwyer/ram:$branch-arm64 .
