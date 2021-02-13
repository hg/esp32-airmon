#!/usr/bin/bash

set -euo pipefail

cd "$(dirname "$0")"

rm --force airmon

podman run --rm -it -v .:/app golang:buster bash -c 'cd /app && go build .'

