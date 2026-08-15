#!/bin/bash
set -e

source ${COLCON_WS}/install/setup.bash

exec "$@"
