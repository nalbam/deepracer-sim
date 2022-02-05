#!/bin/bash

OS_NAME="$(uname | awk '{print tolower($0)}')"

SHELL_DIR=$(dirname $0)

URL_SIMAPP="https://deepracer-melodic-managed-resources-us-east-1.s3.amazonaws.com/deepracer-simapp.tar.gz"

# command -v tput > /dev/null && TPUT=true
TPUT=

_echo() {
  if [ "${TPUT}" != "" ] && [ "$2" != "" ]; then
    echo -e "$(tput setaf $2)$1$(tput sgr0)"
  else
    echo -e "$1"
  fi
}

_result() {
  _echo "# $@" 4
}

_command() {
  _echo "$ $@" 3
}

_success() {
  _echo "+ $@" 2
  exit 0
}

_error() {
  _echo "- $@" 1
  # exit 1
}

_prepare() {
  _command "_prepare"

  YYYY=$(date +%Y)
  MM=$(date +%m)

  # rm -rf ${SHELL_DIR}/build

  mkdir -p ${SHELL_DIR}/build

  echo
}

_build() {
  pushd ${SHELL_DIR}/build

  curl -sL -o deepracer-simapp.tar.gz ${URL_SIMAPP}

  tar -xvf deepracer-simapp.tar.gz
  tar -xvf bundle.tar

  popd

  ROUTE_PATH=${SHELL_DIR}/build/opt/install/deepracer_simulation_environment/share/deepracer_simulation_environment/routes/

  if [ -d ${ROUTE_PATH} ]; then
    rsync -avu --delete ${ROUTE_PATH} ${SHELL_DIR}/routes/
  fi

  # commit message
  printf "$(date +%Y%m%d-%H%M)" >${SHELL_DIR}/build/commit_message.txt
}

_run() {
  _prepare

  _build

  _success
}

_run
