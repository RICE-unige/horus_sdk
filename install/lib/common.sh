#!/usr/bin/env bash

set -o pipefail

HORUS_INSTALLER_VERSION="0.1.0"

if [ -t 1 ]; then
  C_RESET='\033[0m'
  C_BLUE='\033[1;34m'
  C_GREEN='\033[1;32m'
  C_YELLOW='\033[1;33m'
  C_RED='\033[1;31m'
  C_DIM='\033[2m'
else
  C_RESET=''
  C_BLUE=''
  C_GREEN=''
  C_YELLOW=''
  C_RED=''
  C_DIM=''
fi

log_info() {
  printf "%b[horus]%b %s\n" "$C_BLUE" "$C_RESET" "$*"
}

log_success() {
  printf "%b[ok]%b %s\n" "$C_GREEN" "$C_RESET" "$*"
}

log_warn() {
  printf "%b[warn]%b %s\n" "$C_YELLOW" "$C_RESET" "$*"
}

log_error() {
  printf "%b[error]%b %s\n" "$C_RED" "$C_RESET" "$*" >&2
}

die() {
  log_error "$*"
  exit 1
}

command_exists() {
  command -v "$1" >/dev/null 2>&1
}

run_cmd() {
  log_info "Running: $*"
  "$@"
}

NO_TTY_PROMPT_WARNED=0

read_prompt_value() {
  local prompt="$1"
  local result_var="$2"
  local response=""

  # Always initialize caller-side variable to avoid nounset issues.
  printf -v "$result_var" '%s' ""

  if tty -s; then
    if ! read -r -p "$prompt" response < /dev/tty; then
      response=""
    fi
  else
    if [ "$NO_TTY_PROMPT_WARNED" -eq 0 ]; then
      log_warn "No interactive TTY detected; installer will use default prompt values."
      NO_TTY_PROMPT_WARNED=1
    fi
    response=""
  fi

  printf -v "$result_var" '%s' "$response"
}

prompt_with_default() {
  local prompt="$1"
  local default="$2"
  local response=""

  read_prompt_value "$prompt [$default]: " response
  if [ -z "$response" ]; then
    printf "%s" "$default"
  else
    printf "%s" "$response"
  fi
}

prompt_with_default_into() {
  local result_var="$1"
  local prompt="$2"
  local default="$3"
  local response=""

  read_prompt_value "$prompt [$default]: " response
  if [ -z "$response" ]; then
    response="$default"
  fi

  printf -v "$result_var" '%s' "$response"
}

prompt_yes_no() {
  local prompt="$1"
  local default="$2"
  local response=""
  local hint

  if [ "$default" = "yes" ]; then
    hint="Y/n"
  else
    hint="y/N"
  fi

  read_prompt_value "$prompt ($hint): " response
  response="${response,,}"

  if [ -z "$response" ]; then
    response="$default"
  fi

  case "$response" in
    y|yes) return 0 ;;
    n|no) return 1 ;;
    *)
      log_warn "Invalid response '$response', using default '$default'."
      [ "$default" = "yes" ]
      ;;
  esac
}

normalize_path() {
  local raw="$1"
  if [[ "$raw" == ~* ]]; then
    printf "%s" "${raw/#\~/$HOME}"
  else
    printf "%s" "$raw"
  fi
}

timestamp_utc() {
  date -u +"%Y-%m-%dT%H:%M:%SZ"
}

is_wsl() {
  grep -qiE "microsoft|wsl" /proc/version 2>/dev/null
}
