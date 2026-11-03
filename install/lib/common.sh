#!/usr/bin/env bash
# Shared colours and version are consumed by the installer UI.
# shellcheck disable=SC2034
# =============================================================================
# HORUS Installer - Common Utilities
#
# Logging, prompts, and shared helpers. All user-facing log functions also
# write to the log file (via _log_to_file from ui.sh).
# =============================================================================

set -o pipefail

HORUS_INSTALLER_VERSION="0.1.0"

# ---------------------------------------------------------------------------
# Color codes (disabled when not a TTY)
# ---------------------------------------------------------------------------
if [ -t 1 ] && [ -z "${NO_COLOR+x}" ] && [ "${TERM:-dumb}" != dumb ]; then
  C_RESET='\033[0m'
  C_BOLD='\033[1m'
  C_CYAN='\033[1;96m'
  C_BLUE='\033[1;34m'
  C_MAGENTA='\033[1;95m'
  C_PURPLE='\033[1;35m'
  C_LIGHT_RED='\033[1;91m'
  C_GREEN='\033[1;32m'
  C_YELLOW='\033[1;33m'
  C_RED='\033[1;31m'
  C_DIM='\033[2m'
else
  C_RESET=''
  C_BOLD=''
  C_CYAN=''
  C_BLUE=''
  C_MAGENTA=''
  C_PURPLE=''
  C_LIGHT_RED=''
  C_GREEN=''
  C_YELLOW=''
  C_RED=''
  C_DIM=''
fi

# ---------------------------------------------------------------------------
# Logging (user-facing + log file)
# ---------------------------------------------------------------------------

log_info() {
  printf "  %b[horus]%b %s\n" "$C_BLUE" "$C_RESET" "$*"
  _log_to_file "[info] $*" 2>/dev/null || true
}

log_success() {
  printf "  %b[ok]%b %s\n" "$C_GREEN" "$C_RESET" "$*"
  _log_to_file "[ok] $*" 2>/dev/null || true
}

log_warn() {
  printf "  %b[warn]%b %s\n" "$C_YELLOW" "$C_RESET" "$*"
  _log_to_file "[warn] $*" 2>/dev/null || true
}

log_error() {
  printf "  %b[error]%b %s\n" "$C_RED" "$C_RESET" "$*" >&2
  _log_to_file "[error] $*" 2>/dev/null || true
}

die() {
  log_error "$*"
  exit 1
}

# ---------------------------------------------------------------------------
# Command helpers
# ---------------------------------------------------------------------------

command_exists() {
  command -v "$1" >/dev/null 2>&1
}

# ---------------------------------------------------------------------------
# Prompt helpers
# ---------------------------------------------------------------------------

read_prompt_value() {
  local prompt="$1"
  local result_var="$2"
  local _prompt_answer="" _prompt_fd
  # stdin can be a bootstrap pipe even when a controlling terminal exists.
  { exec {_prompt_fd}<>/dev/tty; } 2>/dev/null || die "No terminal available. Re-run with --yes for unattended installation."
  printf '%s' "$prompt" >&"$_prompt_fd"
  if ! IFS= read -r _prompt_answer <&"$_prompt_fd"; then
    exec {_prompt_fd}>&-
    die "Input ended; installation cancelled."
  fi
  exec {_prompt_fd}>&-
  printf -v "$result_var" '%s' "$_prompt_answer"
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

# ---------------------------------------------------------------------------
# Path helpers
# ---------------------------------------------------------------------------

normalize_path() {
  local raw="$1"
  [ -n "$raw" ] || die "Path must not be empty"
  # Match a literal tilde supplied through a quoted command-line argument.
  # shellcheck disable=SC2088
  case "$raw" in
    '~') raw="$HOME" ;;
    '~/'*) raw="$HOME/${raw:2}" ;;
  esac
  [[ "$raw" != *$'\n'* && "$raw" != *$'\r'* && "$raw" != *:* ]] || die "Paths must not contain newlines or colons"
  realpath -m -- "$raw"
}

timestamp_utc() {
  date -u +"%Y-%m-%dT%H:%M:%SZ"
}

is_wsl() {
  grep -qiE "microsoft|wsl" /proc/version 2>/dev/null
}
