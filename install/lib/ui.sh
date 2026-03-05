#!/usr/bin/env bash
# =============================================================================
# HORUS Installer UI Primitives
#
# Provides: banner, spinner, step tracking, run_silent, config/completion
# summaries, and cleanup traps. All presentation logic is isolated here.
# =============================================================================

set -o pipefail

# ---------------------------------------------------------------------------
# TTY detection (set once at source-time)
# ---------------------------------------------------------------------------
IS_TTY=0
if [ -t 1 ]; then
  IS_TTY=1
fi

# ---------------------------------------------------------------------------
# Step tracking globals
# ---------------------------------------------------------------------------
STEP_CURRENT=0
STEP_TOTAL=0

# ---------------------------------------------------------------------------
# Spinner state
# ---------------------------------------------------------------------------
_SPINNER_PID=""
_SPINNER_MSG=""

# ---------------------------------------------------------------------------
# Timing
# ---------------------------------------------------------------------------
_INSTALL_START_TIME=""

# ===========================================================================
# Banner
# ===========================================================================

print_banner() {
  if [ "$IS_TTY" -eq 1 ]; then
    local sdk_cyan sdk_blue sdk_magenta sdk_purple sdk_red
    sdk_cyan=$'\033[96m'
    sdk_blue=$'\033[94m'
    sdk_magenta=$'\033[95m'
    sdk_purple=$'\033[35m'
    sdk_red=$'\033[91m'

    printf '\n'

    printf '    %b%s%b%b%s%b%b%s%b%b%s%b%b%s%b\n' \
      "${sdk_cyan}" $'\u2588\u2588\u2557  \u2588\u2588\u2557 ' "${C_RESET}" \
      "${sdk_blue}" $'\u2588\u2588\u2588\u2588\u2588\u2588\u2557 ' "${C_RESET}" \
      "${sdk_magenta}" $'\u2588\u2588\u2588\u2588\u2588\u2588\u2557 ' "${C_RESET}" \
      "${sdk_purple}" $'\u2588\u2588\u2557   \u2588\u2588\u2557' "${C_RESET}" \
      "${sdk_red}" $'\u2588\u2588\u2588\u2588\u2588\u2588\u2588\u2557' "${C_RESET}"

    printf '    %b%s%b%b%s%b%b%s%b%b%s%b%b%s%b\n' \
      "${sdk_cyan}" $'\u2588\u2588\u2551  \u2588\u2588\u2551' "${C_RESET}" \
      "${sdk_blue}" $'\u2588\u2588\u2554\u2550\u2550\u2550\u2588\u2588\u2557' "${C_RESET}" \
      "${sdk_magenta}" $'\u2588\u2588\u2554\u2550\u2550\u2588\u2588\u2557' "${C_RESET}" \
      "${sdk_purple}" $'\u2588\u2588\u2551   \u2588\u2588\u2551' "${C_RESET}" \
      "${sdk_red}" $'\u2588\u2588\u2554\u2550\u2550\u2550\u2550\u255d' "${C_RESET}"

    printf '    %b%s%b%b%s%b%b%s%b%b%s%b%b%s%b\n' \
      "${sdk_cyan}" $'\u2588\u2588\u2588\u2588\u2588\u2588\u2588\u2551' "${C_RESET}" \
      "${sdk_blue}" $'\u2588\u2588\u2551   \u2588\u2588\u2551' "${C_RESET}" \
      "${sdk_magenta}" $'\u2588\u2588\u2588\u2588\u2588\u2588\u2554\u255d' "${C_RESET}" \
      "${sdk_purple}" $'\u2588\u2588\u2551   \u2588\u2588\u2551' "${C_RESET}" \
      "${sdk_red}" $'\u2588\u2588\u2588\u2588\u2588\u2588\u2588\u2557' "${C_RESET}"

    printf '    %b%s%b%b%s%b%b%s%b%b%s%b%b%s%b\n' \
      "${sdk_cyan}" $'\u2588\u2588\u2554\u2550\u2550\u2588\u2588\u2551' "${C_RESET}" \
      "${sdk_blue}" $'\u2588\u2588\u2551   \u2588\u2588\u2551' "${C_RESET}" \
      "${sdk_magenta}" $'\u2588\u2588\u2554\u2550\u2550\u2588\u2588\u2557' "${C_RESET}" \
      "${sdk_purple}" $'\u2588\u2588\u2551   \u2588\u2588\u2551' "${C_RESET}" \
      "${sdk_red}" $'\u255a\u2550\u2550\u2550\u2550\u2588\u2588\u2551' "${C_RESET}"

    printf '    %b%s%b%b%s%b%b%s%b%b%s%b%b%s%b\n' \
      "${sdk_cyan}" $'\u2588\u2588\u2551  \u2588\u2588\u2551' "${C_RESET}" \
      "${sdk_blue}" $'\u255a\u2588\u2588\u2588\u2588\u2588\u2588\u2554\u255d' "${C_RESET}" \
      "${sdk_magenta}" $'\u2588\u2588\u2551  \u2588\u2588\u2551' "${C_RESET}" \
      "${sdk_purple}" $'\u255a\u2588\u2588\u2588\u2588\u2588\u2588\u2554\u255d' "${C_RESET}" \
      "${sdk_red}" $'\u2588\u2588\u2588\u2588\u2588\u2588\u2588\u2551' "${C_RESET}"

    printf '    %b%s%b%b%s%b%b%s%b%b%s%b%b%s%b\n' \
      "${sdk_cyan}" $'\u255a\u2550\u255d  \u255a\u2550\u255d ' "${C_RESET}" \
      "${sdk_blue}" $'\u255a\u2550\u2550\u2550\u2550\u2550\u255d ' "${C_RESET}" \
      "${sdk_magenta}" $'\u255a\u2550\u255d  \u255a\u2550\u255d ' "${C_RESET}" \
      "${sdk_purple}" $'\u255a\u2550\u2550\u2550\u2550\u2550\u255d ' "${C_RESET}" \
      "${sdk_red}" $'\u255a\u2550\u2550\u2550\u2550\u2550\u2550\u255d' "${C_RESET}"

    printf '\n'
    printf '    %b        Holistic Operational Reality%b\n' "${sdk_blue}" "${C_RESET}"
    printf '    %b            for Unified Systems%b\n' "${sdk_magenta}" "${C_RESET}"
    printf '\n'
    printf '    %bMixed Reality Robot Management Platform%b\n' "${C_DIM}" "${C_RESET}"
    printf '    %bRICE Lab, University of Genoa%b\n' "${C_DIM}" "${C_RESET}"
    printf '\n'
  else
    printf 'HORUS - Holistic Operational Reality for Unified Systems\n'
    printf 'Mixed Reality Robot Management Platform | RICE Lab, University of Genoa\n\n'
  fi
}


# ===========================================================================
# Log-file-only write helper
# ===========================================================================

_log_to_file() {
  if [ -n "${LOG_FILE:-}" ] && [ -d "$(dirname "$LOG_FILE" 2>/dev/null)" ]; then
    printf '[%s] %s\n' "$(date '+%H:%M:%S')" "$*" >>"$LOG_FILE" 2>/dev/null || true
  fi
}

# ===========================================================================
# Spinner
# ===========================================================================

_SPINNER_CHARS='|/-\'

_spinner_loop() {
  local msg="$1"
  local i=0
  local len=${#_SPINNER_CHARS}
  local start_time
  start_time=$(date +%s)

  # Hide cursor
  printf '\033[?25l' >&2

  while true; do
    local now
    now=$(date +%s)
    local elapsed=$(( now - start_time ))
    local mins=$(( elapsed / 60 ))
    local secs=$(( elapsed % 60 ))
    local time_str
    if [ "$mins" -gt 0 ]; then
      time_str="${mins}m${secs}s"
    else
      time_str="${secs}s"
    fi

    local char="${_SPINNER_CHARS:$((i % len)):1}"
    printf '\r    %b%s%b %s %b(%s)%b\033[K' \
      "${C_BLUE}" "$char" "${C_RESET}" \
      "$msg" \
      "${C_DIM}" "$time_str" "${C_RESET}"

    i=$(( i + 1 ))
    sleep 0.1
  done
}

spinner_start() {
  local msg="$1"
  _SPINNER_MSG="$msg"

  if [ "$IS_TTY" -eq 0 ]; then
    printf '    ... %s\n' "$msg"
    return 0
  fi

  _spinner_loop "$msg" &
  _SPINNER_PID=$!
  disown "$_SPINNER_PID" 2>/dev/null || true
}

spinner_stop() {
  local status="${1:-ok}"

  if [ "$IS_TTY" -eq 0 ]; then
    if [ "$status" = "ok" ]; then
      printf '    %b[ok]%b %s\n' "${C_GREEN}" "${C_RESET}" "$_SPINNER_MSG"
    else
      printf '    %b[FAIL]%b %s\n' "${C_RED}" "${C_RESET}" "$_SPINNER_MSG"
    fi
    return 0
  fi

  if [ -n "$_SPINNER_PID" ]; then
    kill "$_SPINNER_PID" 2>/dev/null || true
    wait "$_SPINNER_PID" 2>/dev/null || true
    _SPINNER_PID=""
  fi

  # Restore cursor
  printf '\033[?25s' >&2

  if [ "$status" = "ok" ]; then
    printf '\r    %b[ok]%b %s\033[K\n' "${C_GREEN}" "${C_RESET}" "$_SPINNER_MSG"
  else
    printf '\r    %b[FAIL]%b %s\033[K\n' "${C_RED}" "${C_RESET}" "$_SPINNER_MSG"
  fi
}

# ===========================================================================
# Trap cleanup (Ctrl+C, errors)
# ===========================================================================

_ui_cleanup() {
  if [ -n "$_SPINNER_PID" ]; then
    kill "$_SPINNER_PID" 2>/dev/null || true
    wait "$_SPINNER_PID" 2>/dev/null || true
    _SPINNER_PID=""
  fi
  if [ "$IS_TTY" -eq 1 ]; then
    printf '\033[?25s' >&2
  fi
}

setup_ui_traps() {
  trap '_ui_cleanup; exit 130' INT
  trap '_ui_cleanup; exit 143' TERM
  trap '_ui_cleanup' EXIT
}

# ===========================================================================
# Step tracking
# ===========================================================================

step_init() {
  STEP_TOTAL="$1"
  STEP_CURRENT=0
  _INSTALL_START_TIME=$(date +%s)
}

step_start() {
  local msg="$1"
  STEP_CURRENT=$(( STEP_CURRENT + 1 ))

  printf '\n  %b[%d/%d]%b %s\n' \
    "${C_BLUE}" "$STEP_CURRENT" "$STEP_TOTAL" "${C_RESET}" \
    "$msg"

  _log_to_file "=== [step $STEP_CURRENT/$STEP_TOTAL] $msg ==="
}

step_done() {
  local msg="${1:-}"
  if [ -n "$msg" ]; then
    printf '    %b[ok]%b %s\n' "${C_GREEN}" "${C_RESET}" "$msg"
  fi
  _log_to_file "[step $STEP_CURRENT/$STEP_TOTAL] completed"
}

step_fail() {
  local msg="${1:-failed}"
  printf '    %b[FAIL]%b %s\n' "${C_RED}" "${C_RESET}" "$msg"
  _log_to_file "[step $STEP_CURRENT/$STEP_TOTAL] FAILED: $msg"
}

# ===========================================================================
# run_silent - core command execution primitive
#
# Runs a command with all output captured to the log file. Shows a spinner
# while running. On failure, displays the last lines of the log.
#
# Usage: run_silent "description" command [args...]
# ===========================================================================

run_silent() {
  local description="$1"
  shift

  _log_to_file ">>> $description"
  _log_to_file ">>> command: $*"

  spinner_start "$description"

  local exit_code=0
  "$@" >>"$LOG_FILE" 2>&1 || exit_code=$?

  if [ "$exit_code" -eq 0 ]; then
    spinner_stop "ok"
    _log_to_file ">>> completed (exit 0): $description"
  else
    spinner_stop "fail"
    _log_to_file ">>> FAILED (exit $exit_code): $description"
    _show_log_tail "$description" "$exit_code"
    return "$exit_code"
  fi

  return 0
}

# ===========================================================================
# Error recovery - show last N lines of log on failure
# ===========================================================================

_show_log_tail() {
  local description="$1"
  local exit_code="$2"
  local tail_lines=20

  printf '\n'
  printf '    %b--- Last %d lines of log (exit code %d) ---%b\n' \
    "${C_RED}" "$tail_lines" "$exit_code" "${C_RESET}"

  if [ -f "$LOG_FILE" ]; then
    tail -n "$tail_lines" "$LOG_FILE" 2>/dev/null | while IFS= read -r line; do
      printf '    %b|%b %s\n' "${C_DIM}" "${C_RESET}" "$line"
    done
  fi

  printf '    %b--- End of log tail ---%b\n' "${C_RED}" "${C_RESET}"
  printf '\n'
  printf '    Full log: %s\n\n' "$LOG_FILE"
}

# ===========================================================================
# Configuration summary box
# ===========================================================================

print_config_summary() {
  local install_root="$1"
  local channel="$2"
  local ros_distro="$3"
  local webrtc="$4"
  local shell_config="$5"
  local os_info="$6"

  printf '\n'
  printf '  %b+----------------------------------------------------------+%b\n' "${C_DIM}" "${C_RESET}"
  printf '  %b|%b  %-56s  %b|%b\n' "${C_DIM}" "${C_RESET}" "" "${C_DIM}" "${C_RESET}"
  printf '  %b|%b  %b%-56s%b  %b|%b\n' "${C_DIM}" "${C_RESET}" "${C_BOLD}" "Installation Configuration" "${C_RESET}" "${C_DIM}" "${C_RESET}"
  printf '  %b|%b  %-56s  %b|%b\n' "${C_DIM}" "${C_RESET}" "" "${C_DIM}" "${C_RESET}"
  printf '  %b|%b  %-22s%-34s  %b|%b\n' "${C_DIM}" "${C_RESET}" "Install root:" "$install_root" "${C_DIM}" "${C_RESET}"
  printf '  %b|%b  %-22s%-34s  %b|%b\n' "${C_DIM}" "${C_RESET}" "Channel:" "$channel" "${C_DIM}" "${C_RESET}"
  printf '  %b|%b  %-22s%-34s  %b|%b\n' "${C_DIM}" "${C_RESET}" "ROS distro:" "$ros_distro" "${C_DIM}" "${C_RESET}"
  printf '  %b|%b  %-22s%-34s  %b|%b\n' "${C_DIM}" "${C_RESET}" "WebRTC:" "$webrtc" "${C_DIM}" "${C_RESET}"
  printf '  %b|%b  %-22s%-34s  %b|%b\n' "${C_DIM}" "${C_RESET}" "Shell config:" "$shell_config" "${C_DIM}" "${C_RESET}"
  printf '  %b|%b  %-22s%-34s  %b|%b\n' "${C_DIM}" "${C_RESET}" "Platform:" "$os_info" "${C_DIM}" "${C_RESET}"
  printf '  %b|%b  %-56s  %b|%b\n' "${C_DIM}" "${C_RESET}" "" "${C_DIM}" "${C_RESET}"
  printf '  %b+----------------------------------------------------------+%b\n' "${C_DIM}" "${C_RESET}"
  printf '\n'
}

# ===========================================================================
# Completion summary
# ===========================================================================

print_completion_summary() {
  local install_root="$1"
  local runtime_mode="${2:-full}"
  local runtime_reason="${3:-}"
  local elapsed_total
  elapsed_total=$(( $(date +%s) - _INSTALL_START_TIME ))
  local mins=$(( elapsed_total / 60 ))
  local secs=$(( elapsed_total % 60 ))

  local title_color="$C_GREEN"
  local title_text="   HORUS installation completed successfully               "
  if [ "$runtime_mode" != "full" ]; then
    title_color="$C_YELLOW"
    title_text="   HORUS installation completed with warnings              "
  fi

  printf '\n'
  printf '  %b===========================================================%b\n' "${title_color}" "${C_RESET}"
  printf '  %b%s%b\n' "${title_color}" "${title_text}" "${C_RESET}"
  printf '  %b===========================================================%b\n' "${title_color}" "${C_RESET}"
  printf '\n'

  if [ "$runtime_mode" != "full" ]; then
    if [ -n "$runtime_reason" ]; then
      printf '  %bRuntime note:%b %s\n' "${C_YELLOW}" "${C_RESET}" "$runtime_reason"
    fi
    printf '  %bUnity bridge was not built; this install is backend-only.%b\n' "${C_YELLOW}" "${C_RESET}"
    printf '  %bUse ROS 2 Jazzy for full backend + Unity bridge support.%b\n' "${C_YELLOW}" "${C_RESET}"
    printf '\n'
  fi

  printf '  Completed in %dm %ds\n' "$mins" "$secs"
  printf '  Installed to: %s\n' "$install_root"
  printf '  Full log:     %s\n' "$LOG_FILE"
  printf '\n'
  printf '  %bNext steps:%b\n' "${C_BLUE}" "${C_RESET}"
  printf '\n'
  printf '    1. Open a new shell (or run: %bsource ~/.bashrc%b)\n' "${C_BOLD}" "${C_RESET}"
  printf '    2. Check status:   %bhorus-status%b\n' "${C_BOLD}" "${C_RESET}"
  printf '    3. Start backend:  %bhorus-start%b\n' "${C_BOLD}" "${C_RESET}"
  printf '    4. Run demo:\n'
  printf '       %bhorus-python%b %s/sdk/python/examples/sdk_typical_ops_demo.py \\\n' "${C_BOLD}" "${C_RESET}" "$install_root"
  printf '         --robot-count 10 --workspace-scale 0.1\n'
  printf '    5. Uninstall:      %bhorus-uninstall%b\n' "${C_BOLD}" "${C_RESET}"
  printf '\n'
}
