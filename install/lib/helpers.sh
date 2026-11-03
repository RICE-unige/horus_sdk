#!/usr/bin/env bash
check_legacy_python_shim() {
  local root="$1"
  if [ -e "$root/bin/python3" ] || [ -L "$root/bin/python3" ]; then
    [ -f "$root/bin/python3" ] && [ ! -L "$root/bin/python3" ] &&
      [ "$(sha256sum "$root/bin/python3" | cut -d' ' -f1)" = 1588115d04121a8118d965acc3ca5332e05be5297fe38df7f73741be6d23fdf6 ] ||
      die "Unrecognized $root/bin/python3; move it outside the helper PATH yourself. It was not changed."
  fi
}

write_helper_scripts() {
  local root="$1" helper
  check_legacy_python_shim "$root"
  mkdir -p "$root/bin" "$root/state" "$root/lib/installer"
  # A previous release installed a global Python shim. Retire only that exact file.
  if [ -f "$root/bin/python3" ]; then
    mv -- "$root/bin/python3" "$root/state/python3-shim.retired-$(date +%s)-$$"
  fi
  install -m 644 "$SCRIPT_DIR/lib/manage.py" "$SCRIPT_DIR/lib/launch_manager.py" "$root/lib/installer/"
  install -m 755 "$SCRIPT_DIR/templates/horus-env" "$root/bin/horus-env"
  for helper in horus-python horus-start horus-stop horus-status horus-update horus-uninstall; do
    install -m 755 "$SCRIPT_DIR/templates/horus-command" "$root/bin/$helper"
  done
}

configure_shell_integration() {
  /usr/bin/python3 "$SCRIPT_DIR/lib/manage.py" shell "$1" "$2"
}

write_install_state() {
  local root="$1"
  /usr/bin/python3 "$SCRIPT_DIR/lib/manage.py" state "$@" \
    "$(git -C "$root/sdk" rev-parse HEAD)" "$(git -C "$root/ros2" rev-parse HEAD)"
}
