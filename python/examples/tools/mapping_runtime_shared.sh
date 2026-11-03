# shellcheck shell=bash
# Process-local consumer profile; the shared runtime is provisioned separately.
export HORUS_REMOTE_PYTHON="${HORUS_REMOTE_PYTHON:-$HOME/isaac/env_isaacsim/bin/python}"
case "$HORUS_REMOTE_PYTHON" in
    "$HOME/isaac/env_isaacsim_61/"*)
        printf 'The legacy Isaac 6.1 Python is not qualified with the migrated Lab source; select the canonical runtime.\n' >&2
        return 2 ;;
esac
export HORUS_REMOTE_CACHE_ROOT="${HORUS_REMOTE_CACHE_ROOT:-${XDG_CACHE_HOME:-$HOME/.cache}/horus/remote_render/isaacsim-6.2-py312-torch2.11-cu130}"
export HORUS_NVENC_ROOT="${HORUS_NVENC_ROOT:-$HORUS_REMOTE_CACHE_ROOT/nvenc-color-vui-v1}"
export TORCH_EXTENSIONS_DIR="$HORUS_REMOTE_CACHE_ROOT/torch-extensions"
export PYTHONNOUSERSITE=1

# Remove only the superseded HORUS overlays/toolkit, not unrelated ROS paths.
for _horus_variable in PATH PYTHONPATH LD_LIBRARY_PATH; do
    _horus_filtered=""
    IFS=: read -r -a _horus_entries <<< "${!_horus_variable:-}"
    for _horus_entry in "${_horus_entries[@]}"; do
        case "$_horus_entry" in
            "$HOME/isaac/env_isaacsim_61"|"$HOME/isaac/env_isaacsim_61/"*|*/HORUS/diagnostics/mapping-fixes-2026-10-08/gpu-python|*/HORUS/diagnostics/mapping-fixes-2026-10-08/cuda-12.8/bin|"$HOME/.cache/horus/mapping-demo/cuda-12.8/bin"|"") continue ;;
        esac
        if [[ "$_horus_variable" == PYTHONPATH && "$_horus_entry" == "$HORUS_REMOTE_CACHE_ROOT/site-packages" ]]; then
            continue
        fi
        if [[ "$_horus_variable" == PATH && -n "${HORUS_REMOTE_CUDA_HOME:-}" && "$_horus_entry" == "$HORUS_REMOTE_CUDA_HOME/bin" ]]; then
            continue
        fi
        _horus_filtered="${_horus_filtered:+$_horus_filtered:}$_horus_entry"
    done
    if [[ -n "$_horus_filtered" ]]; then
        printf -v "$_horus_variable" '%s' "$_horus_filtered"
        export "${_horus_variable?}"
    else
        unset "$_horus_variable"
    fi
done
export PYTHONPATH="$HORUS_REMOTE_CACHE_ROOT/site-packages${PYTHONPATH:+:$PYTHONPATH}"

for _horus_variable in CUDA_HOME CUDA_PATH; do
    case "${!_horus_variable:-}" in
        */HORUS/diagnostics/mapping-fixes-2026-10-08/cuda-12.8|"$HOME/.cache/horus/mapping-demo/cuda-12.8") unset "$_horus_variable" ;;
    esac
done
# Use only the externally selected toolkit; do not guess another compiler.
if [[ -n "${HORUS_REMOTE_CUDA_HOME:-}" ]]; then
    if [[ ! -x "$HORUS_REMOTE_CUDA_HOME/bin/nvcc" ]]; then
        printf 'HORUS CUDA compiler is unavailable: %s/bin/nvcc\n' "$HORUS_REMOTE_CUDA_HOME" >&2
        return 2
    fi
    export CUDA_HOME="$HORUS_REMOTE_CUDA_HOME"
    export PATH="$CUDA_HOME/bin${PATH:+:$PATH}"
fi
unset _horus_variable _horus_filtered _horus_entries _horus_entry
