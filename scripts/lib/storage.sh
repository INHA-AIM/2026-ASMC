#!/usr/bin/env bash
# Optional local storage integration. No drive or personal dataset is required.

asmc_storage_load() {
  local repo_root="$1" storage_env explicit=false
  local old_data="${ASMC_DATA-}" old_data_set="${ASMC_DATA+x}"
  local old_repo="${ASMC-}" old_repo_set="${ASMC+x}"
  local old_twinlite="${TWINLITE_SOURCE-}" old_twinlite_set="${TWINLITE_SOURCE+x}"
  if [[ -v ASMC_STORAGE_ENV ]]; then
    explicit=true
    storage_env="$ASMC_STORAGE_ENV"
  else
    storage_env="$(dirname "$repo_root")/.storage/env.sh"
  fi
  if [[ -n "$storage_env" && -r "$storage_env" ]]; then
    # Trusted, machine-local shell configuration; never downloaded by this helper.
    source "$storage_env"
  elif [[ "$explicit" == true && -n "$storage_env" ]]; then
    echo "[FAIL] ASMC_STORAGE_ENV is not readable: $storage_env" >&2
    return 1
  fi
  # An explicitly exported caller path wins over a profile's default value.
  if [[ -n "$old_data_set" ]]; then export ASMC_DATA="$old_data"; fi
  if [[ -n "$old_repo_set" ]]; then export ASMC="$old_repo"; fi
  if [[ -n "$old_twinlite_set" ]]; then export TWINLITE_SOURCE="$old_twinlite"; fi
  export ASMC="${ASMC:-$repo_root}"
  export ASMC_DATA="${ASMC_DATA:-$(dirname "$repo_root")/data}"
  export TWINLITE_SOURCE="${TWINLITE_SOURCE:-$(dirname "$repo_root")/external/baselines/TwinLiteNetPlus}"
  # Compose otherwise interprets a relative volume path against its YAML,
  # whereas mkdir interprets it against the caller's working directory.
  export ASMC_DATA="$(readlink -m -- "$ASMC_DATA")"
  export TWINLITE_SOURCE="$(readlink -m -- "$TWINLITE_SOURCE")"
}

asmc_storage_prepare_data() {
  local repo_root="$1"
  if [[ -n "${ASMC_STORAGE_GUARD:-}" ]]; then
    if [[ -z "${ASMC_STORAGE_WORKSPACE:-}" || ! -r "$ASMC_STORAGE_GUARD" ]]; then
      echo "[FAIL] Local storage guard/workspace configuration is incomplete" >&2
      return 1
    fi
    python3 "$ASMC_STORAGE_GUARD" --workspace "$ASMC_STORAGE_WORKSPACE" check >/dev/null
  fi
  # No policy => portable no-op. Managed mixed data views are checked using the
  # collection category's resolved volume, not the workspace map directory.
  PYTHONPATH="$repo_root/src/data_collection/src${PYTHONPATH:+:$PYTHONPATH}" \
    python3 -c 'import sys; from data_collection.storage_paths import validate_output; validate_output(sys.argv[1], "datasets")' "$ASMC_DATA"
  mkdir -p "$ASMC_DATA"
}

asmc_storage_require_overlay() {
  if [[ -n "$1" && ! -r "$1" ]]; then
    echo "[FAIL] Configured storage compose overlay is not readable: $1" >&2
    return 1
  fi
}
