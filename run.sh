#!/usr/bin/env bash
# Public project entry point. Relative arguments resolve from the caller's directory.
set -euo pipefail
_run_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PROJECT_COMMAND_NAME="$0"
if [[ $# -eq 0 ]]; then
  set -- --help
fi
exec "$_run_root/scripts/project_control.sh" "$@"
