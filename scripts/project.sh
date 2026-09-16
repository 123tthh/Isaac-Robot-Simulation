#!/usr/bin/env bash
# Stable project command; the dated launcher remains for older instructions.
set -euo pipefail
_project_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PROJECT_COMMAND_NAME="${0}"
exec "$_project_script_dir/project_control.sh" "$@"
