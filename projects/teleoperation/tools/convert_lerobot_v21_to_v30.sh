#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DATASET="${1:-${ROOT_DIR}/place_tray_middle}"
CONVERTED_PARENT="${2:-${ROOT_DIR}/converted_lerobot}"
REPO_ID="local/place_tray_middle"
DEST_DATASET="${CONVERTED_PARENT}/${REPO_ID}"
LOG_DIR="${ROOT_DIR}/outputs"
LOG_FILE="${LOG_DIR}/lerobot_conversion_error.log"
PYTHON_BIN="${PYTHON_BIN:-python3}"

mkdir -p "${LOG_DIR}" "${CONVERTED_PARENT}/local"
: > "${LOG_FILE}"

if [[ ! -d "${SRC_DATASET}" ]]; then
  echo "source dataset not found: ${SRC_DATASET}" | tee -a "${LOG_FILE}"
  exit 1
fi

if [[ -e "${DEST_DATASET}" ]]; then
  BACKUP="${DEST_DATASET}_$(date +%Y%m%d_%H%M%S)"
  echo "Destination exists; moving old converted copy to ${BACKUP}" | tee -a "${LOG_FILE}"
  mv "${DEST_DATASET}" "${BACKUP}"
fi

echo "Copying ${SRC_DATASET} -> ${DEST_DATASET}"
cp -a "${SRC_DATASET}" "${DEST_DATASET}"

echo "Using Python: $(${PYTHON_BIN} --version 2>&1)"
echo "Inspecting converter help..." | tee -a "${LOG_FILE}"
if ! "${PYTHON_BIN}" -m lerobot.datasets.v30.convert_dataset_v21_to_v30 --help > "${LOG_DIR}/lerobot_converter_help.txt" 2>>"${LOG_FILE}"; then
  echo "Module help failed; trying to locate lerobot package." | tee -a "${LOG_FILE}"
  "${PYTHON_BIN}" - <<'PY' >>"${LOG_FILE}" 2>&1
import pathlib
import lerobot
print(pathlib.Path(lerobot.__file__).parent)
PY
fi

HELP_TEXT="$(cat "${LOG_DIR}/lerobot_converter_help.txt" 2>/dev/null || true)"
CMD=("${PYTHON_BIN}" -m lerobot.datasets.v30.convert_dataset_v21_to_v30 "--repo-id=${REPO_ID}" "--push-to-hub=false" "--force-conversion")
if grep -q -- "--root" <<<"${HELP_TEXT}"; then
  CMD+=("--root=${CONVERTED_PARENT}")
fi

echo "Running converter: ${CMD[*]}"
if ! "${CMD[@]}" >>"${LOG_FILE}" 2>&1; then
  echo "Official conversion failed. Error log: ${LOG_FILE}"
  echo "Leaving copied dataset untouched at: ${DEST_DATASET}"
  exit 2
fi

echo "Inspecting converted dataset..."
"${PYTHON_BIN}" "${ROOT_DIR}/tools/inspect_lerobot_dataset.py" --dataset-root "${DEST_DATASET}" > "${LOG_DIR}/converted_dataset_inspect.txt" 2>>"${LOG_FILE}" || {
  echo "Conversion finished but inspect failed. Error log: ${LOG_FILE}"
  exit 3
}

echo "Conversion completed: ${DEST_DATASET}"
