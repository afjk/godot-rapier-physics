#!/usr/bin/env bash
set -euo pipefail

if [[ "$#" -ne 2 ]]; then
  echo "usage: $0 GODOT_BIN OUTPUT_JSON" >&2
  exit 2
fi

godot_bin="$1"
output_json="$2"
work_dir="$(mktemp -d)"
trap 'rm -rf "${work_dir}"' EXIT

(
  cd "${work_dir}"
  "${godot_bin}" --headless --dump-extension-api
)

source_json="${work_dir}/extension_api.json"
version="$(jq -r '[.header.version_major, .header.version_minor, .header.version_patch] | map(tostring) | join(".")' "${source_json}")"
if [[ "${version}" != "4.6.3" ]]; then
  echo "expected Godot extension API 4.6.3, got ${version}" >&2
  exit 1
fi

mode_flag_count="$(jq '[.classes[] | select(.name == "FileAccess") | .methods[] | select(.name == "create_temp") | .arguments[] | select(.name == "mode_flags" and .type == "enum::FileAccess.ModeFlags")] | length' "${source_json}")"
if [[ "${mode_flag_count}" != "1" ]]; then
  echo "unexpected FileAccess.create_temp mode_flags signature" >&2
  exit 1
fi

mkdir -p "$(dirname "${output_json}")"
jq '
  (.classes[]
    | select(.name == "FileAccess")
    | .methods[]
    | select(.name == "create_temp")
    | .arguments[]
    | select(.name == "mode_flags")
    | .type) = "int"
' "${source_json}" > "${output_json}"

jq -e '
  .header.version_major == 4
  and .header.version_minor == 6
  and .header.version_patch == 3
  and any(
    .classes[]
    | select(.name == "FileAccess")
    | .methods[]
    | select(.name == "create_temp")
    | .arguments[];
    .name == "mode_flags" and .type == "int"
  )
' "${output_json}" > /dev/null
