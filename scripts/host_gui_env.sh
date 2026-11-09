#!/usr/bin/env bash
# Source for native GUI programs launched from an IDE installed as a Snap.
# Keep GL/CUDA vendor selection intact: this only removes Snap GUI overrides.
unset GTK_PATH GTK_EXE_PREFIX GTK_MODULES GTK_IM_MODULE_FILE
unset GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR

for G1_GUI_OVERRIDE in GSETTINGS_SCHEMA_DIR GIO_MODULE_DIR GIO_EXTRA_MODULES; do
  if [[ "${!G1_GUI_OVERRIDE:-}" == *"/snap/"* ]]; then
    unset "$G1_GUI_OVERRIDE"
  fi
done
unset G1_GUI_OVERRIDE

if [[ -n "${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-}" ]]; then
  export XDG_DATA_DIRS="$XDG_DATA_DIRS_VSCODE_SNAP_ORIG"
elif [[ "${XDG_DATA_DIRS:-}" == *"/snap/"* ]]; then
  IFS=: read -r -a G1_GUI_DATA_PATHS <<< "$XDG_DATA_DIRS"
  G1_GUI_HOST_PATHS=()
  for G1_GUI_DATA_PATH in "${G1_GUI_DATA_PATHS[@]}"; do
    if [[ -n "$G1_GUI_DATA_PATH" && "$G1_GUI_DATA_PATH" != *"/snap/"* ]]; then
      G1_GUI_HOST_PATHS+=("$G1_GUI_DATA_PATH")
    fi
  done
  XDG_DATA_DIRS="$(IFS=:; echo "${G1_GUI_HOST_PATHS[*]}")"
  export XDG_DATA_DIRS="${XDG_DATA_DIRS:-/usr/local/share:/usr/share}"
  unset G1_GUI_DATA_PATHS G1_GUI_HOST_PATHS G1_GUI_DATA_PATH
fi
