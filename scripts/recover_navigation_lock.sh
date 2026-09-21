#!/usr/bin/env bash
set -euo pipefail

LOCK_FILE="${1:-/tmp/g1_backend_domain_44.lock}"

# A flock is released by the kernel when its owner exits.  Therefore a busy
# lock always means that an older process is still alive; deleting the file
# would only hide it and allow two DDS simulations to run concurrently.
if flock -n "$LOCK_FILE" true; then
  exit 0
fi

if ! command -v fuser >/dev/null 2>&1; then
  echo "El dominio DDS está ocupado y falta 'fuser' para identificar al proceso." >&2
  echo "Instalarlo con: sudo apt install psmisc" >&2
  exit 1
fi

read -r -a OWNERS <<< "$(fuser "$LOCK_FILE" 2>/dev/null || true)"
if (( ${#OWNERS[@]} == 0 )); then
  echo "El dominio DDS está ocupado, pero no se pudo identificar al propietario." >&2
  exit 1
fi

SAFE_OWNERS=()
for pid in "${OWNERS[@]}"; do
  [[ "$pid" =~ ^[0-9]+$ ]] || continue
  command_line="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
  if [[ "$command_line" != *"navigation_sim.py"* ]]; then
    echo "No se cerrará PID $pid: no corresponde a navigation_sim.py ($command_line)" >&2
    exit 1
  fi
  SAFE_OWNERS+=("$pid")
done

if (( ${#SAFE_OWNERS[@]} == 0 )); then
  echo "No se encontró una simulación de navegación segura para cerrar." >&2
  exit 1
fi

echo "Cerrando simulación de navegación anterior (PID: ${SAFE_OWNERS[*]})..."
kill -TERM "${SAFE_OWNERS[@]}" 2>/dev/null || true
for _ in {1..30}; do
  if flock -n "$LOCK_FILE" true; then
    echo "La ejecución anterior se cerró correctamente."
    exit 0
  fi
  sleep 0.1
done

echo "La ejecución anterior no respondió; forzando su cierre..." >&2
kill -KILL "${SAFE_OWNERS[@]}" 2>/dev/null || true
for _ in {1..20}; do
  if flock -n "$LOCK_FILE" true; then
    echo "El dominio DDS quedó liberado."
    exit 0
  fi
  sleep 0.1
done

echo "No se pudo liberar $LOCK_FILE; se cancela el arranque por seguridad." >&2
exit 1
