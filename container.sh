#!/usr/bin/env bash
#
# Yahboom ROSMaster container helper (Docker)
#
#    ./container.sh start      create + start the container
#    ./container.sh enter      open a new terminal inside it
#    ./container.sh build      recompile the workspace
#    ./container.sh stop       stop it (keeps compiled artifacts)
#    ./container.sh clean      stop AND delete it (fresh start next time)
#    ./container.sh doctor     diagnose GUI / display problems
#
# Works both on a local session (DISPLAY=:0) and over SSH (ssh -XYC).
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ENGINE="docker"
IMAGE="${IMAGE:-yahboom_rosmaster:latest}"
CONTAINER_NAME="${CONTAINER_NAME:-yahboom_rosmaster}"
CONTAINER_USER="alumno"
WS="/home/${CONTAINER_USER}/rosmaster_ws"
PKG_DIR="${SCRIPT_DIR}/pkg"

# X11 cookie exchange: a host directory bind-mounted into the container.
# A directory (not a file) so that regenerating the cookie is visible inside.
X11_HOST_DIR="/tmp/rosmaster-x11-$(id -u)-${CONTAINER_NAME}"
X11_CONT_DIR="/tmp/.x11host"

# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
die() {
  echo "Error: $*" >&2
  exit 1
}
info() { echo ">> $*"; }

command -v "$ENGINE" >/dev/null 2>&1 || die "$ENGINE is not installed."

container_state() {
  $ENGINE inspect -f '{{.State.Status}}' "$CONTAINER_NAME" 2>/dev/null || echo missing
}

require_running() {
  [ "$(container_state)" = "running" ] ||
    die "Container is not running. Run: $0 start"
}

find_containerfile() {
  local f
  for f in Dockerfile; do
    [ -f "${SCRIPT_DIR}/${f}" ] && {
      echo "${SCRIPT_DIR}/${f}"
      return 0
    }
  done
  return 1
}

build_image() {
  local cfile
  cfile="$(find_containerfile)" ||
    die "No Dockerfile found in ${SCRIPT_DIR}"

  info "Building image '$IMAGE' from $(basename "$cfile")."
  info "This takes a long time the first time (compiles the whole workspace)."
  $ENGINE build -f "$cfile" -t "$IMAGE" "$SCRIPT_DIR"
  info "Image built."
}

ensure_image() {
  $ENGINE image inspect "$IMAGE" >/dev/null 2>&1 && return 0
  info "Image '$IMAGE' not found locally."
  build_image
}

# --------------------------------------------------------------------------
# X11
# --------------------------------------------------------------------------

# Is $DISPLAY a local unix socket (":0", "unix:0") or a TCP one ("localhost:10.0")?
display_is_local() {
  case "${DISPLAY:-}" in
  :* | unix:*) return 0 ;;
  *) return 1 ;;
  esac
}

# Regenerate the cookie for the CURRENT $DISPLAY into the shared directory.
# The family field is rewritten to 'ffff' (FamilyWild) so the entry is accepted
# regardless of the hostname the container reports.
# Returns non-zero if no usable cookie could be produced.
refresh_xauth() {
  local cookie="${X11_HOST_DIR}/Xauthority"

  mkdir -p "$X11_HOST_DIR"
  chmod 755 "$X11_HOST_DIR"

  [ -n "${DISPLAY:-}" ] || return 1

  command -v xauth >/dev/null 2>&1 || {
    info "Warning: 'xauth' is not installed on the host."
    info "         Install it with: sudo apt install x11-xserver-utils"
    return 1
  }

  rm -f "${cookie}.tmp"
  : >"${cookie}.tmp"
  xauth nlist "$DISPLAY" 2>/dev/null |
    sed -e 's/^..../ffff/' |
    xauth -f "${cookie}.tmp" nmerge - 2>/dev/null || true

  mv -f "${cookie}.tmp" "$cookie"
  chmod 644 "$cookie"

  [ -s "$cookie" ]
}

# Build the -e flags handed to every `docker exec`, so each shell picks up the
# display of the SSH session it was launched from.
DISPLAY_ARGS=()
set_display_args() {
  DISPLAY_ARGS=()

  if [ -z "${DISPLAY:-}" ]; then
    info "Warning: DISPLAY is not set; graphical apps will not work."
    info "         Over SSH, reconnect with:  ssh -XYC user@host"
    return 0
  fi

  DISPLAY_ARGS+=(-e "DISPLAY=$DISPLAY")

  if refresh_xauth; then
    DISPLAY_ARGS+=(-e "XAUTHORITY=${X11_CONT_DIR}/Xauthority")
  else
    info "Warning: no X cookie found for DISPLAY=$DISPLAY."
    if display_is_local; then
      info "         On the host run:  xhost +local:"
    else
      info "         Check that the SSH session really forwards X (ssh -XYC)."
    fi
  fi

  # No GPU node available -> avoid rviz2 dying inside GLX.
  if [ ! -e /dev/dri ] || [ -n "${FORCE_SOFTWARE_GL:-}" ]; then
    DISPLAY_ARGS+=(-e "LIBGL_ALWAYS_SOFTWARE=1")
  fi
}

# Safe expansion of a possibly-empty array under `set -u`.
exec_env() {
  set_display_args
  printf '%s\n' ${DISPLAY_ARGS[@]+"${DISPLAY_ARGS[@]}"}
}

# --------------------------------------------------------------------------
# commands
# --------------------------------------------------------------------------
show_help() {
  cat <<EOF
Usage: $0 <command> [args]

Commands:
  start             Create and start the container (idempotent)
  enter             Open a bash shell inside the running container
  run <cmd...>      Run one command inside the container, ROS already sourced
  build             Recompile the workspace with colcon
  stop              Stop the container, keeping its state
  clean             Stop and remove the container
  image             Force a rebuild of the image from the Dockerfile
  status            Show the current state
  doctor            Diagnose display / GUI problems
  help              Show this message

The image is built automatically on the first 'start' if it is missing.

Graphical apps:
  Locally      just run '$0 enter' from a desktop session.
  Remotely     connect with 'ssh -XYC user@host', then '$0 enter'.
               Each new SSH session is picked up automatically; there is no
               need to restart the container after reconnecting.

Your packages:
  Put each package in a folder inside:  ${PKG_DIR}
  It appears inside the container at:   ${WS}/src/pkg
  After adding or editing one, run:     $0 build
EOF
}

start_container() {
  local state
  state="$(container_state)"

  if [ "$state" = "running" ]; then
    info "Container is already running. Use '$0 enter' to open a terminal."
    return 0
  fi

  if [ "$state" = "exited" ] || [ "$state" = "created" ]; then
    info "Starting existing container..."
    if $ENGINE start "$CONTAINER_NAME" >/dev/null 2>&1; then
      info "Ready. Use '$0 enter' to open a terminal."
      return 0
    else
      info "Failed to start existing container. Recreating it..."
      $ENGINE rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
    fi
  fi

  ensure_image
  mkdir -p "$PKG_DIR"
  mkdir -p "$X11_HOST_DIR"
  chmod 755 "$X11_HOST_DIR"

  # Mounts only. DISPLAY and XAUTHORITY are injected per `exec`, not here,
  # because they change with every SSH session.
  local x11_mounts=()
  [ -d /tmp/.X11-unix ] && x11_mounts+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
  x11_mounts+=(-v "${X11_HOST_DIR}:${X11_CONT_DIR}:ro")

  local gpu_args=()

  # NVIDIA GPU support
  if command -v nvidia-smi >/dev/null 2>&1; then
    gpu_args+=(
      --gpus all
      -e NVIDIA_VISIBLE_DEVICES=all
      -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display
    )
    info "NVIDIA GPU detected: enabling CUDA/OpenGL passthrough."
  elif [ -e /dev/dri ]; then
    # Intel/AMD fallback
    gpu_args+=(--device /dev/dri --group-add video)
    info "Using /dev/dri GPU passthrough."
  else
    info "No GPU detected, using software rendering."
  fi

  info "Creating container '$CONTAINER_NAME' (first run may take a while)..."
  $ENGINE run -d \
    --name "$CONTAINER_NAME" \
    --network host \
    --ipc host \
    ${gpu_args[@]+"${gpu_args[@]}"} \
    ${x11_mounts[@]+"${x11_mounts[@]}"} \
    -e "XDG_RUNTIME_DIR=/tmp/runtime-${CONTAINER_USER}" \
    -e "QT_X11_NO_MITSHM=1" \
    -v "${PKG_DIR}:${WS}/src/pkg" \
    -w "$WS" \
    "$IMAGE" sleep infinity >/dev/null

  # XDG_RUNTIME_DIR is referenced above but never created by the image.
  $ENGINE exec -u root "$CONTAINER_NAME" bash -c "
    mkdir -p /tmp/runtime-${CONTAINER_USER}
    chown ${CONTAINER_USER} /tmp/runtime-${CONTAINER_USER}
    chmod 700 /tmp/runtime-${CONTAINER_USER}
  " >/dev/null 2>&1 || true

  info "Ready. Use '$0 enter' to open a terminal."
}

enter_container() {
  require_running
  set_display_args
  $ENGINE exec -it ${DISPLAY_ARGS[@]+"${DISPLAY_ARGS[@]}"} \
    -w "$WS" "$CONTAINER_NAME" bash
}

run_in_container() {
  require_running
  [ $# -gt 0 ] || die "run: no command given."
  set_display_args
  $ENGINE exec -it ${DISPLAY_ARGS[@]+"${DISPLAY_ARGS[@]}"} \
    -w "$WS" "$CONTAINER_NAME" bash -ic "$*"
}

build_workspace() {
  require_running
  info "Building workspace..."
  $ENGINE exec -it -w "$WS" "$CONTAINER_NAME" bash -c '
    source /opt/ros/$ROS_DISTRO/setup.bash
    colcon build --symlink-install
  '
  info "Build finished. Open a NEW terminal with '"'"'$0 enter'"'"' to pick up the changes."
}

stop_container() {
  case "$(container_state)" in
  missing) info "Container does not exist." ;;
  running)
    info "Stopping..."
    $ENGINE stop "$CONTAINER_NAME" >/dev/null
    info "Stopped."
    ;;
  *) info "Container is already stopped." ;;
  esac
}

clean_container() {
  [ "$(container_state)" = "missing" ] && {
    info "Nothing to remove."
    return 0
  }
  echo "This deletes the container. Compiled files inside it are lost."
  echo "Your packages in ${PKG_DIR} are NOT affected."
  read -r -p "Continue? [y/N] " reply
  case "$reply" in
  [Yy]*)
    $ENGINE rm -f "$CONTAINER_NAME" >/dev/null
    rm -rf "$X11_HOST_DIR"
    info "Removed."
    ;;
  *) info "Cancelled." ;;
  esac
}

doctor() {
  echo "Container '$CONTAINER_NAME': $(container_state)"
  echo "DISPLAY                : ${DISPLAY:-<unset>}"
  if [ -n "${DISPLAY:-}" ]; then
    if display_is_local; then
      echo "Display kind           : local unix socket"
    else
      echo "Display kind           : forwarded / TCP (SSH)"
    fi
  fi
  echo "SSH session            : ${SSH_CONNECTION:+yes}${SSH_CONNECTION:-no}"
  echo "xauth on host          : $(command -v xauth >/dev/null 2>&1 && echo yes || echo "NO - install x11-xserver-utils")"
  echo "/tmp/.X11-unix         : $([ -d /tmp/.X11-unix ] && echo present || echo absent)"
  echo "/dev/dri (GPU)         : $([ -e /dev/dri ] && echo present || echo "absent - software GL will be forced")"

  if [ -n "${DISPLAY:-}" ] && command -v xauth >/dev/null 2>&1; then
    local n
    n="$(xauth nlist "$DISPLAY" 2>/dev/null | wc -l)"
    echo "Cookies for \$DISPLAY   : $n"
    [ "$n" -eq 0 ] && echo "  -> no cookie; on a local session try: xhost +local:"
  fi

  if [ "$(container_state)" = "running" ]; then
    echo
    echo "Testing X connection from inside the container..."
    set_display_args
    if $ENGINE exec ${DISPLAY_ARGS[@]+"${DISPLAY_ARGS[@]}"} \
      "$CONTAINER_NAME" bash -c 'command -v xdpyinfo >/dev/null && xdpyinfo >/dev/null 2>&1'; then
      echo "  OK - the container can talk to the X server."
    else
      echo "  FAILED (or xdpyinfo is not installed in the image)."
      echo "  Add 'x11-utils' and 'mesa-utils' to the Dockerfile to test properly."
    fi
  fi
}

# --------------------------------------------------------------------------
main() {
  local cmd="${1:-help}"
  shift || true
  case "$cmd" in
  start) start_container ;;
  enter) enter_container ;;
  run) run_in_container "$@" ;;
  build) build_workspace ;;
  stop) stop_container ;;
  clean) clean_container ;;
  image) build_image ;;
  status) echo "Container '$CONTAINER_NAME': $(container_state)" ;;
  doctor) doctor ;;
  help | -h | --help) show_help ;;
  *)
    echo "Error: unknown command '$cmd'" >&2
    echo
    show_help
    exit 1
    ;;
  esac
}

main "$@"
