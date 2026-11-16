#!/usr/bin/env bash
# Confluence launcher - one command for everything. No sourcing, no folders to cd into.
#
#   confluence start [airport|warehouse] [gazebo|headless|lite]   start (builds first if needed)
#   confluence shutdown                                             stop everything
#   confluence restart                                              shutdown + start same site/mode
#   confluence status                                               what every robot is doing
#   confluence drive                                                keyboard control
#   confluence send r1 gate_3   |  jog r1 north  |  stop r1  |  home r1  ...   (see: confluence help)
#   confluence open | logs | doctor | build
#
# First time: run it from the unzipped folder:   bash confluence.sh start airport
# After that, just type:                        confluence <command>   (from any terminal)

SELF="$(readlink -f "${BASH_SOURCE[0]}")"
SRC="$(dirname "$SELF")"
WS="${CONFLUENCE_WS:-$HOME/confluence_ws}"
PKG="$WS/src/confluence_sim"
STATE_DIR="$HOME/.ros/confluence"
PIDFILE="$STATE_DIR/run.pid"
RUNFILE="$STATE_DIR/run.conf"
LOG="$STATE_DIR/launch.log"
PORT="${CONFLUENCE_PORT:-8080}"
URL="http://localhost:$PORT"
ROS_ROOT_DIR="${CONFLUENCE_ROS_ROOT:-/opt/ros}"
mkdir -p "$STATE_DIR"

bold() { printf '\033[1m%s\033[0m\n' "$*"; }
ok()   { printf '  \033[32m✓\033[0m %s\n' "$*"; }
warn() { printf '  \033[33m!\033[0m %s\n' "$*"; }
err()  { printf '  \033[31m✗\033[0m %s\n' "$*"; }

# ------------------------------------------------------------------ ROS
pick_distro() {
  if [ -n "${ROS_DISTRO:-}" ] && [ -f "$ROS_ROOT_DIR/$ROS_DISTRO/setup.bash" ]; then echo "$ROS_DISTRO"; return; fi
  local want=""
  case "$(. /etc/os-release 2>/dev/null && echo "$VERSION_ID")" in
    26.04) want=lyrical ;; 24.04) want=jazzy ;; 22.04) want=humble ;;
  esac
  if [ -n "$want" ] && [ -f "$ROS_ROOT_DIR/$want/setup.bash" ]; then echo "$want"; return; fi
  local found=( "$ROS_ROOT_DIR"/*/setup.bash )
  if [ -f "${found[0]}" ] && [ ${#found[@]} -eq 1 ]; then basename "$(dirname "${found[0]}")"; return; fi
  [ -f "${found[0]}" ] && basename "$(dirname "${found[-1]}")"
}

source_ros() {
  DISTRO="$(pick_distro)"
  if [ -z "$DISTRO" ]; then
    err "ROS 2 not found in $ROS_ROOT_DIR. Install ROS 2 first, or use:  confluence start airport lite"
    return 1
  fi
  # shellcheck disable=SC1090
  source "$ROS_ROOT_DIR/$DISTRO/setup.bash"
  if [ -f "$WS/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash"
  fi
  return 0
}

src_hash() {
  ( cd "$1" && find . -type f \( -name '*.py' -o -name '*.yaml' -o -name '*.html' -o -name '*.xml' \
      -o -name '*.cfg' -o -name '*.sh' \) -not -path '*/__pycache__/*' -not -path './docs/*' -print0 \
    | sort -z | xargs -0 sha1sum ) | sha1sum | cut -c1-16
}

need_python_deps() {
  local miss=()
  python3 -c "import yaml" 2>/dev/null || miss+=(python3-yaml)
  python3 -c "import paho.mqtt" 2>/dev/null || miss+=(python3-paho-mqtt)
  if [ ${#miss[@]} -gt 0 ]; then
    bold "Installing ${miss[*]} (needs your password)"
    sudo apt-get install -y "${miss[@]}" || { err "could not install ${miss[*]}"; return 1; }
  fi
}

install_cli() {
  chmod +x "$PKG/confluence.sh" "$PKG/run_sim.sh" 2>/dev/null
  mkdir -p "$HOME/.local/bin"
  ln -sf "$PKG/confluence.sh" "$HOME/.local/bin/confluence"
  if ! grep -q 'confluence launcher' "$HOME/.bashrc" 2>/dev/null; then
    printf '\n# confluence launcher\nexport PATH="$HOME/.local/bin:$PATH"\n' >> "$HOME/.bashrc"
  fi
}

do_build() {   # $1 = force
  source_ros || return 1
  need_python_deps || return 1
  local miss=()
  ros2 pkg prefix ros_gz_sim >/dev/null 2>&1 || miss+=("ros-$DISTRO-ros-gz")
  command -v colcon >/dev/null 2>&1 || miss+=(python3-colcon-common-extensions)
  if [ ${#miss[@]} -gt 0 ]; then
    bold "Installing ${miss[*]} (needs your password)"
    sudo apt-get update && sudo apt-get install -y "${miss[@]}" || { err "install failed"; return 1; }
    source "$ROS_ROOT_DIR/$DISTRO/setup.bash"
  fi
  # copy the package into the workspace (unless we are running from there)
  if [ "$SRC" != "$PKG" ]; then
    mkdir -p "$WS/src"
    rm -rf "$PKG"
    cp -r "$SRC" "$PKG"
    find "$PKG" -name __pycache__ -prune -exec rm -rf {} +
  fi
  local h; h="$(src_hash "$PKG")"
  if [ "$1" != "force" ] && [ -f "$WS/.confluence_built" ] && [ "$(cat "$WS/.confluence_built")" = "$h" ] \
     && [ -d "$WS/install/confluence_sim" ]; then
    install_cli
    return 0
  fi
  bold "Building Confluence (ROS 2 $DISTRO) ..."
  rm -rf "$WS/build/confluence_sim" "$WS/install/confluence_sim"
  ( cd "$WS" && colcon build --base-paths "$WS/src" --packages-select confluence_sim 2>&1 | tail -n 15 )
  if [ ! -d "$WS/install/confluence_sim" ]; then
    err "build failed - full log: $WS/log/latest_build/confluence_sim/stdout_stderr.log"
    return 1
  fi
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  local exes; exes="$(ros2 pkg executables confluence_sim 2>/dev/null)"
  for e in confluence vendor_robots confluence_teleop; do
    echo "$exes" | grep -q " $e\$" || { err "build is missing '$e'"; return 1; }
  done
  echo "$h" > "$WS/.confluence_built"
  install_cli
  ok "built"
}

# ------------------------------------------------------------------ processes
our_pids() {   # everything Confluence starts, never this script itself
  pgrep -f -- "lib/confluence_sim/(confluence|vendor_robots)|vendor_robots --backend|confluence\.launch\.py|sim\.launch\.py|\.ros/confluence_(airport|warehouse)\.sdf|fleet_bridge|-m confluence\.lite" \
    | grep -v -x -e "$$" -e "$PPID" || true
}

api_up() {     # 0 if the dashboard answers
  python3 - "$URL" <<'PY' >/dev/null 2>&1
import sys, urllib.request
urllib.request.urlopen(sys.argv[1] + "/api/state", timeout=2).read()
PY
}

robots_online() {
  python3 - "$URL" <<'PY' 2>/dev/null
import sys, json, urllib.request
s = json.load(urllib.request.urlopen(sys.argv[1] + "/api/state", timeout=2))
print(sum(r["state"] != "OFFLINE" for r in s["robots"]))
PY
}

do_shutdown() {
  local quiet="$1"
  if [ -f "$PIDFILE" ]; then
    local pg; pg="$(cat "$PIDFILE")"
    kill -INT -- "-$pg" 2>/dev/null
  fi
  local p; p="$(our_pids)"
  [ -n "$p" ] && kill -INT $p 2>/dev/null
  [ -z "$p" ] && [ ! -f "$PIDFILE" ] && { [ "$quiet" = "quiet" ] || ok "nothing was running"; return 0; }
  for _ in $(seq 1 20); do
    [ -z "$(our_pids)" ] && break
    sleep 0.5
  done
  p="$(our_pids)"; [ -n "$p" ] && kill -TERM $p 2>/dev/null && sleep 1
  p="$(our_pids)"; [ -n "$p" ] && kill -KILL $p 2>/dev/null
  # Gazebo sometimes leaves its server/gui behind
  pkill -f "gz sim.*confluence_" 2>/dev/null
  rm -f "$PIDFILE"
  [ "$quiet" = "quiet" ] || ok "everything stopped"
}

port_owner() {   # prints who listens on a port ("another program" if we can't see the name)
  local line; line="$(ss -ltnpH "sport = :$1" 2>/dev/null | head -1)"
  [ -z "$line" ] && return
  local name; name="$(echo "$line" | sed -n 's/.*users:(("\([^"]*\)".*/\1/p')"
  echo "${name:-another program}"
}

wait_ready() {
  local pg="$1" t=0 max=150
  printf '  waiting for Confluence'
  while [ $t -lt $max ]; do
    if api_up; then
      printf ' up\n'
      break
    fi
    if ! kill -0 "$pg" 2>/dev/null; then
      printf '\n'; err "it stopped while starting. Last lines of $LOG:"
      tail -n 25 "$LOG" | sed 's/^/    /'
      return 1
    fi
    printf '.'; sleep 1; t=$((t + 1))
  done
  if [ $t -ge $max ]; then printf '\n'; err "no answer after ${max}s - see: confluence logs"; return 1; fi
  # wait for robots to report in
  for _ in $(seq 1 40); do
    [ "$(robots_online)" = "4" ] && break
    sleep 1
  done
  return 0
}

do_start() {
  local site="airport" mode="gazebo"
  for a in "$@"; do
    case "$a" in
      airport|warehouse) site="$a" ;;
      gazebo|gui|full) mode="gazebo" ;;
      headless|lite) mode="$a" ;;
      *) err "unknown option '$a' (use airport|warehouse and gazebo|headless|lite)"; return 1 ;;
    esac
  done
  bold "Starting Confluence - $site ($mode)"
  do_shutdown quiet
  for p in "$PORT" 8081 9090; do        # 1883 (MQTT) may be a real broker - that's fine
    local o; o="$(port_owner "$p")"
    if [ -n "$o" ]; then
      err "port $p is used by $o. Close it (or reboot) and try again."; return 1
    fi
  done
  : > "$LOG"
  if [ "$mode" = "lite" ]; then
    need_python_deps || return 1
    PKG_RUN="$SRC"
    ( cd "$PKG_RUN" && exec setsid -w python3 -u -m confluence.lite --site "$site" --port "$PORT" ) >> "$LOG" 2>&1 &
    local pg=$!
    install_cli_from_src
  else
    do_build || return 1
    PKG_RUN="$PKG"
    local hl="false"; [ "$mode" = "headless" ] && hl="true"
    ( exec setsid -w bash -c "source '$ROS_ROOT_DIR/$DISTRO/setup.bash'; source '$WS/install/setup.bash'; \
        exec ros2 launch confluence_sim confluence.launch.py site:=$site headless:=$hl dashboard_port:=$PORT" ) >> "$LOG" 2>&1 &
    local pg=$!
  fi
  echo "$pg" > "$PIDFILE"
  printf 'site=%s\nmode=%s\n' "$site" "$mode" > "$RUNFILE"
  wait_ready "$pg" || return 1
  python3 "$PKG_RUN/confluence/cli.py" status
  bold "Ready. Control it with:"
  echo "    dashboard   $URL   (click a robot, then click where it should go)"
  echo "    keyboard    confluence drive"
  echo "    commands    confluence send r1 gate_3   ·   confluence help"
  echo "    stop all    confluence shutdown"
  if command -v xdg-open >/dev/null 2>&1 && [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && [ -z "${CONFLUENCE_NO_BROWSER:-}" ]; then
    xdg-open "$URL" >/dev/null 2>&1 &
  fi
}

install_cli_from_src() {   # lite mode: point the command at this folder
  chmod +x "$SELF" 2>/dev/null
  mkdir -p "$HOME/.local/bin"
  ln -sf "$SELF" "$HOME/.local/bin/confluence"
  if ! grep -q 'confluence launcher' "$HOME/.bashrc" 2>/dev/null; then
    printf '\n# confluence launcher\nexport PATH="$HOME/.local/bin:$PATH"\n' >> "$HOME/.bashrc"
  fi
}

do_doctor() {
  bold "Confluence doctor"
  local d; d="$(pick_distro)"
  if [ -n "$d" ]; then ok "ROS 2 $d"; else warn "ROS 2 not found (lite mode still works)"; fi
  python3 -c "import yaml" 2>/dev/null && ok "python3-yaml" || err "python3-yaml missing  ->  sudo apt install python3-yaml"
  python3 -c "import paho.mqtt" 2>/dev/null && ok "python3-paho-mqtt" || err "python3-paho-mqtt missing  ->  sudo apt install python3-paho-mqtt"
  if [ -n "$d" ]; then
    ( source "$ROS_ROOT_DIR/$d/setup.bash"; ros2 pkg prefix ros_gz_sim >/dev/null 2>&1 ) \
      && ok "Gazebo bridge (ros_gz)" || err "ros-$d-ros-gz missing  ->  confluence build"
  fi
  if [ -d "$WS/install/confluence_sim" ]; then
    if [ -f "$WS/.confluence_built" ] && [ "$(cat "$WS/.confluence_built")" = "$(src_hash "$PKG")" ]; then
      ok "workspace built and up to date ($WS)"
    else warn "workspace needs a rebuild (happens automatically on start)"; fi
  else warn "not built yet (happens automatically on start)"; fi
  command -v confluence >/dev/null 2>&1 && ok "'confluence' command installed" \
    || warn "'confluence' command not on PATH yet - open a new terminal, or use ~/.local/bin/confluence"
  local p; p="$(our_pids)"
  if [ -n "$p" ]; then ok "running ($(echo "$p" | wc -w) processes) - $(tr '\n' ' ' < "$RUNFILE" 2>/dev/null)"
  else warn "not running"; fi
  for port in "$PORT" 8081 9090 1883; do
    local o; o="$(port_owner "$port")"
    [ -n "$o" ] && echo "    port $port: $o"
  done
  if api_up; then
    ok "dashboard answers at $URL"
    python3 "$SRC/confluence/cli.py" status
  else
    [ -n "$p" ] && err "processes are running but the dashboard doesn't answer - see: confluence logs"
  fi
}

usage() {
  sed -n '2,13p' "$SELF" | sed 's/^# \{0,1\}//'
  echo
  python3 "$SRC/confluence/cli.py" help | sed -n '4,30p'
}

# ------------------------------------------------------------------ main
cmd="${1:-help}"; shift 2>/dev/null
case "$cmd" in
  start)    do_start "$@" ;;
  shutdown|stopall|quit) do_shutdown ;;
  restart)  site=$(sed -n 's/^site=//p' "$RUNFILE" 2>/dev/null); mode=$(sed -n 's/^mode=//p' "$RUNFILE" 2>/dev/null)
            do_start ${site:-airport} ${mode:-gazebo} ;;
  build)    do_build force ;;
  drive)    exec python3 "$SRC/confluence/teleop.py" --url "$URL" ;;
  open)     xdg-open "$URL" >/dev/null 2>&1 || echo "  open $URL in your browser" ;;
  report)   xdg-open "$URL/api/report" >/dev/null 2>&1 || echo "  open $URL/api/report in your browser" ;;
  logs)     echo "  (Ctrl+C to stop watching - Confluence keeps running)"; tail -n 40 -f "$LOG" ;;
  doctor)   do_doctor ;;
  help|-h|--help) usage ;;
  *)        CONFLUENCE_URL="$URL" exec python3 "$SRC/confluence/cli.py" "$cmd" "$@" ;;
esac
