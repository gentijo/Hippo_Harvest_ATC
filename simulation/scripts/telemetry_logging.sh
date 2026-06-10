#!/usr/bin/env bash

telemetry_enabled() {
  local configured="${HIPPO_TELEMETRY_ENABLED:-${TELEMETRY_ENABLED:-auto}}"
  case "${configured,,}" in
    0|false|off|no|disabled)
      return 1
      ;;
  esac

  export OTEL_EXPORTER_OTLP_ENDPOINT="${OTEL_EXPORTER_OTLP_ENDPOINT:-http://otelcol:4318}"
  return 0
}

run_with_sim_logging() {
  local log_file="$1"
  shift

  if telemetry_enabled; then
    echo "Telemetry enabled; ROS launch output is written to $log_file and simulator telemetry exports to $OTEL_EXPORTER_OTLP_ENDPOINT"
    "$@" >"$log_file" 2>&1
  else
    "$@" 2>&1 | tee "$log_file"
  fi
}
