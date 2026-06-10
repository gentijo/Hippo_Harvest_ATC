"""Telemetry helpers for ATC nodes and the shared run context channel."""

import json
import logging
import os
import socket
import uuid
from contextlib import nullcontext
from urllib.parse import urlparse

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


RUN_ID_TOPIC = "/telemetry/run_id"
ROBOT_CONTEXT_TOPIC = "/telemetry/robot_trace_contexts"
DEFAULT_OTEL_ENDPOINT = "otelcol:4318"


class _NoopSpan:
    """Fallback span object used when OpenTelemetry is unavailable."""

    def set_attribute(self, name, value) -> None:
        pass

    def add_event(self, name, attributes=None) -> None:
        pass

    def record_exception(self, exception) -> None:
        pass

    def end(self) -> None:
        pass


class Telemetry:
    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------
    def __init__(self, service_name: str, node_name: str, logger=None) -> None:
        self.service_name = service_name
        self.node_name = node_name
        self.logger = logger
        self.enabled = False
        self._tracer = None
        self._otel_logger = logging.getLogger(f"otel.{service_name}.{node_name}")
        self.endpoint = os.environ.get("OTEL_EXPORTER_OTLP_ENDPOINT", DEFAULT_OTEL_ENDPOINT).rstrip("/")
        self.export_endpoint = self._export_endpoint(self.endpoint)

        # Telemetry is optional. If the collector is unreachable, the rest of
        # the system should continue normally with console-only logging.
        if not self._endpoint_available():
            if self.logger is not None:
                self.logger.warning(
                    f"OpenTelemetry endpoint {self.endpoint} is unavailable; telemetry disabled"
                )
            return

        try:
            from opentelemetry import _logs, trace
            from opentelemetry.exporter.otlp.proto.http._log_exporter import OTLPLogExporter
            from opentelemetry.exporter.otlp.proto.http.trace_exporter import OTLPSpanExporter
            from opentelemetry.sdk._logs import LoggerProvider, LoggingHandler
            from opentelemetry.sdk._logs.export import BatchLogRecordProcessor
            from opentelemetry.sdk.resources import Resource
            from opentelemetry.sdk.trace import TracerProvider
            from opentelemetry.sdk.trace.export import BatchSpanProcessor

            # Bind spans and logs to a stable service identity so traces from
            # multiple ATC components are easy to correlate in the backend.
            resource = Resource.create(
                {
                    "service.name": service_name,
                    "service.namespace": "hippo_harvest",
                    "ros.node": node_name,
                }
            )
            trace_provider = TracerProvider(resource=resource)
            trace_provider.add_span_processor(
                BatchSpanProcessor(OTLPSpanExporter(endpoint=f"{self.export_endpoint}/v1/traces"))
            )
            trace.set_tracer_provider(trace_provider)
            self._tracer = trace.get_tracer(service_name)

            logger_provider = LoggerProvider(resource=resource)
            logger_provider.add_log_record_processor(
                BatchLogRecordProcessor(OTLPLogExporter(endpoint=f"{self.export_endpoint}/v1/logs"))
            )
            _logs.set_logger_provider(logger_provider)
            self._otel_logger.addHandler(LoggingHandler(level=logging.INFO, logger_provider=logger_provider))
            self._otel_logger.setLevel(logging.INFO)
            self._otel_logger.propagate = False
            self.enabled = True
        except Exception as exc:
            if self.logger is not None:
                self.logger.warning(f"OpenTelemetry initialization failed; telemetry disabled: {exc}")

    # ------------------------------------------------------------------
    # Span and log entry points
    # ------------------------------------------------------------------
    def start_as_current_span(self, name: str, run_id: str = "", robot_id: str = "", traceparent: str = "", **attrs):
        if not self.enabled:
            return nullcontext(_NoopSpan())

        context = self.extract_context(traceparent)
        attributes = self._attributes(run_id, robot_id, traceparent, attrs)
        return self._tracer.start_as_current_span(name, context=context, attributes=attributes)

    def start_span(self, name: str, run_id: str = "", robot_id: str = "", traceparent: str = "", **attrs):
        if not self.enabled:
            return _NoopSpan()

        context = self.extract_context(traceparent)
        attributes = self._attributes(run_id, robot_id, traceparent, attrs)
        return self._tracer.start_span(name, context=context, attributes=attributes)

    def log(
        self,
        message: str,
        level: str = "info",
        run_id: str = "",
        robot_id: str = "",
        traceparent: str = "",
        **attrs,
    ) -> None:
        extra = self._attributes(run_id, robot_id, traceparent, attrs)
        normalized_level = level.lower()
        if self.enabled:
            log_method = getattr(self._otel_logger, normalized_level, self._otel_logger.info)
            log_method(message, extra=extra)
            return

        if self.logger is None:
            return
        console_message = self._console_message(message, extra)
        log_method = getattr(self.logger, normalized_level, self.logger.info)
        log_method(console_message)

    # ------------------------------------------------------------------
    # Trace-context utilities
    # ------------------------------------------------------------------
    @staticmethod
    def extract_context(traceparent: str):
        if not traceparent:
            return None
        try:
            from opentelemetry import propagate

            return propagate.extract({"traceparent": traceparent})
        except Exception:
            return None

    @staticmethod
    def make_traceparent() -> str:
        return f"00-{uuid.uuid4().hex}-{uuid.uuid4().hex[:16]}-01"

    @staticmethod
    def traceparent_from_span(span) -> str:
        try:
            span_context = span.get_span_context()
        except Exception:
            return ""
        if not span_context or not getattr(span_context, "is_valid", False):
            return ""
        sampled = "01" if int(span_context.trace_flags) & 0x01 else "00"
        return f"00-{span_context.trace_id:032x}-{span_context.span_id:016x}-{sampled}"

    # ------------------------------------------------------------------
    # Attribute rendering
    # ------------------------------------------------------------------
    @staticmethod
    def _attributes(run_id: str, robot_id: str, traceparent: str, attrs: dict) -> dict:
        attributes = {key: value for key, value in attrs.items() if value is not None}
        attributes["run_id"] = run_id
        attributes["traceparent"] = traceparent
        if robot_id:
            attributes["robot.id"] = robot_id
        return attributes

    @staticmethod
    def _console_message(message: str, attributes: dict) -> str:
        rendered_attrs = " ".join(f"{key}={value}" for key, value in sorted(attributes.items()))
        if not rendered_attrs:
            return message
        return f"{message} | {rendered_attrs}"

    # ------------------------------------------------------------------
    # Environment and endpoint plumbing
    # ------------------------------------------------------------------
    def _endpoint_available(self) -> bool:
        parsed = urlparse(self.export_endpoint)
        host = parsed.hostname
        port = parsed.port or (443 if parsed.scheme == "https" else 80)
        if not host:
            return False
        try:
            with socket.create_connection((host, port), timeout=1.0):
                return True
        except OSError:
            return False

    @staticmethod
    def _export_endpoint(endpoint: str) -> str:
        if "://" in endpoint:
            return endpoint
        return f"http://{endpoint}"


class RunContextSubscriber:
    """Track the current run ID and shared robot traceparent map on ROS topics."""

    def __init__(self, node: Node) -> None:
        self.run_id = ""
        self.robot_traceparents: dict[str, str] = {}
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # The run context is retained so late joiners can reconstruct trace
        # chains without waiting for the next ATC event.
        node.create_subscription(String, RUN_ID_TOPIC, self._on_run_id, qos)
        node.create_subscription(String, ROBOT_CONTEXT_TOPIC, self._on_robot_contexts, qos)
        self.robot_context_pub = node.create_publisher(String, ROBOT_CONTEXT_TOPIC, qos)

    def robot_traceparent(self, robot_id: str) -> str:
        return self.robot_traceparents.get(robot_id, "")

    def update_robot_traceparent(self, robot_id: str, traceparent: str) -> None:
        if not robot_id or not traceparent:
            return
        self.robot_traceparents[robot_id] = traceparent
        context_msg = String()
        context_msg.data = json.dumps(self._serialized_contexts(), sort_keys=True)
        self.robot_context_pub.publish(context_msg)

    def _on_run_id(self, msg: String) -> None:
        self.run_id = msg.data.strip()

    def _on_robot_contexts(self, msg: String) -> None:
        try:
            decoded = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        robot_traceparents = {}
        for robot_id, value in decoded.items():
            traceparent = value.get("traceparent") if isinstance(value, dict) else value
            if isinstance(traceparent, str):
                robot_traceparents[str(robot_id)] = traceparent
        self.robot_traceparents = robot_traceparents

    def _serialized_contexts(self) -> dict[str, dict[str, str]]:
        return {
            robot_id: {"traceparent": traceparent}
            for robot_id, traceparent in self.robot_traceparents.items()
        }
