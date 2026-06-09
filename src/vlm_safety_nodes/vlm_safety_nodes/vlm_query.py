#!/usr/bin/env python3
import os
import json
import time
import math
import re
from pathlib import Path
from typing import Optional, Dict, Any, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from vlm_safety_msgs.msg import VLMQuery, VLMResponse

# Google GenAI SDK
from google import genai
from google.genai import types

# Optional: only used if we need to re-encode to jpeg/png
try:
    import cv2
    import numpy as np
except Exception:
    cv2 = None
    np = None


# ---------- Your prompt (exactly as you wrote, injected via f-string) ----------
def build_vlm_prompt(
    class_name: str,
    track_id: int,
    distance_m: float,
    risk_band: int,
    robot_velocity_scale: float,
    timestamp: str,
) -> str:
    return f"""
You are a robot safety reasoning assistant for a mobile robot operating in a construction site.

You are given:
- ONE camera image captured at the moment a hazard was detected
- Structured hazard metadata from the robot perception system

Your role is to visually validate the hazard, assess the risk, and recommend the safest action.
Always be conservative. If unsure, choose the safer option.

=====================
INPUT METADATA
=====================
- class_name: {class_name}
- track_id: {track_id}
- distance_m: {distance_m}
- risk_band: {risk_band}
  (0=ignore, 1=monitor, 2=high, 3=critical)
- robot_velocity_scale: {robot_velocity_scale}
  (0.0=stopped, 0.5=slowed, 1.0=normal)
- timestamp: {timestamp}

=====================
RISK BAND LOGIC
=====================
The robot has already applied a distance-based safety layer:

- Band 3 (critical):
  Object ≤ 1.0 m or extreme danger
  → robot must stop immediately

- Band 2 (high):
  Object within 1.0–2.5 m or dangerous object within 5 m
  → robot is slowed and monitoring

- Band 1 (monitor):
  Object within 2.5–5.0 m
  → robot continues cautiously

- Band 0 (ignore):
  Object beyond 5.0 m
  → no action required

Do NOT contradict this safety layer.
You may only recommend actions that are equal or more conservative.

=====================
TASKS
=====================
1. Verify visually whether the object is clearly visible and matches the reported class
   (human / vehicle / machinery / staircase).
2. Assess whether the object appears active, moving, or operating.
3. Validate whether the assigned risk_band is reasonable based on the image.
4. Recommend the safest immediate action:
   - continue
   - slow_down
   - stop_and_wait
   - divert_path
5. If divert_path is recommended, explain how Nav2 should handle it
   (e.g., global replan, costmap inflation).

=====================
NAV2 ACTION GUIDELINES
=====================
- stop_and_wait:
  Keep velocity scale at 0.0 until hazard clears.
- slow_down:
  Maintain reduced speed and monitor hazard.
- divert_path:
  Recommend global replanning (Least Preferred), temporary obstacle marking,
  or increased costmap inflation.
- continue:
  Only if clearly safe.

=====================
OUTPUT FORMAT (JSON ONLY)
=====================
Return valid JSON only. Do not include extra text.

{{
  "hazard_confirmed": true,
  "visual_assessment": "brief description of what is seen",
  "risk_validation": "risk band is appropriate | too high | too low",
  "recommended_action": "continue | slow_down | stop_and_wait | divert_path",
  "nav2_guidance": "none | wait | replan | increase_costmap_inflation",
  "reasoning": "short safety-focused explanation",
  "notes": "optional additional caution or observation"
}}

=====================
FINAL RULES
=====================
- Be conservative.
- Do not hallucinate objects not visible.
- Do not contradict the risk band safety layer.
- Output JSON only.
""".strip()


# ---------- Structured output schema (Gemini JSON Schema mode) ----------
# Docs: response_mime_type="application/json" + response_json_schema=... :contentReference[oaicite:4]{index=4}
VLM_JSON_SCHEMA: Dict[str, Any] = {
    "type": "object",
    "properties": {
        "hazard_confirmed": {"type": "boolean"},
        "visual_assessment": {"type": "string"},
        "risk_validation": {"type": "string", "enum": ["risk band is appropriate", "too high", "too low"]},
        "recommended_action": {"type": "string", "enum": ["continue", "slow_down", "stop_and_wait", "divert_path"]},
        "nav2_guidance": {"type": "string", "enum": ["none", "wait", "replan", "increase_costmap_inflation"]},
        "reasoning": {"type": "string"},
        "notes": {"type": "string"},
    },
    "required": [
        "hazard_confirmed",
        "visual_assessment",
        "risk_validation",
        "recommended_action",
        "nav2_guidance",
        "reasoning",
        "notes",
    ],
    "additionalProperties": False,
}


def _stamp_to_str(stamp) -> str:
    return f"{int(stamp.sec)}.{int(stamp.nanosec):09d}"


def _guess_mime_from_compressed_format(fmt: str) -> str:
    s = (fmt or "").lower()
    if "png" in s:
        return "image/png"
    if "jpg" in s or "jpeg" in s:
        return "image/jpeg"
    return "image/jpeg"


def _ensure_jpeg_or_png_bytes(msg_image) -> Tuple[bytes, str]:
    raw = bytes(msg_image.data)
    mime = _guess_mime_from_compressed_format(getattr(msg_image, "format", ""))

    if mime in ("image/jpeg", "image/png"):
        return raw, mime

    # If format is weird, try decode+re-encode to jpeg (optional dependency)
    if cv2 is None or np is None:
        return raw, "image/jpeg"

    arr = np.frombuffer(raw, dtype=np.uint8)
    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if bgr is None:
        return raw, "image/jpeg"

    ok, enc = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if not ok:
        return raw, "image/jpeg"
    return enc.tobytes(), "image/jpeg"


def _action_rank(a: str) -> int:
    a = (a or "").strip().lower()
    if a == "continue":
        return 0
    if a == "slow_down":
        return 1
    if a == "divert_path":
        return 2
    if a == "stop_and_wait":
        return 3
    return 0


def _min_allowed_action_from_band(band: int) -> str:
    if band >= 3:
        return "stop_and_wait"
    if band == 2:
        return "slow_down"
    return "continue"


def _enforce_conservative_action(risk_band: int, recommended_action: str) -> str:
    min_action = _min_allowed_action_from_band(risk_band)
    if _action_rank(recommended_action) < _action_rank(min_action):
        return min_action
    return (recommended_action or min_action).strip().lower()


def _parse_json(text: str) -> Optional[Dict[str, Any]]:
    if not text:
        return None
    text = text.strip()
    try:
        return json.loads(text)
    except Exception:
        pass
    m = re.search(r"\{.*\}", text, flags=re.DOTALL)
    if not m:
        return None
    try:
        return json.loads(m.group(0))
    except Exception:
        return None


class VLMQueryNode(Node):
    def __init__(self):
        super().__init__("vlm_query")

        # Topics
        self.declare_parameter("query_topic", "/vlm/query")
        self.declare_parameter("velocity_topic", "/velocity_scale")
        self.declare_parameter("output_topic", "/vlm/output")

        # Model
        self.declare_parameter("gemini_model", "gemini-2.5-flash-lite")

        # Structured output knobs
        self.declare_parameter("temperature", 0.2)
        self.declare_parameter("max_output_tokens", 512)

        # Reliability
        self.declare_parameter("timeout_s", 8.0)   # used indirectly (SDK uses http under ho
        self.declare_parameter("max_retries", 2)

        # Save-image option
        self.declare_parameter("save_sent_images", True)
        self.declare_parameter("save_dir", "/home/aryan/Research/research_ws2/images_vlm/test1")

        self.query_topic = self.get_parameter("query_topic").value
        self.velocity_topic = self.get_parameter("velocity_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.model = self.get_parameter("gemini_model").value

        self.temperature = float(self.get_parameter("temperature").value)
        self.max_output_tokens = int(self.get_parameter("max_output_tokens").value)

        self.max_retries = int(self.get_parameter("max_retries").value)

        self.save_sent_images = bool(self.get_parameter("save_sent_images").value)
        self.save_dir = Path(self.get_parameter("save_dir").value)
        if self.save_sent_images:
            self.save_dir.mkdir(parents=True, exist_ok=True)

        # Cache latest velocity scale
        self.latest_velocity_scale = 1.0
        self.create_subscription(Float32, self.velocity_topic, self._on_scale, 10)

        # Pub/Sub
        self.create_subscription(VLMQuery, self.query_topic, self._on_query, 10)
        self.pub_out = self.create_publisher(VLMResponse, self.output_topic, 10)

        # GenAI client (uses GEMINI_API_KEY env by default) :contentReference[oaicite:5]{index=5}
        self.client = genai.Client()

        self.get_logger().info(
            f"Node D running. Sub: {self.query_topic} | Pub: {self.output_topic} | model={self.model} | save_images={self.save_sent_images}"
        )

    def _on_scale(self, msg: Float32):
        self.latest_velocity_scale = float(msg.data)

    def _save_image(self, image_bytes: bytes, mime: str, stamp_str: str, track_id: int, cls: str) -> str:
        if not self.save_sent_images:
            return ""

        ext = ".png" if mime == "image/png" else ".jpg"
        safe_cls = "".join([c for c in cls.lower() if c.isalnum() or c in ("-", "_")])[:32]
        filename = f"{stamp_str}_tid{track_id}_{safe_cls}{ext}"
        path = self.save_dir / filename
        try:
            path.write_bytes(image_bytes)
            return str(path)
        except Exception as e:
            self.get_logger().warn(f"Failed to save image to {path}: {e}")
            return ""

    def _fallback(self, q: VLMQuery, err: str, latency_ms: float) -> VLMResponse:
        out = VLMResponse()
        out.header = q.header
        out.class_name = q.class_name
        out.track_id = int(q.track_id)
        out.distance_m = float(q.distance_m)
        out.risk_band = int(q.risk_band)
        out.robot_velocity_scale = float(self.latest_velocity_scale)

        out.ok = False
        out.model_name = self.model
        out.latency_ms = float(latency_ms)
        out.error_msg = err
        out.raw_json = ""

        # Conservative action consistent with safety layer
        out.hazard_confirmed = False
        out.visual_assessment = "VLM unavailable (fallback)."
        out.risk_validation = "risk band is appropriate"
        out.recommended_action = _min_allowed_action_from_band(int(q.risk_band))
        out.nav2_guidance = "wait" if out.recommended_action == "stop_and_wait" else "none"
        out.reasoning = f"Fallback behavior: {err}"
        out.notes = "Conservative output to avoid contradicting risk band safety layer."
        out.saved_image_path = ""
        return out

    def _on_query(self, q: VLMQuery):
        t0 = time.time()

        stamp_str = _stamp_to_str(q.header.stamp)
        cls = q.class_name
        track_id = int(q.track_id)
        distance_m = float(q.distance_m)
        risk_band = int(q.risk_band)
        vel_scale = float(self.latest_velocity_scale)

        # Ensure we have an image
        if q.image is None or len(q.image.data) == 0:
            out = self._fallback(q, "No image in VLMQuery.", (time.time() - t0) * 1000.0)
            self.pub_out.publish(out)
            return

        image_bytes, mime = _ensure_jpeg_or_png_bytes(q.image)

        # Save the exact bytes being sent (your new requirement)
        saved_path = self._save_image(image_bytes, mime, stamp_str, track_id, cls)

        prompt = build_vlm_prompt(
            class_name=cls,
            track_id=track_id,
            distance_m=distance_m,
            risk_band=risk_band,
            robot_velocity_scale=vel_scale,
            timestamp=stamp_str,
        )

        # Build SDK "Part" from bytes (official vision path) :contentReference[oaicite:6]{index=6}
        img_part = types.Part.from_bytes(data=image_bytes, mime_type=mime)

        # Call Gemini with structured output config :contentReference[oaicite:7]{index=7}
        last_err = None
        resp_text = ""
        for attempt in range(self.max_retries + 1):
            try:
                resp = self.client.models.generate_content(
                    model=self.model,
                    contents=[img_part, prompt],
                    config={
                        "response_mime_type": "application/json",
                        "response_json_schema": VLM_JSON_SCHEMA,
                        "temperature": self.temperature,
                        "max_output_tokens": self.max_output_tokens,
                    },
                )
                resp_text = getattr(resp, "text", "") or ""
                last_err = None
                break
            except Exception as e:
                last_err = e
                time.sleep(0.25 * (attempt + 1))

        latency_ms = (time.time() - t0) * 1000.0

        if last_err is not None:
            out = self._fallback(q, f"Gemini call failed: {repr(last_err)}", latency_ms)
            out.saved_image_path = saved_path
            self.pub_out.publish(out)
            return

        parsed = _parse_json(resp_text)
        if parsed is None:
            out = self._fallback(q, "Could not parse JSON from model response.", latency_ms)
            out.raw_json = resp_text[:4000]
            out.saved_image_path = saved_path
            self.pub_out.publish(out)
            return

        # Enforce conservative action (never less conservative than risk_band)
        rec = (parsed.get("recommended_action") or "").strip().lower()
        clamped = _enforce_conservative_action(risk_band, rec)
        parsed["recommended_action"] = clamped
        if rec and rec != clamped:
            parsed["notes"] = (parsed.get("notes") or "")
            parsed["notes"] = (parsed["notes"] + " | " if parsed["notes"] else "") + \
                              "Action clamped to not contradict risk band safety layer."

        # Fill VLMResponse
        out = VLMResponse()
        out.header = q.header
        out.class_name = cls
        out.track_id = track_id
        out.distance_m = distance_m
        out.risk_band = risk_band
        out.robot_velocity_scale = vel_scale

        out.ok = True
        out.model_name = self.model
        out.latency_ms = float(latency_ms)
        out.saved_image_path = saved_path
        out.raw_json = json.dumps(parsed, ensure_ascii=False)
        out.error_msg = ""

        out.hazard_confirmed = bool(parsed.get("hazard_confirmed", False))
        out.visual_assessment = str(parsed.get("visual_assessment", ""))
        out.risk_validation = str(parsed.get("risk_validation", ""))
        out.recommended_action = str(parsed.get("recommended_action", "continue"))
        out.nav2_guidance = str(parsed.get("nav2_guidance", "none"))
        out.reasoning = str(parsed.get("reasoning", ""))
        out.notes = str(parsed.get("notes", ""))

        self.pub_out.publish(out)


def main():
    rclpy.init()
    node = VLMQueryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
