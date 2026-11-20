"""
rapid_parser.py - ABB RAPID Language Parser & Trajectory Simulator (v3)

Features:
  - Full core ABB RAPID syntax: MODULE/PROC, CONST/VAR/PERS robtarget & num,
    MoveJ, MoveL, MoveC, Offs(), WaitTime, Set/Reset/SetDO/PulseDO, TPWrite,
    FOR, WHILE, IF/ELSE, sub-procedures, and RobotStudio controller directives.
  - Real Tool Switching: Reads the tool parameter (e.g. tGripper / tWelder / tool0)
    in MoveJ, MoveL, and MoveC to switch the 3D end-effector dynamically.
  - Real ZoneData Corner Blending: Parses zonedata (fine, z0, z5, z10, z50, z100, z200)
    and applies quadratic Bezier corner rounding between consecutive moves when zone > 0.
"""

import math
import re
from typing import Dict, List, Any, Tuple
from kinematics import (
    HOME_JOINTS,
    KinematicsError,
    forward_kinematics,
    inverse_kinematics,
    interpolate_movej,
    interpolate_movel,
    interpolate_movec,
)

SPEED_TABLE = {
    "v50": 50.0,
    "v100": 100.0,
    "v200": 200.0,
    "v300": 300.0,
    "v500": 500.0,
    "v800": 800.0,
    "v1000": 1000.0,
    "v1500": 1500.0,
    "v2000": 2000.0,
    "vmax": 2000.0,
}

ZONE_TABLE = {
    "fine": 0.0,
    "z0": 0.0,
    "z1": 1.0,
    "z5": 5.0,
    "z10": 10.0,
    "z15": 15.0,
    "z20": 20.0,
    "z30": 30.0,
    "z40": 40.0,
    "z50": 50.0,
    "z60": 60.0,
    "z80": 80.0,
    "z100": 100.0,
    "z150": 150.0,
    "z200": 200.0,
}


class RapidSyntaxError(Exception):
    """Raised when invalid RAPID syntax is encountered."""
    def __init__(self, message: str, line_number: int):
        super().__init__(f"Line {line_number}: {message}")
        self.line_number = line_number
        self.raw_message = message


def _strip_comment(line: str) -> str:
    in_quotes = False
    for idx, ch in enumerate(line):
        if ch == '"':
            in_quotes = not in_quotes
        elif ch == "!" and not in_quotes:
            return line[:idx]
    return line


def _split_top_level_commas(text: str) -> List[str]:
    parts = []
    buf = []
    depth_paren = 0
    depth_brack = 0
    for ch in text:
        if ch == "(":
            depth_paren += 1
        elif ch == ")":
            depth_paren -= 1
        elif ch == "[":
            depth_brack += 1
        elif ch == "]":
            depth_brack -= 1
        elif ch == "," and depth_paren == 0 and depth_brack == 0:
            parts.append("".join(buf).strip())
            buf = []
            continue
        buf.append(ch)
    if buf:
        parts.append("".join(buf).strip())
    return parts


def _eval_num(expr: str, env: Dict[str, float], line_num: int) -> float:
    expr = expr.strip()
    for k, v in env.items():
        expr = re.sub(rf"\b{re.escape(k)}\b", str(v), expr)
    if not re.fullmatch(r"[0-9eE\.\+\-\*/\(\)\s]+", expr):
        raise RapidSyntaxError(f"Invalid numeric expression: '{expr}'", line_num)
    try:
        return float(eval(expr, {"__builtins__": {}}, {}))
    except Exception as exc:
        raise RapidSyntaxError(f"Could not evaluate '{expr}': {exc}", line_num)


def _eval_cond(expr: str, env: Dict[str, float], line_num: int) -> bool:
    cond = expr.strip()
    for k, v in env.items():
        cond = re.sub(rf"\b{re.escape(k)}\b", str(v), cond)
    cond = re.sub(r"(?<![<>!])=(?!=)", "==", cond)
    cond = cond.replace("<>", "!=")
    cond = re.sub(r"\bAND\b", "and", cond, flags=re.IGNORECASE)
    cond = re.sub(r"\bOR\b", "or", cond, flags=re.IGNORECASE)
    cond = re.sub(r"\bNOT\b", "not", cond, flags=re.IGNORECASE)
    cond = re.sub(r"\bTRUE\b", "True", cond, flags=re.IGNORECASE)
    cond = re.sub(r"\bFALSE\b", "False", cond, flags=re.IGNORECASE)

    if not re.fullmatch(r"[0-9eE\.\+\-\*/\(\)\s<>=!a-zA-Z]+", cond):
        raise RapidSyntaxError(f"Invalid condition: '{expr}'", line_num)
    try:
        return bool(eval(cond, {"__builtins__": {}}, {}))
    except Exception as exc:
        raise RapidSyntaxError(f"Could not evaluate condition '{expr}': {exc}", line_num)


def _parse_speed(speed_token: str) -> float:
    token = speed_token.strip().lower()
    if token in SPEED_TABLE:
        return SPEED_TABLE[token]
    m = re.match(r"^v(\d+)$", token)
    if m:
        return float(m.group(1))
    return 500.0


def _parse_zone(zone_token: str) -> Tuple[float, str]:
    """Parse ABB zonedata (fine, z0, z10, z50, z100, ...) into (radius_mm, label)."""
    raw = zone_token.strip()
    token = raw.lower()
    if token in ZONE_TABLE:
        return ZONE_TABLE[token], raw
    m = re.match(r"^z(\d+(?:\.\d+)?)$", token)
    if m:
        return min(250.0, float(m.group(1))), raw
    return 0.0, raw or "fine"


def _parse_tool(tool_token: str) -> Tuple[str, str]:
    """Classify tool token into ('welder' | 'gripper', original_name)."""
    raw = tool_token.strip() if tool_token else "tool0"
    low = raw.lower()
    if any(k in low for k in ("weld", "torch", "arc", "laser", "gun")):
        return "welder", raw
    return "gripper", raw


def _resolve_target(
    target_expr: str,
    targets: Dict[str, List[float]],
    env: Dict[str, float],
    line_num: int,
) -> List[float]:
    expr = target_expr.strip()

    inline_match = re.match(r"^\[\s*\[([^\]]+)\].*\]$", expr)
    if inline_match:
        coords = [c.strip() for c in inline_match.group(1).split(",")]
        if len(coords) != 3:
            raise RapidSyntaxError("Inline robtarget must have 3 coordinates [x, y, z].", line_num)
        return [_eval_num(c, env, line_num) for c in coords]

    offs_match = re.match(r"^Offs\s*\((.+)\)$", expr, re.IGNORECASE)
    if offs_match:
        args = _split_top_level_commas(offs_match.group(1))
        if len(args) != 4:
            raise RapidSyntaxError("Offs() requires 4 arguments: Offs(target, dx, dy, dz)", line_num)
        base_xyz = _resolve_target(args[0], targets, env, line_num)
        dx = _eval_num(args[1], env, line_num)
        dy = _eval_num(args[2], env, line_num)
        dz = _eval_num(args[3], env, line_num)
        return [base_xyz[0] + dx, base_xyz[1] + dy, base_xyz[2] + dz]

    if expr not in targets:
        raise RapidSyntaxError(f"Undefined robtarget '{expr}'", line_num)
    return list(targets[expr])


def _dist3(a: List[float], b: List[float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _point_along(p_from: List[float], p_to: List[float], dist_from_start: float) -> List[float]:
    d = _dist3(p_from, p_to)
    if d < 1e-6:
        return list(p_from)
    ratio = dist_from_start / d
    return [
        p_from[0] + (p_to[0] - p_from[0]) * ratio,
        p_from[1] + (p_to[1] - p_from[1]) * ratio,
        p_from[2] + (p_to[2] - p_from[2]) * ratio,
    ]


def _interpolate_zone_blend(
    q1: List[float],
    corner: List[float],
    q2: List[float],
    speed_mm_s: float,
    fps: int = 30,
) -> List[Dict[str, Any]]:
    """
    Smooth quadratic Bezier corner-blending trajectory from q1 -> q2
    with control point at the original programmed corner.
    """
    arc_approx = _dist3(q1, corner) + _dist3(corner, q2)
    duration = max(0.12, (arc_approx * 0.85) / max(50.0, speed_mm_s))
    steps = max(8, int(duration * fps))

    frames = []
    for i in range(1, steps + 1):
        t = i / steps
        mt = 1.0 - t
        x = mt * mt * q1[0] + 2.0 * mt * t * corner[0] + t * t * q2[0]
        y = mt * mt * q1[1] + 2.0 * mt * t * corner[1] + t * t * q2[1]
        z = mt * mt * q1[2] + 2.0 * mt * t * corner[2] + t * t * q2[2]
        joints = inverse_kinematics(x, y, z)
        frames.append({
            "joints": joints,
            "tcp": [round(x, 2), round(y, 2), round(z, 2)],
        })
    return frames


def simulate_rapid(code: str, fps: int = 30) -> Dict[str, Any]:
    raw_lines = code.splitlines()
    targets: Dict[str, List[float]] = {}
    global_vars: Dict[str, float] = {}
    procs: Dict[str, List[Tuple[int, str]]] = {}

    current_proc = None

    # Pass 1: Extract declarations and PROC blocks
    for idx, raw_line in enumerate(raw_lines, start=1):
        line = _strip_comment(raw_line).strip()
        if not line:
            continue

        if re.match(r"^(MODULE\s+\w+|ENDMODULE)\b", line, re.IGNORECASE):
            continue

        rt_match = re.match(
            r"^(?:CONST|VAR|PERS)\s+robtarget\s+(\w+)\s*:=\s*\[\s*\[([^\]]+)\].*\]\s*;?$",
            line,
            re.IGNORECASE,
        )
        if rt_match:
            name = rt_match.group(1)
            coords = [c.strip() for c in rt_match.group(2).split(",")]
            if len(coords) != 3:
                raise RapidSyntaxError(f"robtarget '{name}' must have 3 coordinates [x, y, z].", idx)
            try:
                targets[name] = [float(coords[0]), float(coords[1]), float(coords[2])]
            except ValueError:
                raise RapidSyntaxError(f"Non-numeric coordinates in robtarget '{name}'.", idx)
            continue

        if current_proc is None:
            num_match = re.match(
                r"^(?:CONST|VAR|PERS)\s+num\s+(\w+)\s*(?::=\s*(.+?))?\s*;$",
                line,
                re.IGNORECASE,
            )
            if num_match:
                vname = num_match.group(1)
                vexpr = num_match.group(2)
                global_vars[vname] = _eval_num(vexpr, global_vars, idx) if vexpr else 0.0
                continue

        if re.match(r"^(?:CONST|VAR|PERS)\s+(?:tooldata|wobjdata|speeddata|zonedata)\b", line, re.IGNORECASE):
            continue

        proc_match = re.match(r"^PROC\s+(\w+)\s*\(\s*\)$", line, re.IGNORECASE)
        if proc_match:
            current_proc = proc_match.group(1)
            procs[current_proc] = []
            continue

        if re.match(r"^ENDPROC$", line, re.IGNORECASE):
            current_proc = None
            continue

        if current_proc is not None:
            procs[current_proc].append((idx, line))
        else:
            procs.setdefault("__top__", []).append((idx, line))

    entry_proc = None
    for name in procs:
        if name.lower() == "main":
            entry_proc = name
            break
    if entry_proc is None:
        if "__top__" in procs:
            entry_proc = "__top__"
        elif procs:
            entry_proc = next(iter(procs))
        else:
            raise RapidSyntaxError("No executable instructions or PROC main() found.", 1)

    # Pass 2: Execute control flow into a flat list of robot actions
    actions: List[Dict[str, Any]] = []
    MAX_ACTIONS = 2000

    def collect_block(statements: List[Tuple[int, str]], env: Dict[str, float], depth: int = 0):
        if depth > 14:
            raise RapidSyntaxError("Maximum nesting/recursion depth exceeded.", statements[0][0])

        i = 0
        while i < len(statements):
            if len(actions) > MAX_ACTIONS:
                raise RapidSyntaxError("Program exceeded maximum instruction count (infinite loop?).", statements[i][0])

            line_num, stmt = statements[i]

            local_num = re.match(
                r"^(?:CONST|VAR|PERS)\s+num\s+(\w+)\s*(?::=\s*(.+?))?\s*;$",
                stmt,
                re.IGNORECASE,
            )
            if local_num:
                vname = local_num.group(1)
                vexpr = local_num.group(2)
                env[vname] = _eval_num(vexpr, env, line_num) if vexpr else 0.0
                i += 1
                continue

            assign_match = re.match(r"^(\w+)\s*:=\s*(.+?);$", stmt)
            if assign_match:
                env[assign_match.group(1)] = _eval_num(assign_match.group(2), env, line_num)
                i += 1
                continue

            for_match = re.match(
                r"^FOR\s+(\w+)\s+FROM\s+(.+?)\s+TO\s+(.+?)\s+DO$",
                stmt,
                re.IGNORECASE,
            )
            if for_match:
                var_name = for_match.group(1)
                start_val = int(round(_eval_num(for_match.group(2), env, line_num)))
                end_val = int(round(_eval_num(for_match.group(3), env, line_num)))
                nest = 1
                j = i + 1
                body = []
                while j < len(statements):
                    ln_j, st_j = statements[j]
                    if re.match(r"^FOR\b", st_j, re.IGNORECASE):
                        nest += 1
                    elif re.match(r"^ENDFOR$", st_j, re.IGNORECASE):
                        nest -= 1
                        if nest == 0:
                            break
                    body.append((ln_j, st_j))
                    j += 1
                if nest != 0:
                    raise RapidSyntaxError("Missing ENDFOR for FOR loop.", line_num)
                step = 1 if end_val >= start_val else -1
                for val in range(start_val, end_val + step, step):
                    env[var_name] = float(val)
                    collect_block(body, env, depth + 1)
                i = j + 1
                continue

            while_match = re.match(r"^WHILE\s+(.+?)\s+DO$", stmt, re.IGNORECASE)
            if while_match:
                cond_expr = while_match.group(1)
                nest = 1
                j = i + 1
                body = []
                while j < len(statements):
                    ln_j, st_j = statements[j]
                    if re.match(r"^WHILE\b", st_j, re.IGNORECASE):
                        nest += 1
                    elif re.match(r"^ENDWHILE$", st_j, re.IGNORECASE):
                        nest -= 1
                        if nest == 0:
                            break
                    body.append((ln_j, st_j))
                    j += 1
                if nest != 0:
                    raise RapidSyntaxError("Missing ENDWHILE for WHILE loop.", line_num)
                guard = 0
                while _eval_cond(cond_expr, env, line_num):
                    guard += 1
                    if guard > 500:
                        raise RapidSyntaxError("WHILE loop exceeded 500 iterations.", line_num)
                    collect_block(body, env, depth + 1)
                i = j + 1
                continue

            if_match = re.match(r"^IF\s+(.+?)\s+THEN$", stmt, re.IGNORECASE)
            if if_match:
                cond_expr = if_match.group(1)
                nest = 1
                j = i + 1
                then_body, else_body = [], []
                in_else = False
                while j < len(statements):
                    ln_j, st_j = statements[j]
                    if re.match(r"^IF\b", st_j, re.IGNORECASE):
                        nest += 1
                    elif re.match(r"^ENDIF$", st_j, re.IGNORECASE):
                        nest -= 1
                        if nest == 0:
                            break
                    elif re.match(r"^ELSE$", st_j, re.IGNORECASE) and nest == 1:
                        in_else = True
                        j += 1
                        continue
                    (else_body if in_else else then_body).append((ln_j, st_j))
                    j += 1
                if nest != 0:
                    raise RapidSyntaxError("Missing ENDIF for IF block.", line_num)
                if _eval_cond(cond_expr, env, line_num):
                    collect_block(then_body, env, depth + 1)
                elif else_body:
                    collect_block(else_body, env, depth + 1)
                i = j + 1
                continue

            # MoveC pVia, pEnd, speed, zone, tool;
            movec_match = re.match(r"^MoveC\s+(.+?);$", stmt, re.IGNORECASE)
            if movec_match:
                parts = _split_top_level_commas(movec_match.group(1))
                if len(parts) < 3:
                    raise RapidSyntaxError(
                        "MoveC requires via-point, end-point, and speed (e.g. MoveC pVia, pEnd, v300, fine, tWelder;).",
                        line_num,
                    )
                via_xyz = _resolve_target(parts[0], targets, env, line_num)
                end_xyz = _resolve_target(parts[1], targets, env, line_num)
                speed = _parse_speed(parts[2])
                zone_mm, zone_str = _parse_zone(parts[3] if len(parts) >= 4 else "fine")
                tool_kind, tool_name = _parse_tool(parts[4] if len(parts) >= 5 else "tool0")
                actions.append({
                    "type": "MOVE",
                    "cmd": "MOVEC",
                    "via": via_xyz,
                    "end": end_xyz,
                    "speed": speed,
                    "zone_mm": zone_mm,
                    "zone_str": zone_str,
                    "tool": tool_kind,
                    "tool_name": tool_name,
                    "line": line_num,
                })
                i += 1
                continue

            # MoveJ / MoveL target, speed, zone, tool;
            move_match = re.match(r"^(MoveJ|MoveL)\s+(.+?);$", stmt, re.IGNORECASE)
            if move_match:
                move_type = move_match.group(1).upper()
                parts = _split_top_level_commas(move_match.group(2))
                if len(parts) < 2:
                    raise RapidSyntaxError(
                        f"{move_type} requires at least target and speed (e.g. MoveJ p1, v500, z10, tool0;).",
                        line_num,
                    )
                target_xyz = _resolve_target(parts[0], targets, env, line_num)
                speed = _parse_speed(parts[1])
                zone_mm, zone_str = _parse_zone(parts[2] if len(parts) >= 3 else "fine")
                tool_kind, tool_name = _parse_tool(parts[3] if len(parts) >= 4 else "tool0")
                actions.append({
                    "type": "MOVE",
                    "cmd": move_type,
                    "via": None,
                    "end": target_xyz,
                    "speed": speed,
                    "zone_mm": zone_mm,
                    "zone_str": zone_str,
                    "tool": tool_kind,
                    "tool_name": tool_name,
                    "line": line_num,
                })
                i += 1
                continue

            wait_match = re.match(r"^WaitTime\s+(.+?);$", stmt, re.IGNORECASE)
            if wait_match:
                secs = max(0.05, min(10.0, _eval_num(wait_match.group(1), env, line_num)))
                actions.append({"type": "WAIT", "secs": secs, "line": line_num})
                i += 1
                continue

            io_match = re.match(r"^(Set|Reset|PulseDO)\s+(\w+)\s*;$", stmt, re.IGNORECASE)
            setdo_match = re.match(r"^SetDO\s+(\w+)\s*,\s*([01])\s*;$", stmt, re.IGNORECASE)
            if io_match or setdo_match:
                if io_match:
                    state = io_match.group(1).lower() in ("set", "pulsedo")
                    sig = io_match.group(2).lower()
                else:
                    sig = setdo_match.group(1).lower()
                    state = setdo_match.group(2) == "1"
                actions.append({"type": "IO", "sig": sig, "state": state, "line": line_num})
                i += 1
                continue

            tp_match = re.match(r'^TPWrite\s+"([^"]*)"\s*;$', stmt, re.IGNORECASE)
            if tp_match:
                actions.append({"type": "LOG", "msg": f"TPWrite: {tp_match.group(1)}", "line": line_num})
                i += 1
                continue

            if re.match(r"^(ConfJ|ConfL|AccSet|VelSet|SingArea|TPErase|Stop)\b", stmt, re.IGNORECASE):
                actions.append({"type": "CTRL", "msg": f"Controller: {stmt}", "line": line_num})
                i += 1
                continue

            call_match = re.match(r"^(\w+)\s*;$", stmt)
            if call_match and call_match.group(1) in procs:
                collect_block(procs[call_match.group(1)], env, depth + 1)
                i += 1
                continue

            raise RapidSyntaxError(f"Unrecognized instruction: '{stmt}'", line_num)

    collect_block(procs[entry_proc], dict(global_vars))

    # Pass 3: Generate trajectory frames with real zonedata corner blending & tool switching
    cur_joints = list(HOME_JOINTS)
    cur_tcp = forward_kinematics(cur_joints)["tcp"]
    gripper_closed = False
    weld_active = False
    cur_tool = "gripper"
    cur_tool_name = "tool0"
    cur_zone_str = "fine"

    frames: List[Dict[str, Any]] = [
        {
            "joints": cur_joints,
            "tcp": cur_tcp,
            "gripper": gripper_closed,
            "weld": weld_active,
            "tool": cur_tool,
            "tool_name": cur_tool_name,
            "zone": cur_zone_str,
            "line": 1,
            "cmd": "INIT",
            "log": None,
        }
    ]

    MAX_FRAMES = 14000
    blended_corners = 0

    for idx_act, act in enumerate(actions):
        if len(frames) > MAX_FRAMES:
            raise RapidSyntaxError("Program exceeded max simulation length.", act["line"])

        atype = act["type"]

        if atype == "MOVE":
            cmd = act["cmd"]
            end_pt = act["end"]
            speed = act["speed"]
            zone_mm = act["zone_mm"]
            cur_zone_str = act["zone_str"]
            line_num = act["line"]

            # Detect tool change and log it
            tool_log = None
            if act["tool"] != cur_tool or act["tool_name"] != cur_tool_name:
                if act["tool"] != cur_tool:
                    tool_log = f"Tool changed -> {act['tool_name']} ({act['tool'].upper()})"
                cur_tool = act["tool"]
                cur_tool_name = act["tool_name"]

            # Check if we should blend the corner into the NEXT move (when zone_mm > 0)
            next_act = actions[idx_act + 1] if idx_act + 1 < len(actions) else None
            can_blend = (
                zone_mm > 0.0
                and cmd in ("MOVEL", "MOVEJ")
                and next_act is not None
                and next_act["type"] == "MOVE"
                and next_act["cmd"] in ("MOVEL", "MOVEJ")
            )

            blend_q1 = None
            blend_q2 = None
            if can_blend:
                len_in = _dist3(cur_tcp, end_pt)
                len_out = _dist3(end_pt, next_act["end"])
                radius = min(zone_mm, 0.45 * len_in, 0.45 * len_out)
                if radius > 2.0:
                    blend_q1 = _point_along(end_pt, cur_tcp, radius)
                    blend_q2 = _point_along(end_pt, next_act["end"], radius)

            target_this_seg = blend_q1 if blend_q1 is not None else end_pt

            try:
                if cmd == "MOVEJ":
                    end_joints = inverse_kinematics(*target_this_seg)
                    seg = interpolate_movej(cur_joints, end_joints, speed, fps=fps)
                elif cmd == "MOVEL":
                    seg = interpolate_movel(cur_tcp, target_this_seg, speed, fps=fps)
                else:  # MOVEC
                    seg = interpolate_movec(cur_tcp, act["via"], end_pt, speed, fps=fps)
            except KinematicsError as ke:
                raise RapidSyntaxError(str(ke), line_num)

            for k, fr in enumerate(seg):
                frames.append({
                    "joints": fr["joints"],
                    "tcp": fr["tcp"],
                    "gripper": gripper_closed,
                    "weld": weld_active,
                    "tool": cur_tool,
                    "tool_name": cur_tool_name,
                    "zone": cur_zone_str,
                    "line": line_num,
                    "cmd": cmd,
                    "log": tool_log if k == 0 else None,
                })
            if seg:
                cur_joints = seg[-1]["joints"]
                cur_tcp = seg[-1]["tcp"]

            # Insert smooth quadratic Bezier corner-blending arc from Q1 -> Q2
            if blend_q1 is not None and blend_q2 is not None:
                try:
                    blend_seg = _interpolate_zone_blend(
                        blend_q1, end_pt, blend_q2, min(speed, next_act["speed"]), fps=fps
                    )
                except KinematicsError as ke:
                    raise RapidSyntaxError(str(ke), line_num)
                blended_corners += 1
                for fr in blend_seg:
                    frames.append({
                        "joints": fr["joints"],
                        "tcp": fr["tcp"],
                        "gripper": gripper_closed,
                        "weld": weld_active,
                        "tool": cur_tool,
                        "tool_name": cur_tool_name,
                        "zone": cur_zone_str,
                        "line": line_num,
                        "cmd": cmd,
                        "log": None,
                    })
                if blend_seg:
                    cur_joints = blend_seg[-1]["joints"]
                    cur_tcp = blend_seg[-1]["tcp"]

        elif atype == "WAIT":
            wait_frames = max(2, int(act["secs"] * fps))
            for _ in range(wait_frames):
                frames.append({
                    "joints": cur_joints,
                    "tcp": cur_tcp,
                    "gripper": gripper_closed,
                    "weld": weld_active,
                    "tool": cur_tool,
                    "tool_name": cur_tool_name,
                    "zone": cur_zone_str,
                    "line": act["line"],
                    "cmd": "WAIT",
                    "log": None,
                })

        elif atype == "IO":
            sig, state = act["sig"], act["state"]
            if "grip" in sig or "clamp" in sig or "suction" in sig:
                gripper_closed = state
                msg = f"I/O: {sig} -> {'CLOSED (1)' if state else 'OPEN (0)'}"
            elif "weld" in sig or "laser" in sig or "arc" in sig:
                weld_active = state
                msg = f"I/O: {sig} -> {'ON (1)' if state else 'OFF (0)'}"
            else:
                msg = f"I/O: {sig} -> {1 if state else 0}"

            for k in range(6):
                frames.append({
                    "joints": cur_joints,
                    "tcp": cur_tcp,
                    "gripper": gripper_closed,
                    "weld": weld_active,
                    "tool": cur_tool,
                    "tool_name": cur_tool_name,
                    "zone": cur_zone_str,
                    "line": act["line"],
                    "cmd": "IO",
                    "log": msg if k == 0 else None,
                })

        elif atype in ("LOG", "CTRL"):
            frames.append({
                "joints": cur_joints,
                "tcp": cur_tcp,
                "gripper": gripper_closed,
                "weld": weld_active,
                "tool": cur_tool,
                "tool_name": cur_tool_name,
                "zone": cur_zone_str,
                "line": act["line"],
                "cmd": atype,
                "log": act["msg"],
            })

    return {
        "frames": frames,
        "targets": targets,
        "fps": fps,
        "summary": (
            f"Compiled successfully: {len(targets)} robtargets, "
            f"{len(frames)} frames ({len(frames)/fps:.1f}s), "
            f"{blended_corners} zoned corners blended."
        ),
    }
