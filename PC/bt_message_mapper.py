"""
bt_message_mapper.py
----------------------------------------
Handles mapping between:
- RPi internal JSON/state format
- Android Bluetooth plain-text format:
    ROBOT, X, Y, DIRECTION
    OBJECT#, X, Y, DIRECTION
    TARGET, OBSTACLE_NUMBER, TARGET_ID
    WEEK8_TASK_DONE / WEEK9_TASK_DONE
"""

import logging

logger = logging.getLogger(__name__)


class BTMessageMapper:
    """Utility class to map messages between Android and RPi formats"""

    # ==========================================================
    # RPi → Android (send)
    # ==========================================================
    @staticmethod
    def format_robot_position(x: int, y: int, direction: str) -> str:
        if x == -1 and y == -1:
            return "ROBOT, UNKNOWN"
        return f"ROBOT, {x}, {y}, {direction}"

    @staticmethod
    def format_object_position(object_id: int, x: int, y: int, direction: str) -> str:
        if x == -1 and y == -1:
            return f"OBJECT{object_id}, UNKNOWN"
        return f"OBJECT{object_id}, {x}, {y}, {direction}"

    @staticmethod
    def format_target(obstacle_number: int, target_id: int) -> str:
        return f"TARGET, {obstacle_number}, {target_id}"

    @staticmethod
    def format_task_done(week: int) -> str:
        return f"WEEK{week}_TASK_DONE"

    # ==========================================================
    # Android → RPi (receive)
    # ==========================================================
    @staticmethod
    def parse_android_message(msg: str) -> dict:
        """
        Converts Android message → dict
        Returns None if unrecognized.
        """
        try:
            msg = msg.strip()
            if not msg:
                return None

            parts = [p.strip() for p in msg.split(",")]
            cmd = parts[0].upper()

            # --- Robot position ---
            if cmd == "ROBOT":
                if len(parts) == 2 and parts[1].upper() == "UNKNOWN":
                    return {"type": "robot_pos", "x": -1, "y": -1, "direction": "UNKNOWN"}
                elif len(parts) == 4:
                    x, y, direction = int(parts[1]), int(parts[2]), parts[3].upper()
                    return {"type": "robot_pos", "x": x, "y": y, "direction": direction}

            # --- Object position ---
            if cmd.startswith("OBJECT"):
                object_id = int(cmd.replace("OBJECT", ""))
                if len(parts) == 2 and parts[1].upper() == "UNKNOWN":
                    return {"type": "object_pos", "object_id": object_id, "x": -1, "y": -1, "direction": "UNKNOWN"}
                elif len(parts) == 4:
                    x, y, direction = int(parts[1]), int(parts[2]), parts[3].upper()
                    return {"type": "object_pos", "object_id": object_id, "x": x, "y": y, "direction": direction}

            # --- Target mapping ---
            if cmd == "TARGET" and len(parts) == 3:
                obstacle_num, target_id = int(parts[1]), int(parts[2])
                return {"type": "target", "obstacle": obstacle_num, "target_id": target_id}

            # --- Task completion ---
            if "WEEK" in cmd and "TASK_DONE" in cmd:
                return {"type": "task_done", "week": cmd.replace("WEEK", "").replace("_TASK_DONE", "")}

            return None

        except Exception as e:
            logger.warning(f"[BTMapper] Parse failed for '{msg}': {e}")
            return None
