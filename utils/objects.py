from enum import Enum
from dataclasses import dataclass

class RobotState(int, Enum):
    STATE_UNKNOWN = 0
    STATE_KILLED = 1
    STATE_MANUAL = 2
    STATE_AUTO = 3

class TaskResult(int, Enum):
    SUCCESS = 0
    CANCELLED = 1
    FAILED = 2
    NOT_STARTED = 3
    RUNNING = 4

class TaskState(int, Enum):
    TASK_1 = 1
    TASK_2 = 2
    TASK_3 = 3
    TASK_UNKNOWN = 4
    TASK_NONE = 5

class Color(str, Enum):
    ORANGE = "ORANGE"
    YELLOW = "YELLOW"
    GREEN = "GREEN"
    RED = "RED"
    BLACK = "BLACK"
    UNKNOWN = "UNKNOWN"

class BuoyRole(str, Enum):
    EDGE = "EDGE"
    OBSTACLE = "OBSTACLE"
    TARGET = "TARGET"
    UNKNOWN = "UNKNOWN"

@dataclass
class TaskObject:
    global_x: float
    global_y: float
    global_z: float
    cam_x: float
    cam_y: float
    cam_z: float
    w: float
    h: float
    dist: float
    confidence:float
    health: float
    last_seen: float
    object_id: int
    is_ghost: bool
    
@dataclass
class Buoy(TaskObject):
    yolo_class_id: int
    color: Color
    role: BuoyRole

@dataclass
class BuoyProfile:
    name: str
    width: float
    height: float
    color: Color
    role: BuoyRole

@dataclass
class WGS84GPS:
    lat: float
    lon: float
    alt: float
    def __iter__(self):
        yield self.lat
        yield self.lon
        yield self.alt
@dataclass
class ENU:
    east: float
    north: float
    up: float
    def __iter__(self):
        yield self.east
        yield self.north
        yield self.up
@dataclass
class ECEF:
    x: float
    y: float
    z: float
    def __iter__(self):
        yield self.x
        yield self.y
        yield self.z