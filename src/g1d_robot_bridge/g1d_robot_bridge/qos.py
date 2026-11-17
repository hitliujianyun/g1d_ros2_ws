from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# Unitree / SLAMWARE writers: a best-effort reader matches both reliable and best-effort writers.
ROBOT_IN = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
SENSOR_OUT = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                        reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
EVENTS = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                    reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
