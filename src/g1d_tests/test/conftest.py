import os

# Isolate the closed-loop tests (and every process they launch) from other test runs.
os.environ["ROS_DOMAIN_ID"] = "54"
