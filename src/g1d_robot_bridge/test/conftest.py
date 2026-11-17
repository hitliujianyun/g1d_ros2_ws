import os

# Isolate this package's ROS tests from other packages' tests running at the same time.
os.environ["ROS_DOMAIN_ID"] = "51"
