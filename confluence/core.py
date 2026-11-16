"""Builds a Confluence instance from a fleet config: adapters + orchestrator +
compliance + dashboard. Used by both the ROS 2 node and the no-ROS lite mode."""
import os

import yaml

from confluence import site
from confluence.adapters import make_adapter
from confluence.compliance import Compliance
from confluence.dashboard.server import Dashboard
from confluence.orchestrator import Orchestrator

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "config")


def default_config(site_name=None):
    return os.path.join(CONFIG_DIR, f"fleet_{site_name or site.NAME}.yaml")


DEFAULT_CONFIG = "auto"   # "auto" = fleet_<active site>.yaml (kept non-empty for ROS 2 parameters)


def load_config(path=None):
    """Load a fleet config. The config's `site:` selects the site map."""
    if not path or path == "auto":
        path = default_config()
    with open(path) as f:
        cfg = yaml.safe_load(f)
    if cfg.get("site") and cfg["site"] != site.NAME:
        site.load(cfg["site"])
    return cfg


class Confluence:
    def __init__(self, cfg, ros_bus=None, dashboard_port=None, log_dir=None):
        self.cfg = cfg
        log_dir = os.path.expanduser(log_dir or cfg.get("log_dir", "~/.ros/confluence"))
        self.compliance = Compliance(log_dir)
        robot_cfgs = {r["id"]: r for r in cfg["robots"]}
        adapters = [make_adapter(r, ros_bus) for r in cfg["robots"]]
        self.orch = Orchestrator(adapters, robot_cfgs, self.compliance,
                                 auto_tasks=cfg.get("auto_tasks", True),
                                 task_interval=cfg.get("task_interval_s", 10))
        port = dashboard_port if dashboard_port is not None else cfg.get("dashboard_port", 8080)
        self.dashboard = Dashboard(self.orch, self.compliance, port) if port else None

    def start(self):
        self.orch.start()
        if self.dashboard:
            self.dashboard.start()
        return self

    def stop(self):
        self.orch.stop()
        self.compliance.report(self.orch.snapshot())
