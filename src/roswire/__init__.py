# -*- coding: utf-8 -*-
from loguru import logger as _logger
_logger.disable('roswire')

from . import name
from .app import App, AppDescription, AppInstance
from .distribution import ROSDistribution, ROSVersion
from .interface import Node, NodeManager
from .roswire import ROSWire
from .version import __version__

# these will be dropped in 2.1.0
System = AppInstance
SystemDescription = AppDescription
