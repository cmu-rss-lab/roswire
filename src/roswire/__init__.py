# -*- coding: utf-8 -*-
from loguru import logger as _logger
_logger.disable('roswire')

from .version import __version__
from .app import App, AppDescription, AppInstance
from .roswire import ROSWire
from . import name

# these will be dropped in 2.1.0
System = AppInstance
SystemDescription = AppDescription
