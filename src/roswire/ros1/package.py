# -*- coding: utf-8 -*-
from typing import List, Optional

from ..common import PackageDatabase


class ROS1PackageDatabase(PackageDatabase):

    @classmethod
    def _determine_paths(cls, app_instance: 'AppInstance') -> List[str]:
