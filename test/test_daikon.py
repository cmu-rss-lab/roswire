import os

from roswire.bag.daikon import bag_to_decls

from test_bag import load_mavros_description

DIR_TEST = os.path.dirname(__file__)


def test_bag_to_decls():
    system_desc = load_mavros_description()
    fn_bag = os.path.join(DIR_TEST, 'example.bag')
    bag_to_decls(fn_bag, system_desc)
