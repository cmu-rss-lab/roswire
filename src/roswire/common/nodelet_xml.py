# -*- coding: utf-8 -*-
import typing as t
import xml.dom.minidom as dom

import attr
from loguru import logger


@attr.s(frozen=True, auto_attribs=True)
class NodeletLibrary:
    """
    This data class represents a piece of information found in the
    nodelet_plygin.xml file:

    path: str
        The path to the library containing the nodelet
    nodelet_name: str
        The class name of the main entrypoint for the nodelet
    class_name: str
        The type of the class
    """

    path: str = attr.ib()
    name: str = attr.ib()
    type_: str = attr.ib()

    @property
    def entrypoint(self) -> str:
        return self.type_ + "::onInit"


@attr.s(frozen=True, auto_attribs=True)
class NodeletsInfo:
    libraries: t.Set['NodeletLibrary']

    @classmethod
    def from_nodelet_xml(cls, contents: str) -> 'NodeletsInfo':
        libraries: t.Set['NodeletLibrary'] = set()
        contents = "<root>\n" + contents + "\n</root>"
        tree = dom.parseString(contents)
        root = get_xml_nodes_by_name('root', tree)[0]
        libraries_dom = get_xml_nodes_by_name('library', root)
        if not libraries_dom:
            logger.warning(f"Expected there to be <library/> elements in nodelet_plugins.xml, but there are none.")
            logger.debug(contents)
        for library_dom in libraries_dom:
            assert isinstance(library_dom, dom.Element)
            path = library_dom.getAttribute('path')
            class_doms = get_xml_nodes_by_name(
                'class',
                library_dom
            )
            for class_dom in class_doms:
                assert isinstance(class_dom, dom.Element)
                name = class_dom.getAttribute('name')
                type_ = class_dom.getAttribute('type')
                libraries.add(NodeletLibrary(path=path,
                                             name=name,
                                             type_=type_,
                                             ))
        return NodeletsInfo(libraries=libraries)


def get_xml_nodes_by_name(tag_name: str, tree: dom.Node) -> t.List[dom.Node]:
    return [n for n in tree.childNodes
            if n.nodeType == n.ELEMENT_NODE and n.tagName == tag_name]
