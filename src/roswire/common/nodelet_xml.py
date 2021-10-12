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
    class_name: str
        The class name of the main entrypoint for the nodelet
    class_type: str
        The type of the class
    base_name: str
        The name of the base class that the class inherits from
    description: str
        A description of the nodelet
    """

    path: str = attr.ib()
    class_name: str = attr.ib()
    class_type: str = attr.ib()
    base_class: str = attr.ib()
    description: t.Optional[str] = attr.ib()

    @property
    def entrypoint(self):
        return self.class_type + "::OnInit"


@attr.s(frozen=True, auto_attribs=True)
class NodeletInfo:
    libraries: t.Set['NodeletLibrary']

    @classmethod
    def from_nodelet_xml(cls, contents: str) -> 'NodeletInfo':
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
            assert len(class_doms) == 1
            class_dom = class_doms[0]
            assert isinstance(class_dom, dom.Element)
            class_name = class_dom.getAttribute('name')
            class_type = class_dom.getAttribute('type')
            base_class = class_dom.getAttribute('base_class_type')
            description_dom = get_xml_nodes_by_name(
                'description',
                class_dom
            )
            description = None
            if len(description_dom) == 1:
                description = "\n".join(n.data
                                        for n in description_dom[0].childNodes
                                        if n.nodeType == n.TEXT_NODE)
            libraries.add(NodeletLibrary(path=path,
                                         class_name=class_name,
                                         class_type=class_type,
                                         base_class=base_class,
                                         description=description))
        return NodeletInfo(libraries=libraries)


def get_xml_nodes_by_name(tag_name: str, tree: dom.Node) -> t.List[dom.Node]:
    return [n for n in tree.childNodes
            if n.nodeType == n.ELEMENT_NODE and n.tagName == tag_name]
