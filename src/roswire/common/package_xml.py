# -*- coding: utf-8 -*-

# Largely copied from https://github.com/ros-infrastructure/catkin_pkg/blob/master/src/catkin_pkg/package.py
import typing as t
from copy import deepcopy

import attr

from ..util import safer_xml_from_string


@attr.s(slots=True, auto_attribs=True)
class Export:
    tagName: str
    attributes: t.Dict[str, t.Any]
    content: str
    evaluated_condition: t.Optional[bool]

    def evaluate_condition(self, context):
        """
        Evaluate the condition.
        The result is also stored in the member variable `evaluated_condition`.
        :param context: A dictionary with key value pairs to replace variables
          starting with $ in the condition.
        :returns: True if the condition evaluates to True, else False
        :raises: :exc:`ValueError` if the condition fails to parse
        """
        evaluated_condition = evaluate_condition(self.attributes.get('condition'), context)
        self.__setattr__('evaluated_condition', evaluated_condition)
        return evaluated_condition


class License(str):

    def __new__(cls, value, file_=None):
        obj = str.__new__(cls, str(value))
        obj.file = file_
        return obj


@attr.s(slots=True, auto_attribs=True)
class Person:
    name: str
    email: t.Optional[str]


@attr.s(slots=True, auto_attribs=True)
class Url:
    url: str
    type: t.Optional[str]


@attr.s(slots=True, auto_attribs=True)
class GroupDependency:
    name: str
    condition: t.Any
    evaluated_condition: t.Optional[bool] = attr.ib(default=None)
    members: t.Set[str] = attr.ib(factory=set)

    def evaluate_condition(self, context):
        """
        Evaluate the condition.
        The result is also stored in the member variable `evaluated_condition`.
        :param context: A dictionary with key value pairs to replace variables
          starting with $ in the condition.
        :returns: True if the condition evaluates to True, else False
        :raises: :exc:`ValueError` if the condition fails to parse
        """
        self.evaluated_condition = evaluate_condition(self.condition, context)
        return self.evaluated_condition

    def extract_group_members(self, packages):
        members: t.Set[str] = set()
        for pkg in packages:
            for g in pkg.member_of_groups:
                assert g.evaluated_condition is not None
            if self.name in (g.name for g in pkg.member_of_groups if g.evaluated_condition):
                members.add(pkg.name)
        self.members = members

class InvalidPackage(Exception):

    def __init__(self, msg, package_path=None):
        self.msg = msg
        self.package_path = package_path
        Exception.__init__(self, self.msg)

    def __str__(self):
        result = '' if not self.package_path else "Error(s) in package '%s':\n" % self.package_path
        return result + Exception.__str__(self)


def parse_pkg_string(data: str,
                         filename: t.Optional[str] = None,
                         warnings: t.Optional[t.List[str]] = None) -> PackageDefinition:
    """ Parse package.xml string contents.

    Parameters
    ----------
    data: str
        The contents of the package.xml
    filename: Optional[str]
        file path relative to the package path
    warnings: Optional[List[str]]
        print warnings if None or return them in the given list

    Returns
    -------
    PackageDefinition
    """
    try:
        root = safer_xml_from_string(data, "package")
    except Exception:
        raise


    # verify unique root node
    nodes = _get_nodes(root, 'package')
    if len(nodes) != 1:
        raise InvalidPackage("The manifest must contain a single 'package' root tag", filename)
    root = nodes[0]

    # format attribute
    value = _get_node_attr(root, 'format', default=1)
    pkg_format = int(value)
    assert pkg_format in (1, 2, 3), \
        f"Unable to handle package.xml format version '{pkg_format}', please update catkin_pkg"

    # name
    pkg_name = _get_node_value(_get_node(root, 'name', filename))

    # version and optional compatibility
    version_node = _get_node(root, 'version', filename)
    pkg_version = _get_node_value(version_node)
    pkg_version_compatibility = _get_node_attr(version_node, 'compatibility', default=None)

    # description
    pkg_description = _get_node_value(_get_node(root, 'description', filename), allow_xml=True, apply_str=False)

    # at least one maintainer, all must have email
    maintainers = _get_nodes(root, 'maintainer')
    pkg_maintainers: t.List[Person] = []
    for node in maintainers:
        pkg_maintainers.append(Person(_get_node_value(node, apply_str=False), _get_node_attr(node, 'email')))

    # urls with optional type
    url = _get_nodes(root, 'url')
    pkg_urls: t.List[Url] = []
    for node in urls:
        pkg_urls.append(Url(_get_node_value(node), _get_node_attr(node, 'type', default='website')))

    # authors with optional email
    pkg_authors: t.List[Person] = []
    authors = _get_nodes(root, 'author')
    for node in authors:
        pkg_authors.append(Person(_get_node_value(node, apply_str=False),
                                      _get_node_attr(node, 'email', default=None)))

    # at least one license
    licenses = _get_nodes(root, 'license')
    pkg_licenses: t.List[License] = []
    for node in licenses:
        pkg_licenses.append(License(_get_node_value(node), _get_node_attr(node, 'file', default=None)))

    errors = []
    # dependencies and relationships
    pkg_build_depends = _get_dependencies(root, 'build_depends')
    pkg_buildtool_depends = _get_dependencies(root, 'buildtool_depends')
    pkg_build_export_depends: t.List[Dependency] = []
    pkg_exec_depends: t.List[Dependency] = []
    pkg_buildtool_export_depends: t.List[Dependency] = []
    pkg_doc_depends: t.List[Dependencu] = []

    if pkg_format == 1:
        run_depends = _get_dependencies(root, 'run_depend')
        for d in run_depends:
            pkg_build_export_depends.append(deepcopy(d))
            pkg_exec_depends.append(deepcopy(d))
    if pkg_format != 1:
        pkg_build_export_depends = _get_dependencies(root, 'build_export_depend')
        pkg_buildtool_export_depends = _get_dependencies(root, 'buildtool_export_depends')
        pkg_exec_depends = _get_dependencies(root, 'exec_depend')
        depends = _get_dependencies(root, 'depend')
        for dep in depends:
            # check for collisions with specific dependencies
            same_build_depends = ['build_depend' for d in pkg_build_depends if d == dep]
            same_build_export_depends = ['build_export_depend' for d in pkg_build_export_depends if d == dep]
            same_exec_depends = ['exec_depend' for d in pkg_exec_depends if d == dep]
            if same_build_depends or same_build_export_depends or same_exec_depends:
                errors.append("The generic dependency on '%s' is redundant with: %s" % (
                dep.name, ', '.join(same_build_depends + same_build_export_depends + same_exec_depends)))
            # only append non-duplicates
            if not same_build_depends:
                pkg_build_depends.append(deepcopy(dep))
            if not same_build_export_depends:
                pkg_build_export_depends.append(deepcopy(dep))
            if not same_exec_depends:
                pkg_exec_depends.append(deepcopy(dep))
        pkg_doc_depends = _get_dependencies(root, 'doc_depend')
    pkg_test_depends = _get_dependencies(root, 'test_depend')
    pkg_conflicts = _get_dependencies(root, 'conflict')
    pkg_replaces = _get_dependencies(root, 'replace')

    # group dependencies and memberships
    pkg_group_depends = _get_group_dependencies(root, 'group_depend')
    pkg_member_of_groups = _get_group_memberships(root, 'member_of_group')

    if pkg_format == 1:
        for test_depend in pkg_test_depends:
            same_build_depends = ['build_depend' for d in pkg_build_depends if d == test_depend]
            same_run_depends = ['run_depend' for d in pkg_build_export_depends if d == test_depend]
            if same_build_depends or same_run_depends:
                errors.append(f'The test dependency on "{test_depend.name}" is redundant with: '
                              f'{", ".join(same_build_depends + same_build_export_depends + same_exec_depends)}')

    # exports
    export_node = _get_optional_node(root, 'export', filename)
    pkg_exports: t.List[Export] = []
    if export_node is not None:
        for node in [n for n in export_node.childNodes if n.nodeType == n.ELEMENT_NODE]:
            export = Export(str(node.tagName), _get_node_value(node, allow_xml=True))
            for key, value in node.attributes.items():
                export.attributes[str(key)] = str(value)
            pkg_exports.append(export)

    # verify that no unsupported tags and attributes are present
    errors += _check_known_attributes(root, ['format'])
    depend_attributes = ['version_lt', 'version_lte', 'version_eq', 'version_gte', 'version_gt']
    if pkg_format > 2:
        depend_attributes.append('condition')
    known = {
        'name': [],
        'version': ['compatibility'],
        'description': [],
        'maintainer': ['email'],
        'license': [],
        'url': ['type'],
        'author': ['email'],
        'build_depend': depend_attributes,
        'buildtool_depend': depend_attributes,
        'test_depend': depend_attributes,
        'conflict': depend_attributes,
        'replace': depend_attributes,
        'export': [],
    }
    if pkg_format == 1:
        known.update({
            'run_depend': depend_attributes,
        })
    if pkg_format != 1:
        known.update({
            'build_export_depend': depend_attributes,
            'buildtool_export_depend': depend_attributes,
            'depend': depend_attributes,
            'exec_depend': depend_attributes,
            'doc_depend': depend_attributes,
        })
    if pkg_format > 2:
        known.update({
            'group_depend': ['condition'],
            'member_of_group': ['condition']
        })
    if pkg_format > 2:
        known.update({
            'license': ['file'],
        })
    nodes = [n for n in root.childNodes if n.nodeType == n.ELEMENT_NODE]
    unknown_tags = set([n.tagName for n in nodes if n.tagName not in known.keys()])
    if unknown_tags:
        errors.append(f'The manifest of package "{pkg_name}" (with format version %d) must not contain the following '
                      f'tags: ", ".join(unknown_tags)')
    if 'run_depend' in unknown_tags and pkg_format >= 2:
        errors.append('Please replace <run_depend> tags with <exec_depend> tags.')
    elif 'exec_depend' in unknown_tags and pkg_format < 2:
        errors.append('Either update to a newer format or replace <exec_depend> tags with <run_depend> tags.')
    for node in [n for n in nodes if n.tagName in known.keys()]:
        errors += _check_known_attributes(node, known[node.tagName])
        if node.tagName not in ['description', 'export']:
            subnodes = [n for n in node.childNodes if n.nodeType == n.ELEMENT_NODE]
            if subnodes:
                errors.append(f'The "{node.tagName}" tag must not contain the following children: '
                              f'{", ".join([n.tagName for n in subnodes])}')

    if errors:
        raise InvalidPackage('Error(s):%s' % (''.join(['\n- %s' % e for e in errors])), filename)

def _get_nodes(parent, tagname):
    return [n for n in parent.childNodes if n.nodeType == n.ELEMENT_NODE and n.tagName == tagname]


def _get_node(parent, tagname, filename):
    nodes = _get_nodes(parent, tagname)
    if len(nodes) != 1:
        raise InvalidPackage('The manifest must contain exactly one "%s" tag' % tagname, filename)
    return nodes[0]


def _get_optional_node(parent, tagname, filename):
    nodes = _get_nodes(parent, tagname)
    if len(nodes) > 1:
        raise InvalidPackage('The manifest must not contain more than one "%s" tags' % tagname, filename)
    return nodes[0] if nodes else None


def _get_node_value(node, allow_xml=False, apply_str=True):
    if allow_xml:
        value = (''.join([n.toxml() for n in node.childNodes])).strip(' \n\r\t')
    else:
        value = (''.join([n.data for n in node.childNodes if n.nodeType == n.TEXT_NODE])).strip(' \n\r\t')
    if apply_str:
        value = str(value)
    return value


def _get_node_attr(node, attr, default=False):
    """:param default: False means value is required."""
    if node.hasAttribute(attr):
        return str(node.getAttribute(attr))
    if default is False:
        raise InvalidPackage('The "%s" tag must have the attribute "%s"' % (node.tagName, attr))
    return default


def _get_dependencies(parent, tagname):
    depends = []
    for node in _get_nodes(parent, tagname):
        depend = Dependency(_get_node_value(node))
        for attr in ('version_lt', 'version_lte', 'version_eq', 'version_gte', 'version_gt', 'condition'):
            setattr(depend, attr, _get_node_attr(node, attr, None))
        depends.append(depend)
    return depends


def _get_group_dependencies(parent, tagname):
    from .group_dependency import GroupDependency
    depends = []
    for node in _get_nodes(parent, tagname):
        depends.append(
            GroupDependency(
                _get_node_value(node),
                condition=_get_node_attr(node, 'condition', default=None)))
    return depends


def _get_group_memberships(parent, tagname):
    from .group_membership import GroupMembership
    memberships = []
    for node in _get_nodes(parent, tagname):
        memberships.append(
            GroupMembership(
                _get_node_value(node),
                condition=_get_node_attr(node, 'condition', default=None)))
    return memberships
