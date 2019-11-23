# -*- coding: utf-8 -*-
__all__ = ('TCPROSHeader',)

from io import BytesIO
from typing import Dict, Union, Optional, BinaryIO

import attr

from ..definitions.encode import write_uint32, write_encoded_header
from ..definitions.decode import read_string_dictionary

_UTF8_ONE = '1'.encode('utf-8')


@attr.s(frozen=True, auto_attribs=True, slots=True)
class TCPROSHeader:
    """Stores information for a TCPROS header.

    Attributes
    ----------
    callerid: str
        The name of the node that is sending data.
    md5sum: str
        The md5sum of the associated message type.
    type_: str
        The name of the associated message type.
    message_definition: str
        The full text of the associated message definition.
    error: str, optional
        An optional human-readable error message if the connection is not
        successful
    persistent: bool, optional
        Sent from a service client to a service. If :code:`True`, the
        connection will be kept open for multiple requests.
    tcp_nodelay: bool, optional
        Sent from a subscriber to a publisher. If :code:`True`, the publisher
        will set TCP_NODELAY on the socket if possible.
    latching: bool, optional
        Indicates that the publisher is in latching mode. I.e., the publisher
        sends the last value that it published to any new subscribers.
    topic: str, optional
        The name of the topic, if any, to which the subscriber is connected.
    service: str, optional
        The name of service, if any, that is being called.
    """
    callerid: str
    md5sum: str
    type_: str
    message_definition: str
    tcp_nodelay: Optional[bool] = attr.ib(default=None)
    persistent: Optional[bool] = attr.ib(default=None)
    latching: Optional[bool] = attr.ib(default=None)
    error: Optional[str] = attr.ib(default=None)
    topic: Optional[str] = attr.ib(default=None)
    service: Optional[str] = attr.ib(default=None)

    def write(self, b: BinaryIO) -> None:
        """Writes a binary encoding of this header to a given stream."""
        field_to_bytes: Dict[str, bytes] = {
            'callerid': self.callerid.encode('utf-8'),
            'md5sum': self.md5sum.encode('utf-8')}
        if self.message_definition:
            field_to_bytes['message_definition'] = \
                self.message_definition.encode('utf-8')
        if self.type_:
            field_to_bytes['type'] = self.type_.encode('utf-8')
        if self.service:
            field_to_bytes['service'] = self.service.encode('utf-8')
        if self.topic:
            field_to_bytes['topic'] = self.topic.encode('utf-8')
        if self.error:
            field_to_bytes['error'] = self.error.encode('utf-8')
        if self.tcp_nodelay:
            field_to_bytes['tcp_nodelay'] = _UTF8_ONE
        if self.persistent:
            field_to_bytes['persistent'] = _UTF8_ONE
        if self.latching:
            field_to_bytes['latching'] = _UTF8_ONE
        write_encoded_header(field_to_bytes, b)

    def encode(self) -> bytes:
        """Returns a binary encoding of this header."""
        b = BytesIO()
        self.write(b)
        return b.getvalue()

    @classmethod
    def decode(cls, b: bytes) -> 'TCPROSHeader':
        return cls.read(BytesIO(b))

    @classmethod
    def read(cls, b: BinaryIO) -> 'TCPROSHeader':
        field_to_value: Dict[str, str] = read_string_dictionary(b)

        def fetch_optional_bool(key: str) -> Optional[bool]:
            if key not in field_to_value:
                return None
            return field_to_value[key] == '1'

        callerid = field_to_value['callerid']
        md5sum = field_to_value['md5sum']
        type_ = field_to_value['type']
        message_definition = field_to_value['message_definition']
        service = field_to_value.get('service')
        topic = field_to_value.get('topic')
        error = field_to_value.get('error')
        tcp_nodelay = fetch_optional_bool('tcp_nodelay')
        persistent = fetch_optional_bool('persistent')
        latching = fetch_optional_bool('latching')

        return TCPROSHeader(callerid=callerid,
                            md5sum=md5sum,
                            type_=type_,
                            message_definition=message_definition,
                            tcp_nodelay=tcp_nodelay,
                            persistent=persistent,
                            latching=latching,
                            error=error,
                            topic=topic,
                            service=service)
