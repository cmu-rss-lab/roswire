# TODO how do we capture complex state? arrays, vectors, objects.
# declare the state attributes

ArduState = StateFormat.create() \
    .attr('armed', bool) \
    .attr('mode', str) \
    .attr('connected', bool) \
    .attr('wp_seq', int) \
    .attr('longitude', int) \
    .attr('latitude', float) \
    .attr('alitude', float) \
    .attr('heading', float) \
    .build()

# s.evolve(m) -> State
# s.evolution(msgs) -> Iterator[State]
# State.from_messages(initial, msgs) -> Iterator[State]

# - save to disk: use compact format (Protobuff?(
# - load from disk: use compact format

# - ability to look at states at specific points in time
# - .at_time_offset(210301)

# use a separate .evolve_on for readability
t.evolve_on('/mavros/global_position/global',
    lambda s, m: s.update('latitude', m.latitude))
t.evolve_on('/mavros/global_position/global',
    lambda s, m: s.update('longitude', m.longitude))
t.evolve_on('/mavros/global_position/global',
    lambda s, m: s.update('altitude', m.altitude))

t.evolve_on('/mavros/global_position/compass_hdg',
    lambda s, m: s.update('heading', m))

# or use a single .evolve_on
# either way, the same code is generated :-)
t.evolve_on('/mavros/state',
    lambda s, m: s.update('armed',      m.armed
                 ).update('mode',       m.mode
                 ).update('connected',  m.connected))

t.evolve_on('/mavros/mission/reached',
    lambda s, m: s.update('wp_seq', m.wp_seq))
