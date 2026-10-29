from x2_twin_sim.mc import input_sources as s


def test_add_then_accept():
    r = s.InputSourceRegistry()
    assert r.apply(s.ADD, 'sdk_nav_adapter', 30, 1000) == s.OK
    assert r.accepts('sdk_nav_adapter')


def test_unregistered_and_disabled_sources_are_dropped():
    r = s.InputSourceRegistry()
    assert not r.accepts('sdk_x')
    r.apply(s.ADD, 'sdk_x', 30, 1000)
    r.apply(s.DISABLE, 'sdk_x')
    assert not r.accepts('sdk_x')
    r.apply(s.ENABLE, 'sdk_x')
    assert r.accepts('sdk_x')


def test_system_sources_cannot_be_touched():
    r = s.InputSourceRegistry()
    for name in ('rc', 'app_proxy', 'linkswarm', 'pnc'):
        assert r.apply(s.ADD, name, 30, 1000) == s.REFUSED
    assert r.counters['refused'] == 4


def test_add_requires_documented_fields_and_rejects_duplicates():
    r = s.InputSourceRegistry()
    assert r.apply(s.ADD, '', 30, 1000) == s.REFUSED
    assert r.apply(s.ADD, 'sdk_a', 130, 1000) == s.REFUSED
    assert r.apply(s.ADD, 'sdk_a', 30, 0) == s.REFUSED
    assert r.apply(s.ADD, 'sdk_a', 30, 1000) == s.OK
    assert r.apply(s.ADD, 'sdk_a', 30, 1000) == s.REFUSED


def test_modify_delete():
    r = s.InputSourceRegistry()
    r.apply(s.ADD, 'sdk_a', 30, 1000)
    assert r.apply(s.MODIFY, 'sdk_a', 35, 500) == s.OK and r.sources['sdk_a'].priority == 35
    assert r.apply(s.DELETE, 'sdk_a') == s.OK and not r.accepts('sdk_a')
    assert r.apply(s.DELETE, 'sdk_a') == s.REFUSED
