"""Status endpoint policy; ROS/parameter names remain legacy wire aliases."""


def _check(transport):
    if transport not in ("local", "lan2lan"):
        raise ValueError("transport_profile must be local or lan2lan")


def ego_status_enabled(transport, requested=None, port=9099):
    """9099 is the verified MolitComp03 CompetitionInfoPublisher input."""
    _check(transport)
    if transport == "lan2lan" and int(port) != 9099:
        raise ValueError("Verified LAN Competition destination port must be 9099")
    enabled = True if requested is None else bool(requested)
    if transport == "lan2lan" and not enabled:
        raise ValueError("LAN requires CompetitionInfoPublisher 9099 /Ego_topic")
    return enabled


def detail_status_enabled(transport, requested=None):
    """Legacy competition_port 9798 is a local-only diagnostic endpoint."""
    _check(transport)
    enabled = transport == "local" if requested is None else bool(requested)
    if transport == "lan2lan" and enabled:
        raise ValueError("9798 Detail is local diagnostic only, not LAN Competition")
    return enabled
