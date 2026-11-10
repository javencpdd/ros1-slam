"""Publisher allow-list policy for the demo 3D goal gate.

Pure stdlib, no ROS import: the console demo runs on hosts without a sourced
ROS environment, so the decision function must be testable offline.

BOUNDARY: ROS1 ``callerid`` comes from the subscriber connection header and is
trivially spoofable by any node that can publish.  This is a demo-plane writer
ownership check, NOT authentication and NOT a security boundary.  It does not
change the "agent trusts the LAN" decision (that is the network side; this is
the ROS topic side).  Formal / EVT / real-robot paths stay on the work order.
"""
from __future__ import annotations

DEFAULT_ALLOWED_CALLERS = ("/bt_console",)


def parse_allowed(raw):
    """Normalise a param value into a tuple of callerids.

    Accepts a comma/space separated string or any sequence.  ``None`` means
    the built-in default.  An explicitly empty value is NOT a wildcard: it
    yields an empty tuple, so every request is refused (fail closed).
    """
    if raw is None:
        return tuple(DEFAULT_ALLOWED_CALLERS)
    if isinstance(raw, str):
        items = [s.strip() for s in raw.replace(",", " ").split()]
    else:
        try:
            items = [str(s).strip() for s in raw]
        except TypeError:
            return tuple()
    return tuple(s for s in items if s)


def accept_request(callerid, lease, fresh, allowed_callers=DEFAULT_ALLOWED_CALLERS,
                   require_acl=True, require_lease=True):
    """Decide whether a 3D goal request may be forwarded.

    Returns ``(accepted, reason)``.  Every unknown / absent identity is
    refused instead of being treated as a wildcard.

    Order matters: the identity check runs first, so a dropped request is
    reported as an ACL refusal even when the lease is also stale.
    """
    if require_acl:
        if not isinstance(callerid, str) or not callerid.strip():
            return False, "callerid_missing"
        if callerid.strip() not in set(allowed_callers):
            return False, "callerid_not_allowed:" + callerid.strip()
    if require_lease and not (lease and fresh):
        return False, "lease_invalid"
    return True, "ok"
