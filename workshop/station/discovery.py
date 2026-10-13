"""Read-only Linux USB/sysfs discovery; never opens or claims an interface."""
from pathlib import Path


def discover_serial(by_id=Path('/dev/serial/by-id')):
    """Inventory stable serial paths without opening their device nodes."""
    return [{'path': str(p), 'device': str(p.resolve())} for p in sorted(Path(by_id).glob('*')) if p.is_symlink()]


def discover_ev3(sysfs=Path('/sys/bus/usb/devices'), by_id=Path('/dev/serial/by-id')):
    """Return original-firmware EV3 USB identities (0694:0005).

    Missing/inaccessible serial descriptors are represented by an empty string;
    selection rejects them. Serial-by-id inventory is separate from EV3 USB.
    """
    identities = []
    for path in sorted(Path(sysfs).glob('*')):
        try:
            vendor = int((path / 'idVendor').read_text().strip(), 16)
            product = int((path / 'idProduct').read_text().strip(), 16)
            if (vendor, product) != (0x0694, 0x0005):
                continue
            try:
                serial = (path / 'serial').read_text().strip()
            except OSError:
                serial = ''
            identities.append({'vendor_id': vendor, 'product_id': product, 'serial': serial,
                               'bus': int((path / 'busnum').read_text().strip()),
                               'address': int((path / 'devnum').read_text().strip()), 'sysfs': str(path),
                               'serial_devices': discover_serial(by_id)})
        except (OSError, ValueError):
            continue
    return identities


def select_identity(identities, expected):
    """Require one exact commissioned VID/PID/serial; no fallback selection."""
    if not isinstance(expected.get('serial'), str) or not expected['serial'].strip():
        raise ValueError('exact EV3 USB serial is required')
    if (expected.get('vendor_id'), expected.get('product_id')) != (0x0694, 0x0005):
        raise ValueError('original EV3 requires USB identity 0694:0005')
    matches = [i for i in identities if all(i.get(k) == expected[k] for k in ('vendor_id', 'product_id', 'serial'))]
    if len(matches) != 1:
        raise ValueError('EV3 identity absent or ambiguous')
    return dict(matches[0])


def discover_arm(sysfs=Path('/sys/bus/usb/devices'), by_id=Path('/dev/serial/by-id')):
    """Inventory USB serial adapters, retaining exact serial and stable paths.

    Device model/follower role cannot be inferred from adapter IDs. The caller
    must select its commissioned VID/PID/serial, never the first tty path.
    """
    serial_paths = discover_serial(by_id)
    identities = []
    for path in sorted(Path(sysfs).glob('*')):
        try:
            vendor = int((path / 'idVendor').read_text().strip(), 16)
            product = int((path / 'idProduct').read_text().strip(), 16)
            nodes = {p.name for pattern in ('ttyUSB*', 'ttyACM*') for p in path.rglob(pattern)}
            paths = [item for item in serial_paths if Path(item['device']).name in nodes]
            if not nodes:
                continue
            try:
                serial = (path / 'serial').read_text().strip()
            except OSError:
                serial = ''
            identities.append({'vendor_id': vendor, 'product_id': product, 'serial': serial,
                               'bus': int((path / 'busnum').read_text().strip()),
                               'address': int((path / 'devnum').read_text().strip()),
                               'sysfs': str(path), 'serial_devices': paths})
        except (OSError, ValueError):
            continue
    return identities
