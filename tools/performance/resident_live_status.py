"""Pure readiness validation for the bounded resident qualification collector."""


def status_ready(text: str, mode: str, already_ready: bool = False) -> bool:
    fields = {k.strip(): v.strip() for part in text.split(';') if '=' in part
              for k, v in [part.split('=', 1)]}
    if 'fatal' in text or fields.get('restart_required') == 'true':
        raise ValueError('source fatal/restart required')
    if fields.get('execution_eligible') != 'false' or fields.get('geometry_admission') != 'unqualified':
        raise ValueError('source qualification safety fields absent/changed')
    phase = text.split(';', 1)[0].strip()
    if phase == 'starting':
        if (already_ready or fields.get('source_admitted') != 'false'
                or fields.get('waiting_for_stable_source') != 'true'
                or 'backend' in fields or 'alignment' in fields):
            raise ValueError('invalid starting status or startup status received after readiness')
        return False
    if phase != 'experimental':
        raise ValueError('unknown or unexpected source status cannot establish readiness')
    expected = ('cpu', 'vendor_cpu') if mode == 'cpu' else ('vulkan', 'raw_vulkan')
    if (fields.get('backend'), fields.get('alignment')) != expected:
        raise ValueError('requested mode did not reach required backend/alignment; retain concrete fallback reason')
    return True
