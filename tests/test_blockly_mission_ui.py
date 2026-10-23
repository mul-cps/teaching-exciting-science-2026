from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def test_ui_consumes_canonical_missions_and_executable_export():
    js=(ROOT/'web/blockly-station/app.js').read_text()
    assert "fetch('curriculum.json')" in js
    assert "api('python',{program:compile()})" in js
    assert 'events.length%3' not in js
    assert 'previewProgram' not in js
    assert 'approach_recording' in js
    assert "admission:window.stationStatus.admission" in js
