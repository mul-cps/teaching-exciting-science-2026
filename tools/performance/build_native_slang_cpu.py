#!/usr/bin/env python3
"""AOT build with fail-closed host ABI validation; no GPU execution or runtime compiler."""
import argparse
import hashlib
import json
import platform
from pathlib import Path
import subprocess


def validate(reflection):
    expected = dict(zip(('input', 'scratch', 'divisions', 'compact', 'p', 'cpuMasks', 'cpuDepth', 'cpuXYZ', 'cpuParents'), ((0,16),(16,16),(32,16),(48,16),(64,8),(72,16),(88,16),(104,16),(120,16))))
    def bindings(items):
        if len({x['name'] for x in items}) != len(items):
            raise ValueError('duplicate Slang CPU binding')
        if any(x['binding']['kind'] != 'uniform' or x['binding'].get('elementStride') != 0 for x in items):
            raise ValueError('unsupported Slang CPU binding kind')
        return {x['name']: (x['binding']['offset'], x['binding']['size']) for x in items}
    if len(reflection['parameters']) != len(expected) or bindings(reflection['parameters']) != expected:
        raise ValueError('unsupported Slang CPU global ABI')
    fields = next(x for x in reflection['parameters'] if x['name'] == 'p')['type']['elementType']
    names = ('width','height','count','capacity','scale','fx','fy','cx','cy','maxDepth','jump','round')
    if len(fields['fields']) != len(names) or bindings(fields['fields']) != {name: (4*i,4) for i,name in enumerate(names)}:
        raise ValueError('unsupported Slang CPU Parameters ABI')
    types = {'divisions':'int32','cpuMasks':'uint8','cpuDepth':'float32','cpuXYZ':'float32','cpuParents':'uint32'}
    for parameter in reflection['parameters']:
        if parameter['type']['sizes'] != [{'kind':'uniform','value':expected[parameter['name']][1],'alignment':8}]:
            raise ValueError('unsupported Slang CPU descriptor alignment')
        if parameter['name'] in types and parameter['type']['resultType']['scalarType'] != types[parameter['name']]:
            raise ValueError('unsupported Slang CPU element type')
    for i, field in enumerate(fields['fields']):
        if field['type']['scalarType'] != ('uint32' if i<4 or i==11 else 'float32'):
            raise ValueError('unsupported Slang CPU Parameters scalar type')
    if fields['sizes'] != [{'kind':'uniform','value':48,'alignment':4}]:
        raise ValueError('unsupported Slang CPU Parameters size')
    for parameter in reflection['parameters']:
        t=parameter['type']; name=parameter['name']
        if name == 'p':
            if t['kind'] != 'constantBuffer':
                raise ValueError('unsupported Slang CPU Parameters kind')
        else:
            shape='structuredBuffer' if name in types else 'byteAddressBuffer'
            if t['kind'] != 'resource' or t['baseShape'] != shape:
                raise ValueError('unsupported Slang CPU resource kind')
            if name in types:
                size=1 if name=='cpuMasks' else 4
                if t['resultType']['sizes'] != [{'kind':'uniform','value':size,'alignment':size}]:
                    raise ValueError('unsupported Slang CPU resource element stride')
    entries = reflection['entryPoints']
    if len(entries) != 2 or {x['name'] for x in entries} != {'cpu_convert','cpu_candidates'}:
        raise ValueError('unsupported Slang CPU exports')
    # The adapter supplies nullptr entryPointParams. Only the system-provided
    # uint3 thread ID is supported: reject any uniform payload or unknown ABI.
    tid = {'name':'tid','semanticName':'SV_DISPATCHTHREADID','type':{
        'kind':'vector','elementCount':3,
        'elementType':{'kind':'scalar','scalarType':'uint32'},'sizes':[]}}
    scope = {'kind':'constantBuffer','binding':{
        'kind':'uniform','offset':0,'size':8,'elementStride':0},'parameters':[tid]}
    for entry in entries:
        if entry.get('parameters') != [tid] or entry.get('scope') != scope:
            raise ValueError('unsupported Slang CPU entry parameter payload')
        if entry['stage'] != 'compute' or entry['threadGroupSize'] != [1,1,1] or bindings(entry['bindings']) != expected:
            raise ValueError('unsupported Slang CPU dispatch ABI')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--slangc', required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--build-tests', action='store_true', help='also build standalone CPU contract and benchmark binaries')
    args = parser.parse_args()
    if platform.machine() not in ('x86_64','AMD64'):
        parser.error('native Slang CPU ABI is currently verified only for x86_64')
    repo = Path(__file__).resolve().parents[2]
    package = repo/'ros2/src/so101_station_vulkan'
    out = args.output_dir.resolve(); out.mkdir(parents=True,exist_ok=True)
    obj = out/'resident_cpu.o'; obj.unlink(missing_ok=True)
    pending = out/'resident_cpu.pending.o'
    reflection = out/'resident_cpu.reflection.json'
    command = [args.slangc,str(package/'shaders/resident_cpu.slang'),'-entry','cpu_convert','-stage','compute','-entry','cpu_candidates','-stage','compute','-target','object-code','-emit-cpu-via-llvm','-llvm-cpu','generic','-fp-mode','precise','-O2','-reflection-json',str(reflection),'-o',str(pending)]
    subprocess.run(command,check=True)
    validate(json.loads(reflection.read_text()))
    pending.replace(obj)
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    source = {f: digest(package/'shaders'/f) for f in ('resident_cpu.slang','resident.slang','exact_divide.slang')}
    identity = {'compiler':str(Path(args.slangc).resolve()),'compiler_sha256':digest(Path(args.slangc)), 'version':subprocess.check_output([args.slangc,'-version'],stderr=subprocess.STDOUT,text=True).strip(),'command':command,'sources':source,'object_sha256':digest(obj),'reflection_sha256':digest(reflection),'abi':'x86_64 Globals136 Parameters48 Grid24 groups1 whole-frame loops','runtime_compiler':False,'cpu_target':'explicit generic; slangc -help states generic CPU baseline'}
    (out/'native_cpu_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
    if args.build_tests:
        common = ['c++','-std=c++17','-O2','-ffp-contract=off','-I'+str(package/'include')]
        sources = [str(package/'src'/f) for f in ('gpu_pipeline.cpp','cpu_pipeline.cpp','frame_pool.cpp','vulkan_context.cpp')]
        native = ['-DRESIDENT_SLANG_CPU',str(package/'src/cpu_slang.cpp'),str(obj)]
        subprocess.run(common+[str(package/'test/cpu_slang_test.cpp')]+sources+['-lvulkan','-o',str(out/'absent-test')],check=True)
        subprocess.run(common+[str(package/'test/cpu_slang_test.cpp')]+sources+native+['-lvulkan','-o',str(out/'native-test')],check=True)
        subprocess.run(common+['-DDEFAULT_SHADER_DIR=""',str(package/'src/benchmark_rgbd.cpp')]+sources+native+['-lvulkan','-o',str(out/'benchmark_rgbd')],check=True)
        subprocess.run([str(out/'absent-test')],check=True)
        subprocess.run([str(out/'native-test')],check=True)
    print(obj)

if __name__ == '__main__':
    main()
