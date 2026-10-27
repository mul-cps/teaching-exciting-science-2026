#!/usr/bin/env python3
"""Verify this frozen negative artifact; does not fit a plane or execute a GPU."""
from pathlib import Path
import argparse,gzip,hashlib,json,subprocess,tarfile
import numpy as np
BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
def sha(data):return hashlib.sha256(data).hexdigest()
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--private-archive',type=Path);args=p.parse_args()
 report=json.loads((ROOT/'docs/performance/support-plane-factor-probe-20261027.json').read_text())
 assert report['qualified'] is False and report['decision']=='QR_NOT_ADMITTED'
 for name,expected in report['evidence_files'].items():
  file=BASE/name;assert file.is_file() and file.stat().st_size==expected['size'] and sha(file.read_bytes())==expected['sha256'],name
 readout=ROOT/'docs/performance/support-plane-factor-probe-20261027.md';assert sha(readout.read_bytes())==report['readout_sha256']
 provenance=json.loads((BASE/'provenance.json').read_text())
 for name,digest in provenance['repository_sources'].items():
  blob=subprocess.check_output(['git','show',provenance['repository_source_commit']+':'+name],cwd=ROOT);assert sha(blob)==digest,name
 witness=json.loads((BASE/'negative-witnesses.json').read_text());assert len(witness['internal_only_normal_sign_cases'])==8
 summary=witness['boundary_summary']['fixed-full'];assert summary['total']==170 and summary['public_passed']==151 and len(summary['public_failures'])==19
 assert witness['ordinary_summary']['fixed-full']['public_passed']==32
 for name,row in witness['selected'].items():
  file=BASE/row['input_file'];assert sha(file.read_bytes())==row['input_file_sha256']
  with np.load(file,allow_pickle=False) as arrays:
   points=arrays['points'];assert points.ndim==2 and points.shape[1]==3 and points.dtype==np.dtype('<f8')
   if 'points_sha256' in row:assert sha(np.ascontiguousarray(points).tobytes())==row['points_sha256']
   if name=='inlier-mask-401-versus402':
    a,b=arrays['reference_mask'],arrays['candidate_mask'];assert int(a.sum())==401 and int(b.sum())==402
    assert np.flatnonzero(a!=b).tolist()==[401]
    assert arrays['reference_residuals'][401]>.003 and arrays['candidate_residuals'][401]<=.003
 archive=json.loads((BASE/'private-archive.json').read_text());raw=gzip.decompress((BASE/archive['inventory']).read_bytes());assert sha(raw)==archive['uncompressed_inventory_sha256'];members=json.loads(raw)
 assert len(members)==archive['regular_file_members'] and sum(x['size'] for x in members)==archive['uncompressed_member_bytes']
 if args.private_archive:
  assert sha(args.private_archive.read_bytes())==archive['sha256'] and args.private_archive.stat().st_size==archive['size']
  with tarfile.open(args.private_archive,'r:gz') as t:
   actual=t.getmembers();assert [x.name for x in actual]==[x['path'] for x in members]
   for entry,expected in zip(actual,members):
    assert entry.isfile() and entry.size==expected['size'];assert sha(t.extractfile(entry).read())==expected['sha256'],entry.name
 payload=[ROOT/'docs/performance/support-plane-factor-probe-20261027.md',ROOT/'docs/performance/support-plane-factor-probe-20261027.json',*BASE.iterdir()]
 total=sum(x.stat().st_size for x in payload if x.is_file());assert total<=200000,total
 print(json.dumps(dict(artifact_verified=True,qualified=False,decision='QR_NOT_ADMITTED',boundary_public_passed=151,boundary_total=170,inlier_counterexample=[401,402],published_bytes=total,private_archive_verified=bool(args.private_archive)),sort_keys=True))
if __name__=='__main__':main()
