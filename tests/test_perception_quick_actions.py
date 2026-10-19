"""Fixed diagnostic service routing; never invoke a physical endpoint."""
import json,sys,types
import pytest
from tools import ros_station_action as action

ENDPOINTS={
 'perception-recover':'/so101_perception_recovery/start_recovery',
 'perception-status':'/so101_perception_recovery/recovery_status',
 'perception-supervisor-enable':'/so101_perception_supervisor/enable',
 'perception-supervisor-disable':'/so101_perception_supervisor/disable',
 'perception-supervisor-status':'/so101_perception_supervisor/status',
 'perception-supervisor-reset-fault':'/so101_perception_supervisor/reset_fault',
 'perception-supervisor-reconcile-consumers':'/so101_perception_supervisor/reconcile_consumers',
}

@pytest.mark.parametrize('command',ENDPOINTS)
def test_fixed_commands_accept_no_endpoint_or_path(command):
 assert action.parse_args([command]).command==command
 with pytest.raises(SystemExit):action.parse_args([command,'/arbitrary/service'])


def install_ros(monkeypatch,message,success=True,hook=lambda stage:None,pending=False):
 calls=[]
 class Request:pass
 class Future:
  def done(self):hook('done');return not pending
  def result(self):hook('result');return types.SimpleNamespace(success=success,message=message if isinstance(message,str) else json.dumps(message))
  def cancel(self):calls.append('cancel')
 class Client:
  def wait_for_service(self,timeout_sec):calls.append(('wait',timeout_sec));hook('discovery');return True
  def call_async(self,request):calls.append('call');hook('dispatch');return Future()
  def remove_pending_request(self,future):calls.append('remove')
 class Node:
  def __init__(self,name):calls.append(('node',name))
  def create_client(self,kind,name):calls.append(('service',name));return Client()
  def destroy_node(self):calls.append('destroy')
 ros=types.ModuleType('rclpy');ros.init=lambda:(calls.append('init'),hook('init'));ros.shutdown=lambda:calls.append('shutdown');ros.spin_once=lambda *a,**k:hook(('spin',k['timeout_sec']))
 node=types.ModuleType('rclpy.node');node.Node=Node
 srvs=types.ModuleType('std_srvs.srv');srvs.Trigger=types.SimpleNamespace(Request=Request)
 monkeypatch.setitem(sys.modules,'rclpy',ros);monkeypatch.setitem(sys.modules,'rclpy.node',node);monkeypatch.setitem(sys.modules,'std_srvs.srv',srvs)
 # Trigger-only perception commands must not need the arm/EV3 workflow interfaces.
 monkeypatch.setitem(sys.modules,'so101_station_workflow.srv',None)
 return calls


@pytest.mark.parametrize('command',ENDPOINTS)
def test_fake_ros_exact_endpoint_one_call_and_no_ready_inference(monkeypatch,capsys,command):
 message={'qualified':False,'execution_enabled':False,'operation_success':False,'ready_now':False}
 if command=='perception-recover':message.update(accepted=True,job_id='a'*32,state='native_preflight')
 if command.endswith('reset-fault'):message.update(reset_check_accepted=True,reset_check_pending=True)
 if command.endswith('reconcile-consumers'):message.update(accepted=True,reconciliation_id='b'*32,enabled=False)
 calls=install_ros(monkeypatch,message)
 monkeypatch.delenv('ROS_DOMAIN_ID',raising=False);monkeypatch.delenv('ROS_LOCALHOST_ONLY',raising=False)
 assert action.main([command])==0
 payload=json.loads(capsys.readouterr().out)
 assert payload['rpc_success'] is True and payload['success'] is True
 assert payload['ready_now'] is False and payload['operation_success'] is False
 assert payload['qualified'] is False and payload['execution_enabled'] is False
 assert payload['status']==message
 assert ('service',ENDPOINTS[command]) in calls and calls.count('call')==1
 assert __import__('os').environ['ROS_DOMAIN_ID']=='76'
 assert __import__('os').environ['ROS_LOCALHOST_ONLY']=='1'
 assert calls[-2:]==['destroy','shutdown']


def test_status_success_is_not_current_readiness(monkeypatch,capsys):
 calls=install_ros(monkeypatch,{'operation_success':True,'ready_now':False,'qualified':False,'execution_enabled':False})
 assert action.main(['perception-status'])==0
 payload=json.loads(capsys.readouterr().out)
 assert payload['operation_success'] is True and payload['ready_now'] is False
 assert calls.count('call')==1


@pytest.mark.parametrize('domain',['0','00','+0','-1','233','x','76.0'])
def test_new_command_invalid_domain_rejected_before_ros(monkeypatch,domain):
 monkeypatch.setenv('ROS_DOMAIN_ID',domain)
 monkeypatch.setitem(sys.modules,'rclpy',None)
 with pytest.raises(ValueError,match='ROS_DOMAIN_ID'):action.main(['perception-status'])


@pytest.mark.parametrize('message',[
 '[]','null','not json','{"qualified":true}','{"execution_enabled":true}',
 '{"ready_now":"false"}','{"operation_success":NaN}','{"value":1e309}',
 '{"ready_now":false,"ready_now":true}','{"text":"'+'x'*16384+'"}',
])
def test_bad_diagnostic_response_is_never_printed(monkeypatch,capsys,message):
 calls=install_ros(monkeypatch,message)
 with pytest.raises(ValueError):action.main(['perception-status'])
 assert not capsys.readouterr().out
 assert calls.count('call')==1 and calls[-2:]==['destroy','shutdown']


def test_preserves_explicit_nonzero_domain_and_localhost_setting(monkeypatch):
 install_ros(monkeypatch,{'ready_now':False})
 monkeypatch.setenv('ROS_DOMAIN_ID','177');monkeypatch.setenv('ROS_LOCALHOST_ONLY','0')
 assert action.main(['perception-status'])==0
 assert __import__('os').environ['ROS_DOMAIN_ID']=='177'
 assert __import__('os').environ['ROS_LOCALHOST_ONLY']=='0'


def test_help_without_any_ros_import(monkeypatch,capsys):
 monkeypatch.setitem(sys.modules,'rclpy',None)
 with pytest.raises(SystemExit) as error:action.main(['--help'])
 assert error.value.code==0 and 'perception-supervisor-reset-fault' in capsys.readouterr().out


@pytest.mark.parametrize('phase',['init','discovery','dispatch','result','serialization'])
def test_one_deadline_rejects_late_response_without_adoption(monkeypatch,capsys,phase):
 clock=types.SimpleNamespace(value=100.)
 monkeypatch.setattr(action,'time',types.SimpleNamespace(monotonic=lambda:clock.value))
 def hook(stage):
  if stage==phase:clock.value+=2.01
 calls=install_ros(monkeypatch,{'ready_now':True,'qualified':False,'execution_enabled':False},hook=hook)
 if phase=='serialization':
  def dump(*a,**kw):encoded=json.dumps(*a,**kw);clock.value+=2.01;return encoded
  monkeypatch.setattr(action,'json',types.SimpleNamespace(loads=json.loads,dumps=dump,JSONDecodeError=json.JSONDecodeError))
 with pytest.raises(TimeoutError,match='unknown'):action.main(['perception-status'])
 assert not capsys.readouterr().out
 assert calls.count('call')==int(phase not in {'init','discovery'})
 if calls.count('call'):assert 'remove' in calls and 'cancel' in calls
 assert calls[-2:]==(['destroy','shutdown'] if phase!='init' else ['init','shutdown'])


def test_discovery_and_rpc_share_budget_without_retry(monkeypatch,capsys):
 clock=types.SimpleNamespace(value=100.)
 monkeypatch.setattr(action,'time',types.SimpleNamespace(monotonic=lambda:clock.value))
 def hook(stage):
  if stage=='discovery':clock.value+=.75
  if isinstance(stage,tuple) and stage[0]=='spin':clock.value+=stage[1]
 calls=install_ros(monkeypatch,{'ready_now':False},hook=hook,pending=True)
 with pytest.raises(TimeoutError):action.main(['perception-status'])
 assert clock.value==pytest.approx(102.)
 assert len([c for c in calls if isinstance(c,tuple) and c[0]=='wait'])==1
 assert calls.count('call')==1 and not capsys.readouterr().out


def test_failed_rpc_status_does_not_infer_readiness(monkeypatch,capsys):
 install_ros(monkeypatch,{'reason':'disabled'},success=False)
 assert action.main(['perception-status'])==1
 value=json.loads(capsys.readouterr().out)
 assert not value['rpc_success'] and not value['ready_now'] and not value['operation_success']


def test_new_action_static_guard_has_one_bounded_rpc_and_no_device_paths():
 import ast,inspect,textwrap
 tree=ast.parse(textwrap.dedent(inspect.getsource(action._perception_action)))
 calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call)]
 assert sum(isinstance(n.func,ast.Attribute) and n.func.attr=='call_async' for n in calls)==1
 names={n.func.attr for n in calls if isinstance(n.func,ast.Attribute)}
 assert not names & {'open','read_bytes','write_bytes','Popen','run','kill','send_signal','sync_read','sync_write','hardware_reset'}
 loops=[n for n in ast.walk(tree) if isinstance(n,ast.While)]
 assert len(loops)==1 and 'remaining()' in ast.unparse(loops[0])
 assert action.PERCEPTION_SERVICES==ENDPOINTS
 source=inspect.getsource(action._perception_action)
 assert source.index('_perception_domain()')<source.index('import rclpy')<source.index('rclpy.init()')
 assert source.index('remaining()\n        print(encoded)')>source.index('json.dumps(payload')


@pytest.mark.parametrize('domain,localhost',[('76','1'),('0','1'),('177','0'),('177',None)])
def test_actual_dds_guard_rejects_wrong_domain_before_init(monkeypatch,domain,localhost):
 from test_perception_quick_actions_ros import _require_isolated_domain
 monkeypatch.setenv('ROS_DOMAIN_ID',domain)
 if localhost is None:monkeypatch.delenv('ROS_LOCALHOST_ONLY',raising=False)
 else:monkeypatch.setenv('ROS_LOCALHOST_ONLY',localhost)
 with pytest.raises(RuntimeError,match='DDS177 localhost1'):_require_isolated_domain()


def test_actual_dds_guard_precedes_init_static():
 import ast
 from pathlib import Path
 tree=ast.parse(Path(__file__).with_name('test_perception_quick_actions_ros.py').read_text())
 fixture=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='test_actual_dds_fixed_recovery_and_supervisor_routes')
 calls=[n for n in ast.walk(fixture) if isinstance(n,ast.Call)]
 guard=next(n for n in calls if isinstance(n.func,ast.Name) and n.func.id=='_require_isolated_domain')
 init=next(n for n in calls if isinstance(n.func,ast.Attribute) and n.func.attr=='init')
 assert guard.lineno<init.lineno
