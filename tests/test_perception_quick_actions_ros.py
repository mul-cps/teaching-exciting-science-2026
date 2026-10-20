"""Synthetic Trigger services only; no physical endpoint, camera or actuation."""
import json,os,threading,uuid
import pytest
from tools import ros_station_action as action


def _require_isolated_domain():
 if os.environ.get('ROS_DOMAIN_ID')!='177' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
  raise RuntimeError('quick action ROS fixture requires explicit DDS177 localhost1')


def test_actual_dds_fixed_recovery_and_supervisor_routes(capsys):
 rclpy=pytest.importorskip('rclpy')
 _require_isolated_domain()  # Reject live/default domains before ROS init/network.
 from rclpy.context import Context
 from rclpy.node import Node
 from rclpy.executors import SingleThreadedExecutor
 from std_srvs.srv import Trigger
 context=Context();rclpy.init(context=context)
 node=Node('quick_action_fixture_'+uuid.uuid4().hex,context=context,use_global_arguments=False)
 executor=SingleThreadedExecutor(context=context);executor.add_node(node)
 calls=[];services=[];stop=threading.Event()
 for command,endpoint in action.PERCEPTION_SERVICES.items():
  def callback(request,response,command=command):
   calls.append(command)
   body={'schema':'synthetic-quick-action-test','operation_success':False,
         'ready_now':False,'qualified':False,'execution_enabled':False}
   if command=='perception-recover':body.update(accepted=True,job_id='b'*32,state='native_preflight')
   if command=='perception-supervisor-adopt-completed-job':
    body.update(accepted=True,adoption_id='d'*32,job_id='e'*32,enabled=False,state='local_adoption_preflight')
   if command=='perception-supervisor-reconcile-consumers':
    body.update(accepted=True,reconciliation_id='c'*32,enabled=False,state='operator_preflight')
   if command.endswith('reset-fault'):body.update(reset_check_accepted=True,reset_check_pending=True)
   if command.endswith('disable'):body['accepted_job_continues']=True
   response.success=True;response.message=json.dumps(body);return response
  services.append(node.create_service(Trigger,endpoint,callback))
 def spin():
  while not stop.is_set():executor.spin_once(timeout_sec=.01)
 thread=threading.Thread(target=spin,name='fake-perception-services');thread.start()
 try:
  for command in action.PERCEPTION_SERVICES:
   assert action.main([command])==0
   body=json.loads(capsys.readouterr().out)
   assert body['rpc_success'] is True and body['ready_now'] is False
   assert body['qualified'] is False and body['operation_success'] is False
   assert body['service']==action.PERCEPTION_SERVICES[command]
   if command=='perception-supervisor-adopt-completed-job':
    assert body['status']['adoption_id']=='d'*32 and body['status']['job_id']=='e'*32
    assert body['status']['accepted'] is True and body['status']['enabled'] is False
   if command=='perception-supervisor-reconcile-consumers':
    status=body['status']
    assert status['accepted'] is True and status['enabled'] is False
    assert status['reconciliation_id']=='c'*32 and status['state']=='operator_preflight'
    assert 'job_id' not in status and status['operation_success'] is False and status['ready_now'] is False
  assert calls==list(action.PERCEPTION_SERVICES)
 finally:
  stop.set();thread.join(1);assert not thread.is_alive()
  executor.remove_node(node);executor.shutdown(timeout_sec=1)
  node.destroy_node();rclpy.shutdown(context=context)
