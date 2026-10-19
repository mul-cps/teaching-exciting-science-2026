"""Synthetic Trigger services only; no physical endpoint, camera or actuation."""
import json,os,threading,uuid
import pytest
from tools import ros_station_action as action


def test_actual_dds_fixed_recovery_and_supervisor_routes(capsys):
 if os.environ.get('ROS_DOMAIN_ID')!='177' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
  pytest.skip('requires explicit isolated DDS177 localhost1; never use live/default domain')
 rclpy=pytest.importorskip('rclpy')
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
  assert calls==list(action.PERCEPTION_SERVICES)
 finally:
  stop.set();thread.join(1);assert not thread.is_alive()
  executor.remove_node(node);executor.shutdown(timeout_sec=1)
  node.destroy_node();rclpy.shutdown(context=context)
