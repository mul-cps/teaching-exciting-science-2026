/* Adapted from ROS Blocky a34adbdef1c4b92374d2880a755e53436ca24754.
 * Copyright (c) 2026 ros-blocky. MIT, see LICENSE.ros-blocky.
 * Compatible block IDs and ROS Python logging generators; no hardware publishing.
 */
function registerRosBlocky(Blockly, pythonGenerator) {
  Blockly.Blocks.node_logging_text = {init() {
    this.appendDummyInput().appendField('"').appendField(new Blockly.FieldTextInput('Hello robot'), 'TEXT').appendField('"');
    this.setOutput(true, 'String'); this.setColour('#7f8c8d');
  }};
  Blockly.Blocks.node_logging_info = {init() {
    this.appendValueInput('MESSAGE').appendField('ROS log info');
    this.setInputsInline(true); this.setPreviousStatement(true); this.setNextStatement(true); this.setColour('#7f8c8d');
  }};
  pythonGenerator.forBlock.node_logging_text = function(block) {
    return [JSON.stringify(block.getFieldValue('TEXT') || ''), pythonGenerator.ORDER_ATOMIC];
  };
  pythonGenerator.forBlock.node_logging_info = function(block) {
    const message=pythonGenerator.valueToCode(block,'MESSAGE',pythonGenerator.ORDER_NONE) || '"Message"';
    return `self.get_logger().info(str(${message}))\n`;
  };
}
if (typeof module !== 'undefined') module.exports={registerRosBlocky};
