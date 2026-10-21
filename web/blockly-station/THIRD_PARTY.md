# Bundled dependencies

- Blockly 10.4.3, npm package `blockly@10.4.3`, upstream
  https://github.com/google/blockly/tree/blockly-v10.4.3 . Apache-2.0 license
  in `vendor/LICENSE.blockly`; compressed JS and media retained offline.
  npm package SHA1: a3ca155e2ba7bae59883a143b8b901c781bfac0d.
- ROS Blocky, https://github.com/ros-blocky/ros-blocky , pinned commit
  a34adbdef1c4b92374d2880a755e53436ca24754, MIT license in LICENSE.ros-blocky.
  `ros-blocky-port.js` ports the `node_logging_text` and `node_logging_info`
  definitions and Python generators from node-blocks.js/node-generator.js.
  Changes: English labels replace app translation imports; standalone registration;
  JSON.stringify provides Python-compatible string literals for the text subset.
  Original block type/field/input IDs and generated ROS logging semantics retained.
  This is a Linux station extension, not an unmodified ROS Blocky desktop build.
