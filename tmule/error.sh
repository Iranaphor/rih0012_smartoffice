INFO: Connecting to Bot at CE:2A:46:46:22:76...
INFO: Connected.
INFO: Turning ON
WARN: No running event loop, creating a new one
INFO: Connecting to Bot at CE:2A:46:46:22:76...
INFO: Failed to execute command: 
Future exception was never retrieved
future: <Future finished exception=BrokenPipeError(32, 'Broken pipe')>
Traceback (most recent call last):
  File "/home/thorvald/ros2_ws/build/switchbot_ros2/switchbot_ros2/control_bot.py", line 56, in command_callback
    loop = asyncio.get_running_loop()
RuntimeError: no running event loop

During handling of the above exception, another exception occurred:

Traceback (most recent call last):
  File "/home/thorvald/.local/lib/python3.10/site-packages/bleak/backends/bluezdbus/client.py", line 214, in connect
    reply = await self._bus.call(
  File "src/dbus_fast/aio/message_reader.py", line 19, in dbus_fast.aio.message_reader._message_reader
  File "src/dbus_fast/_private/unmarshaller.py", line 808, in dbus_fast._private.unmarshaller.Unmarshaller._unmarshall
  File "src/dbus_fast/_private/unmarshaller.py", line 661, in dbus_fast._private.unmarshaller.Unmarshaller._read_header
  File "src/dbus_fast/_private/unmarshaller.py", line 385, in dbus_fast._private.unmarshaller.Unmarshaller._read_to_pos
  File "src/dbus_fast/_private/unmarshaller.py", line 328, in dbus_fast._private.unmarshaller.Unmarshaller._read_sock_with_fds
EOFError

During handling of the above exception, another exception occurred:

Traceback (most recent call last):
  File "/home/thorvald/.local/lib/python3.10/site-packages/dbus_fast/aio/message_bus.py", line 103, in write_callback
    self.offset += sock.send(self.buf[self.offset :])
BrokenPipeError: [Errno 32] Broken pipe
Traceback (most recent call last):
  File "/home/thorvald/ros2_ws/build/switchbot_ros2/switchbot_ros2/control_bot.py", line 56, in command_callback
    loop = asyncio.get_running_loop()
RuntimeError: no running event loop

During handling of the above exception, another exception occurred:

Traceback (most recent call last):
  File "/home/thorvald/ros2_ws/install/switchbot_ros2/lib/switchbot_ros2/control_bot.py", line 33, in <module>
    sys.exit(load_entry_point('switchbot-ros2', 'console_scripts', 'control_bot.py')())
  File "/home/thorvald/ros2_ws/build/switchbot_ros2/switchbot_ros2/control_bot.py", line 75, in main
    executor.spin()
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 294, in spin
    self.spin_once()
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 794, in spin_once
    self._spin_once_impl(timeout_sec)
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 791, in _spin_once_impl
    future.result()
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/task.py", line 94, in result
    raise self.exception()
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/task.py", line 239, in __call__
    self._handler.send(None)
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 437, in handler
    await call_coroutine(entity, arg)
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 362, in _execute_subscription
    await await_or_execute(sub.callback, msg)
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 107, in await_or_execute
    return callback(*args)
  File "/home/thorvald/ros2_ws/build/switchbot_ros2/switchbot_ros2/control_bot.py", line 62, in command_callback
    loop.run_until_complete(self.execute_command(msg.data))
  File "/usr/lib/python3.10/asyncio/base_events.py", line 649, in run_until_complete
    return future.result()
  File "/home/thorvald/ros2_ws/build/switchbot_ros2/switchbot_ros2/control_bot.py", line 51, in execute_command
    await self.find_device()
TypeError: SwitchBotController.find_device() takes 0 positional arguments but 1 was given
The following exception was never retrieved: cannot use Destroyable because destruction was requested
