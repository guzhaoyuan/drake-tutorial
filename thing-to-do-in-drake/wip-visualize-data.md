# \[WIP\] Visualize data

`Drake` could call python functions. So we could use matplotlib in python to create data plots.

This piece of tool is used for translating the function call in C++ into python and transmit some data format as well. We need to run the `call_python_client_cli` to enable this feature.

```text
cd drake
bazel build //common/proto:call_python_client_cli //common/proto:call_python_server_test
# Create default pipe file.
rm -f /tmp/python_rpc && mkfifo /tmp/python_rpc
# Run the translation software.
./bazel-bin/common/proto/call_python_client_cli
```

So in our code, we could use this service and call `plot` in python to draw figures.

```text

```

So we could see the tool receive the python function call request and data passed with this request. A figure pop up after the plot command. This is how we plot data in `Drake`.

