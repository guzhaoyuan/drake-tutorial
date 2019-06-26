# Bazel basics

Everything happens under your _drake/_ folder.

```text
cd drake
```

### Build

To build the whole folder:

```text
bazel build //...
```

`//` means the root path of your drake folder, it equals `drake/`. `...` means build everything. It takes a while to build everything. To build all targets under a folder:

```text
bazel build //tools/...
```

Or to build a specific target under the folder:

```text
bazel build //tools:drake_visualizer
```

### Run

To run a specific executable:

```text
bazel run //examples/double_pendulum:double_pendulum_demo
```

`bazel run` will detect the file modification and build first if the file is changed and then run immediately after the build. `:` indicates what's followed is an executable binary, in this case, `double_pendulum_demo`.

Or you could execute the binary from terminal, which does not check the file change nor recompile. All the bazel binaries are put in the _drake/bazel-bin/_ folder automatically. The detailed location of an executable is defined by `BUILD.bazel`.

```text
bazel-bin/examples/double_pendulum/double_pendulum_demo
```

How to find where is the executable? Well, check the `BUILD.bazel` file and find the `drake_cc_binary` item. The executable sits in the `name` line.

### Test

This command applies when you wrote your own function and test cases. Then you could test your cases by:

```text
bazel test //common:polynomial_test
```

### For more bazel commands

Commands in this page are enough to handle most cases. 

For advanced bazel usage, check out the [Drake document of bazel](https://drake.mit.edu/bazel.html#using-bazel). 

To learn more about bazel, please refer to [bazel official document](https://docs.bazel.build/versions/master/bazel-overview.html).



