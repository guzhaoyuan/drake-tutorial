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

`//` means the root path of your drake folder, it equals `drake/`. `...` means build everything. Build everything can take a long time. We could speed up building by narrow down the range. For example, to build all targets under a subfolder:

```text
bazel build //tools/...
```

Or to build a specific target:

```text
bazel build //tools:drake_visualizer
```

### Run

To run a specific executable:

```text
bazel run //examples/double_pendulum:double_pendulum_demo
```

`bazel run` will detect the relevant file modification. If the files are changed, this command will build first and then run immediately after the build. `:` is followed by the executable binary, in this case, `double_pendulum_demo`.

Another way to run the binary is to type the binary name directly in the terminal. This way it does not check the file change nor recompile. All the bazel binaries are put in the _drake/bazel-bin/_ folder after build. The detailed location of an executable is defined by `BUILD.bazel`.

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



