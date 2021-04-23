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

{% hint style="warning" %}
`bazel build` takes all available CPUs to compile. The compilation would fail if not enough memory is available. To [limit the number of concurrent tasks](https://docs.bazel.build/versions/master/user-manual.html#flag--jobs), use `bazel build //... --jobs=n` , where n is the number of CPUs used.
{% endhint %}

`//` means the root path of your drake folder, it equals `drake/`. `...` means build everything. Build everything can take a long time. We can speed up the building by narrowing down to a subfolder. For example, to build all targets under a subfolder:

```text
bazel build //tools/...
```

Or to build a specific target:

```text
bazel build //tools:drake_visualizer
```

`//tools` tells `bazel` that the path of the target is _drake/tools/_. What's followed after `:` is the target `drake_visualizer`. `bazel` will find the `BUILD.bazel` file under _drake/tools/_ and build the target based on the rules defined in the `BUILD.bazel`.

### Run

To run a specific executable:

```text
bazel run //examples/double_pendulum:double_pendulum_demo
```

`bazel run` will detect the relevant file modification. If the files are changed, this command will build first and then run immediately after the build. `:` is followed by the executable binary, in this case, `double_pendulum_demo`.

Another way to run the binary is to type the binary name directly in the terminal. It does not check the file change nor recompile. All the `bazel` binaries are put in the _drake/bazel-bin/_ folder after being built. The detailed location of an executable is defined by `BUILD.bazel`.

```text
bazel-bin/examples/double_pendulum/double_pendulum_demo
```

How to find where is the executable? Well, check the `BUILD.bazel` file and find the `drake_cc_binary` item. The executable sits in the `name` line.

### Debug

using gdb to debug when you find errors like segmentation fault comes handy. We could compile the executable with gdb by:

```text
bazel build --compilation_mode=dbg //examples/multibody/inclined_plane_with_body
```

Then you could run the executable with gdb inspecting the variables and function calls. When there is a error and program crashes, we could trace back to where it went wrong.

```text
gdb ./bazel-bin/examples/multibody/inclined_plane_with_body
(gdb) run # execute the program and record the runtime stack data
(gdb) bt # back trace to where the program crashes and figure the problem
(gdb) quit
```

### Test

This command applies when you wrote your own function and test cases. Then you could test your cases by:

```text
bazel test //common:polynomial_test
```

### For more bazel commands

Commands in this page are enough to handle most cases. 

For advanced `bazel` usage, check out the [Drake document of bazel](https://drake.mit.edu/bazel.html#using-bazel). 

To learn more about `bazel`, please refer to [bazel official document](https://docs.bazel.build/versions/master/bazel-overview.html).



