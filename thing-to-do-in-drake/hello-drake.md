# Hello, Drake!

From now on, get your hands dirty. Follow the steps to start building your own code.

### Quick access to the tutorial demos

If you just want to see the result really quick, you could always skip the tutorial and use code on [my github branch](https://github.com/guzhaoyuan/drake). Here's how.

```text
cd drake
git remote add gzy https://github.com/guzhaoyuan/drake.git
git pull gzy tutorial
git checkout tutorial
bazel run //examples/hello:hello_exe
```

Same process applies to all the other examples.

### Create workspace

To run a simple example, we need a workspace that hosts C++ source files and `BUILD.bazel` file.

```text
cd drake
mkdir -p examples/hello
```

### Add files to the workspace

Create files named `BUILD.bazel` and `hello.cc` under _examples/hello_.

{% hint style="info" %}
The code is [available here](https://github.com/guzhaoyuan/drake/tree/tutorial/examples/hello).
{% endhint %}

Add the following contents to `BUILD.bazel`.

{% code title="BUILD.bazel" %}
```text
load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_binary",
)

drake_cc_binary(
    name = "hello_exe",
    srcs = ["hello.cc"],
    data = [
    ],
    deps = [
        "//common:text_logging_gflags",
        "@gflags",
    ],
)
```
{% endcode %}

Add the following contents to `hello.cc`.

{% code title="hello.cc" %}
```cpp
/// @file
///
/// This example shows how to compile and run drake files

#include <gflags/gflags.h>

#include "drake/common/text_logging_gflags.h"

namespace drake {
namespace examples {
namespace hello {

DEFINE_string(your_name, "Zion",
              "Putting your name here so Drake recognize you.");

void DoMain() {
  drake::log()->info("Hello " + FLAGS_your_name + " from Drake!");
}

}  // namespace hello
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple hello Drake example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::hello::DoMain();
  return 0;
}
```
{% endcode %}

### Compile and execute

Use the following command to build and run the code.

```bash
bazel run //examples/hello:hello_exe
[2019-06-14 17:23:39.190] [console] [info] Hello Zion from Drake!
```

The `BUILD.bazel` tells `bazel` what and how to compile. So when we run the code, `bazel` will find and build the code under that folder.

Now let's try add argument to the executable. Try the following and see what you get.

```bash
bazel run //examples/hello:hello_exe -- --your_name=Russ
[2019-06-14 17:28:21.056] [console] [info] Hello Russ from Drake!
```

Hooray, you get your first Drake program running! Now you are ready to digest some real meat.

### The Code Explained

#### BUILD.bazel

`bazel` will search the whole directory recursively to find `BUILD.bazel`. `BUILD.bazel` specifies the sources and headers that should be built, and tells `bazel` the target libraries and executables that should be generated. We need a `BUILD.bazel` under the work space folder.

{% code title="BUILD.bazel" %}
```text
load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_binary",
)
```
{% endcode %}

Load pre-defined bazel rule. We need to build Drake binary, so we load `drake_cc_binary` at line 3.

{% code title="BUILD.bazel" %}
```text
drake_cc_binary(
    name = "hello_exe",
    srcs = ["hello.cc"],
    data = [
    ],
    deps = [
        "//common:text_logging_gflags",
        "@gflags",
    ],
)
```
{% endcode %}

Use `drake_cc_binary` to define how the binary is going to be built.  `drake_cc_binary` specifies: 1. target executable name; 2. the files to be build; 3. all the required data \(for example the robot mesh files\); 4. dependencies.

Use `//common:text_logging_gflags` and `@gflags` as dependency. `//common:text_logging_gflags` is a information logging tool. `gflags`  allows us to specify variables at run-time by passing arguments in command line, makes it easy to tune parameters without recompile.

#### hello.cc

{% code title="hello.cc" %}
```cpp
#include <gflags/gflags.h>

#include "drake/common/text_logging_gflags.h"
```
{% endcode %}

Include `gflags` header file. `gflags` parses the command line parameters at run-time.

Include drake logging header file which contains function `drake::log()`.

{% code title="hello.cc" %}
```cpp
DEFINE_string(your_name, "Zion",
              "Putting your name here so Drake recognize you.");
```
{% endcode %}

Define a variable `FLAGS_your_name` with default value and explanation text. We could change the value before executing the binary by adding `-- --your_name=Russ`.

{% code title="hello.cc" %}
```cpp
void DoMain() {
  drake::log()->info("Hello " + FLAGS_your_name + " from Drake!");
}
```
{% endcode %}

Define the `DoMain` function, which serves as our main process. This program plots a string to console.

{% code title="hello.cc" %}
```cpp
int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple hello Drake example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::hello::DoMain();
  return 0;
}
```
{% endcode %}

Entrance to the program. The program starts by parsing the argument specified by user. And then it executes the `DoMain()` function which plot the string with your name!

