# Hello, Drake!

From now on, get your hands dirty. Follow the steps here to start building your own code.

### Create work space

To create a simple example, we need a workspace, a C++ source file and a `BUILD.bazel` file.

```text
cd drake
mkdir -p examples/hello
```

### Add files to the work space

Create files named `BUILD.bazel` and `hello.cc` under _examples/hello_.

Add the following contents to `BUILD.bazel`.

{% code-tabs %}
{% code-tabs-item title="BUILD.bazel" %}
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
{% endcode-tabs-item %}
{% endcode-tabs %}

Add the following contents to `hello.cc`.

{% code-tabs %}
{% code-tabs-item title="hello.cc" %}
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
{% endcode-tabs-item %}
{% endcode-tabs %}

### Compile and execute

Run the following command to build and run the code.

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

Hooray, your get your first Drake program running! Now you are ready to digest some real meat.

### The Code Explained

{% hint style="info" %}
The code is also [available on my github](https://github.com/guzhaoyuan/drake/tree/tutorial/examples/hello), you could copy the entire _hello/_ folder to your _drake/examples/_ and run the same experiment.
{% endhint %}

#### BUILD.bazel

bazel will search the whole directory recursively to find `BUILD.bazel` files thus analysis all the taget libraries and executables. To build binary, we need a BUILD.bazel under the work space folder.

{% code-tabs %}
{% code-tabs-item title="BUILD.bazel" %}
```text
load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_binary",
)
```
{% endcode-tabs-item %}
{% endcode-tabs %}

Load pre-defined bazel rule. We need to build drake binary, so we load `drake_cc_binary` at line 3.

{% code-tabs %}
{% code-tabs-item title="BUILD.bazel" %}
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
{% endcode-tabs-item %}
{% endcode-tabs %}

Use `drake_cc_binary` to define how the binary is going to be built.  `drake_cc_binary` specifies the target executable name, which files to be build and all the required data and dependencies within the same project.

Use `//common:text_logging_gflags` and `gflags` as dependency. `//common:text_logging_gflags` is a infomation logging tool. `gflags` module allow us to specify variables at runtime by passing arguments in command line, makes it easy to tune parameters without recompile.

#### hello.cc

{% code-tabs %}
{% code-tabs-item title="hello.cc" %}
```cpp
#include <gflags/gflags.h>

#include "drake/common/text_logging_gflags.h"
```
{% endcode-tabs-item %}
{% endcode-tabs %}

Include `gflags` header file. 

Include drake logging header file which contains function `drake::log()`.

{% code-tabs %}
{% code-tabs-item title="hello.cc" %}
```cpp
DEFINE_string(your_name, "Zion",
              "Putting your name here so Drake recognize you.");
```
{% endcode-tabs-item %}
{% endcode-tabs %}

Define a variable `FLAGS_your_name` with default value and explanation text. We could change the value before executing the binary by adding `-- --your_name=Russ`.

{% code-tabs %}
{% code-tabs-item title="hello.cc" %}
```cpp
void DoMain() {
  drake::log()->info("Hello " + FLAGS_your_name + " from Drake!");
}
```
{% endcode-tabs-item %}
{% endcode-tabs %}

Define the `Domain` function, which serves as our main process. This program plot a string to concole.

{% code-tabs %}
{% code-tabs-item title="hello.cc" %}
```cpp
int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple hello Drake example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::hello::DoMain();
  return 0;
}
```
{% endcode-tabs-item %}
{% endcode-tabs %}

Entrance of the program. The program starts by parsing the argument specified by user.





