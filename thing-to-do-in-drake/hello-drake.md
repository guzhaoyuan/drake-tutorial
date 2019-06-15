# Hello, Drake!

From now on, get your hands dirty. Follow the steps here to start building your own code.

### Create minimum compile work space

To create a simple example, we need a C++ source file and a `BUILD.bazel` file.

```text
cd drake
mkdir -p examples/hello
```

### Add files to the work space

Create a file named `BUILD.bazel` and `hello.cc` under _examples/hello_. 

Add the following content to `BUILD.bazel`.

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

Add the following content to `hello.cc`.

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

### Code Explanation

{% hint style="info" %}
The code is also [here on my github](https://github.com/guzhaoyuan/drake/tree/tutorial/examples/hello), you could copy the entire _hello/_ folder to your _drake/examples/_ and run the same experiment.
{% endhint %}



Hooray, your get your first Drake program running! Now you are ready to digest some real meat.





