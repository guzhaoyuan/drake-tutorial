# Hello, Drake!

From now on, get your hands dirty. Follow the steps here to start building your own code.

### Create minimum compile workspace

To create a simple example, we need a source file and a BUILD.bazel file.

```text
cd drake
mkdir -p examples/hello
```

Create a file named `BUILD.bazel` and `hello.cc` under  _examples/hello_. Add the following content to `BUILD.bazel`.

```text

```

Add the following content to `hello.cc`.

```text

```

The `BUILD.bazel` tells `bazel` what and how to compile. So when we run the code, `bazel` will find and build the code under the folder. Run the following command to build and run the code.

```text
bazel run //examples/hello:hello
```

See what you get. The result of Drake logging its output to console. Hooray, your get your first Drake program running! Now you are ready to digest some real meat.

