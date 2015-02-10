research-computing-with-cpp-demo
================================

Project to provide a CMake based build system for the course.


Build Instructions
==================

Tested on Linux (gcc), Mac (clang).

```
git clone https://github.com/UCL-RITS/research-computing-with-cpp-demo
mkdir research-computing-with-cpp-demo-build
cd research-computing-with-cpp-demo-build
ccmake ../research-computing-with-cpp-demo
make
```

(should download and build Eigen, Boost, ITK) and a HelloWorld
and HelloWorldTest program. Then

```
cd RCCPP-build
make
```

So you code, compile, code, compile etc, running make in the RCCPP-build folder.

