Inside this directory do as usual.

```bash
mkdir build & cd build
cmake ..
cmake --build .
# If you want to install
# This install everything into a local folder <RepoRoot>/cmake_build/install 
cmake --install .
```

Sourcing the install folder by running
```bash
source <RepoRoot>/cmake_build/install/setup.sh
```

Add this to your .bashrc file to permanently install the lib or change the install cmake install prefix.
