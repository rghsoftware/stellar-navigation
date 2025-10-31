# cFS Custom Applications

## STARNAV_APP

Device driver and protocol handler for STM32 stellar navigation system.

### Installation

Link this app to your cFS installation:
```bash
ln -s $(pwd)/starnav ~/workspace/cFS/apps/starnav
```

### Building

```bash
cd ~/workspace/cFS
make prep
make -j$(nproc)
make install
```

### Configuration

Edit `~/workspace/cFS/sample_defs/targets.cmake` to include:
```cmake
list(APPEND MISSION_GLOBAL_APPLIST starnav)
set(cpu1_APPLIST sample_app starnav)
```
