# Jetson deployment (aarch64)

Build and run `rl_sar` (with the model-preload feature) on an NVIDIA Jetson.
The preload code is portable — the only Jetson-specific work is supplying an
**aarch64 libtorch** and building natively on the device.

## 0. Prerequisites
Native compile on the Jetson (do **not** cross-compile). On the device:

```bash
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev \
                 libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev liblcm-dev
```

## 1. Supply an aarch64 libtorch  (the one real blocker)
CMake expects libtorch at `library/inference_runtime/libtorch/` (must contain
`include/` and `lib/`). The x86 libtorch from the PC will **not** link here.

Easiest source is NVIDIA's "PyTorch for Jetson" wheel (already aarch64 and
matched to your JetPack). After installing it into a Python env:

```bash
TORCH_DIR=$(python3 -c "import torch,os;print(os.path.dirname(torch.__file__))")
rm -rf library/inference_runtime/libtorch          # remove any x86 copy
ln -s "$TORCH_DIR" library/inference_runtime/libtorch
# sanity: these must exist
ls library/inference_runtime/libtorch/include library/inference_runtime/libtorch/lib
```

The pip `torch` package layout (`torch/include`, `torch/lib`,
`torch/share/cmake/Torch`) is exactly what `find_package(Torch)` needs.

> The model is a ~1.5 MB MLP, so CPU inference on an Orin is sub-millisecond.
> You can use the CUDA Jetson wheel and still force CPU at runtime if you want
> to keep the GPU free.

## 2. Build
`CMakeLists.txt` auto-detects Jetson (`/etc/nv_tegra_release`) + aarch64,
disables ONNX, and configures libtorch with Jetson CUDA arches. Use the CMake
path (NOT plain `./build.sh`, which runs colcon and needs ROS packages):

```bash
./build.sh -m       # --cmake: hardware targets incl. rl_real_g1
# or, to also build the MuJoCo sim for on-device testing:
./build.sh -mj      # adds -DUSE_MUJOCO=ON
```

For the real robot you also need **`unitree_sdk2` built for aarch64** (DDS link
to the G1).

Confirm the preload + logging compiled in:
```bash
strings cmake_build/bin/rl_real_g1 | grep -E "INITRL|PRELOAD"   # expect matches
```

## 3. Copy the policies
The preload reads every policy under `POLICY_DIR = <repo>/policy`. Ensure the
`policy/g1/` tree (`.pt` + `config.yaml`) is present on the Jetson.

## 4. Run
```bash
./cmake_build/bin/rl_real_g1 <network_interface>     # e.g. eth0  (real robot)
./cmake_build/bin/rl_sim_mujoco g1 scene_29dof       # sim, if built with -mj
```

At startup: `[PRELOAD] cached g1/asap_dab ...`
At the `5` switch: `[INITRL] model for g1/asap_dab from CACHE (preloaded, no disk I/O) | acquire = ~0 ms`

(Jetson storage is slower than the PC's, so the preload win is *larger* here —
a mid-control disk load would stall the motors for longer.)

## Not tracked in git
The aarch64 libtorch binaries / the `library/inference_runtime/libtorch` symlink
are machine-local — set them up on each Jetson, do not commit them.
