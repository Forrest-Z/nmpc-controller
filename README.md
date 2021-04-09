# nmpc-controller

Implementation of a trajectory tracking controller for differential motion using Nonlinear Model Predictive Control.

## Usage

### Install dependencies

```bash
sudo apt-get install cppad libeigen3-dev libbenchmark-dev
```

### Install Ipopt Solver

```bash
wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip
unzip Ipopt-3.12.7.zip
sudo bash install_ipopt.sh ./Ipopt-3.12.7
```

### Build

```bash
git clone https://github.com/nocoinman/nmpc-controller.git
cd nmpc-controller
mkdir build
cd build
cmake ..
make -j4
```

### Run benchmark

```bash
./build/bm_nmpc
```

#### Disabling CPU Frequency Scaling

If you see this error:

```
***WARNING*** CPU scaling is enabled, the benchmark real time measurements may be noisy and will incur extra overhead.
```

you might want to disable the CPU frequency scaling while running the benchmark:

```bash
sudo cpupower frequency-set --governor performance
./build/bm_nmpc
sudo cpupower frequency-set --governor powersave
```