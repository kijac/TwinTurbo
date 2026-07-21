# TwinTurbo: A Bidirectional RHT Feeder IP for TurboQuant-based KV Cache Compression

TwinTurbo is an FPGA Feeder IP that compresses and reconstructs LLM inference KV
Cache using a 4-bit TurboQuant-based algorithm (Randomized Hadamard Transform +
Lloyd-Max scalar quantization). It is designed to sit transparently between DRAM
and the NPU, reducing KV Cache storage footprint without modifying the existing
compute core.

> **Current status**: RTL design and self-verification (against a Python golden
> reference) are complete. Bring-up of the U250 baseline (llama-fpga) is in
> progress, and NPU integration / system-level performance measurement are
> planned as next steps. All results below are from module-level RTL simulation.

---

## 1. Background & Goals

LLM KV Cache grows linearly with context length, and storing it in FP16 quickly
becomes a memory bottleneck. TwinTurbo aims to compress KV Cache to 4 bits so
that the same memory budget supports longer contexts or larger batch sizes.

The design had to satisfy two conflicting goals simultaneously:

- **Quantization path**: prioritizes compression ratio and low reconstruction error
- **Dequantization path**: sits directly in the attention critical path and is
  cycle-critical

To address this, the following design decisions were made:

- Implemented the rotation transform (RHT) as an O(d log d) Butterfly-structured
  FWHT instead of an O(d²) matrix multiplication
- Absorbed the scaling factors (1/√d, 1/d) into the codebook/norm values offline,
  eliminating runtime multipliers
- Observed that the insertion position in the Bit Packer/Unpacker cycles through
  only a small number of fixed states, and replaced a general-purpose barrel
  shifter with a small fixed-slot multiplexer structure

---

## 2. Architecture

```
[Quantization Engine]
Matrix Processing Unit
        │ (FP16, 16bit)
        ▼
FP16<->INT16 Transformer -> I/O buffer -> L2 norm CALC/divider
        │ (24bit)
        ▼
RHT Calculator (D sign-flip -> FWHT, no 1/sqrt(d))
        │
        ▼
Centroid Sorter (<- Centroid LUT ROM) -> Bit Packer -> Device Memory


[Dequantization Engine]
Device Memory
        │
        ▼
Bit Unpacker -> I/O buffer (L2 norm + idx) -> Centroid Transform (16bit)
        │
        ▼
RHT Calculator (FWHT -> D sign-flip, no 1/sqrt(d))
        │
        ▼
multiplier (L2 norm, 1/d) -> INT16<->FP16 Transformer -> Matrix Processing Unit
```

- Both engines share the same RHT Core (D + FWHT) in a forward/inverse dual-use
  configuration (only the D-FWHT connection order is switched via a mode signal,
  using the relation Π = D·H / Πᵀ = H·D).
- The Centroid LUT ROM shares the same codebook for both the quantization
  comparator and the dequantization lookup.

---

## 3. Implemented Modules

Please refer to the TwinTurbo_PDF file.

## 4. Key Results (RTL simulation, 20-token basis)

### Compression Ratio

| head_dim (D) | Original (FP16) | Compressed | Ratio | Savings |
|---|---|---|---|---|
| 64 | 2560B | 690B | 3.71x | 73.05% |
| 128 | 5120B | 1330B | 3.85x | 74.02% |
| 256 | 10240B | 2610B | 3.92x | 74.51% |

### Accuracy (RTL reconstruction vs. original FP16 input)

| head_dim (D) | NMSE | NRMSE | idx mismatch rate |
|---|---|---|---|
| 64 | 0.0079 | 8.89% | 0.31% |
| 128 | 0.0112 | 10.57% | 0.31% |
| 256 | 1.00 (abnormal) | 100% (abnormal) | 20.20% (abnormal) |

For D=64/128, NMSE falls in a similar range to the 4-bit figure reported by the
TurboQuant paper (0.009). RTL results also match the float64-precision Python
golden reference to roughly 4 decimal places, confirming that the RTL faithfully
reproduces the algorithm under these conditions.

### Throughput (Dequantization path, D=128)

- Latency: 22-24 cycles/vector
- Throughput: ~15 cycles/vector (FWHT is the dominant bottleneck, ~60% of total)
- Approximately 120ns/vector at 125MHz

---

## 5. Known Issues

- **D=256 pipeline error**: Across the full quant->dequant pipeline, D=256 shows
  abnormally high error compared to D=64/128 (20% idx mismatch, NMSE near
  saturation). A guard-bit shortage in the FWHT accumulator is suspected (a
  `rht_warn` flag fires only at D=256), and standalone dequant testing revealed
  a saturation pattern in a specific lane. Root-cause analysis and fix are in
  progress.
- **Pre-NPU-integration**: All current results are from standalone Feeder IP RTL
  simulation. Integration with the U250 + llama-fpga baseline and system-level
  performance measurement (tokens/s, extended context length, etc.) are planned
  next steps.
- **No task-level validation yet**: Downstream-task-level accuracy metrics such
  as perplexity or zero-shot accuracy have not yet been measured.

---

## 6. Bug Found and Fixed

An early design error was discovered where the LUT centroid values were
incorrectly scaled by `centroid × sqrt(d)`.

- **Symptom**: When validated against real KV Cache data via the golden
  reference, SNR degraded sharply as d increased (d=64: 4.7dB -> d=256: -2.8dB)
- **Root-cause analysis**: Algebraically proved that the per-coordinate standard
  deviation of `y' = H·D·x_hat` is always close to 1 regardless of d (since
  ∥y'∥² = d holds as an identity), and re-confirmed this via 500-trial
  statistical verification across d=64-1024. Scaling the codebook by sqrt(d)
  causes an increasingly large mismatch between the actual data distribution
  and the codebook range as d grows.
- **Fix**: Changed the codebook to use the original (unscaled, N(0,1)-based
  Lloyd-Max) values directly.
- **Result**: SNR improved from -2.78dB to 20.09dB (measured on a D=256 synthetic
  test case).

---

## 7. Baseline / Integration Plan

- Baseline accelerator: [llama-fpga](https://github.com/adamgallas/llama-fpga)
  (LLaMA2-7B, AWQ 4-bit weights, supports U250/KV260/ZCU104)
- Target FPGA: Alveo U250 (XCU250)
- Integration point: llama-fpga's AXI DataMover (mm2s/s2mm)-based KV Cache
  read/write path

---

## 8. References

- TurboQuant: Online Vector Quantization with Near-optimal Distortion Rate
  ([arXiv:2504.19874](https://arxiv.org/abs/2504.19874))
- Oaken: Fast and Efficient LLM Serving with Online-Offline Hybrid KV Cache
  Quantization (ISCA 2025)

---

## 9. Environment & Usage

### Requirements

```
- Icarus Verilog (iverilog) 12.0+, or Vivado Simulator / QuestaSim
- Python 3.10+, numpy, scipy, pandas, matplotlib
- (For KV Cache extraction) transformers, accelerate, bitsandbytes, PyTorch (GPU recommended)
- (For baseline integration) Vivado 2024.x, Alveo U250 XDMA driver
```

### RTL Simulation

```bash
# Example: Centroid Sorter + LUT ROM unit test
iverilog -o sim_sorter tb_tq_centroid_sorter.v tq_lut_rom.v tq_centroid_sorter.v
./sim_sorter
```

### Generating the Golden Reference

```bash
python3 tq_feeder_golden_reference.py --all
# Or specify a single vector file
python3 tq_feeder_golden_reference.py --txt ./feeder_ip_test_vectors/d128_k_20.txt --out ./out
```

### Verifying RTL Results

`tq_rtl_verification.ipynb` compares the golden reference (`*.md` files or golden
script output) against RTL simulation dumps (`$display`/`$writememh` output).
idx values must match exactly; reconstructed values are judged PASS/FAIL against
an MSE/NRMSE threshold.

### Extracting Real KV Cache Data (Colab)

Running `extract_kv_cache_multi_d.ipynb` on Colab (GPU runtime) performs a real
forward pass on TinyLlama-1.1B (d=64), LLaMA2-7B (d=128), and Gemma-2B (d=256) to
extract KV Cache and save it as `.npy` files. LLaMA2-7B and Gemma-2B require
accepting the HuggingFace license for gated access.

---

## 10. Roadmap

- [x] Mathematical derivation of TurboQuant Algorithm 1 (RHT + 4-bit Lloyd-Max) and Python golden reference implementation
- [x] Quantization engine RTL implementation (L2 norm, FWHT, Centroid Sorter, Bit Packer)
- [x] Dequantization engine RTL implementation (Bit Unpacker, Centroid Transform integration, Dequant Multiplier)
- [x] Quantitative validation using real LLM KV Cache (D=64/128)
- [x] Hardware resource optimization (fixed-slot redesign)
- [ ] Root-cause and fix for the D=256 pipeline error (FWHT guard-bit suspected)
- [ ] U250 + llama-fpga baseline bring-up
- [ ] Feeder IP <-> baseline AXI path integration
- [ ] System-level performance measurement (tokens/s, max context length)
- [ ] Task-level accuracy validation (perplexity / zero-shot accuracy)
- [ ] Synthesis-based area/timing report (fixed-slot vs. barrel shifter comparison)

---

## 11. Team

| Role | Owner |
|---|---|
| RTL design (quantization/dequantization engines, LUT ROM, Bit Packer/Unpacker) | - |
| Memory controller optimization (ChampSim/Ramulator2-based simulation) | - |
| Testbench / verification (Python golden reference, RTL comparison) | - |

*(Fill in actual team member names.)*

---

## Directory Structure (example, adjust to match the actual repo layout)

```
.
├── rtl/
│   ├── quant/
│   │   ├── tq_lut_rom.v
│   │   ├── tq_centroid_sorter.v
│   │   ├── tq_bit_packer.v
│   │   └── fwht_d128_k128.v
│   └── dequant/
│       ├── tq_bit_unpacker.v
│       └── dequant_mul_128lane.v
├── golden_reference/
│   ├── tq_feeder_golden_reference.py
│   └── tq_golden_reference.ipynb
├── verification/
│   └── tq_rtl_verification.ipynb
├── data_extraction/
│   └── extract_kv_cache_multi_d.ipynb
└── README.md
```

---

## License

*(Specify the project license, e.g., MIT / Apache-2.0 / TBD.)*

## Acknowledgments

- [llama-fpga](https://github.com/adamgallas/llama-fpga) - baseline FPGA LLM accelerator
- Referenced public analyses from the TurboQuant community implementations
  (llama.cpp turbo3/turbo2, OnlyTerp/turboquant, and others).
