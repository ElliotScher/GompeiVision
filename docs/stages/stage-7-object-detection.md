# Stage 7: Object detection (DPU) and localization

## Goal

Detect and localize game pieces on every module (color or mono sensor), for ObjectRecognition → ObjectLocalization → ObjectData, using AMD's prebuilt DPU accelerator.

## Deliverables

- `ml/`: dataset and training pipeline for a small single-shot detector, with INT8 quantization and compilation for the DPU using Vitis AI.
- [PL] DPU IP at the largest size Stage 0b found to fit next to the tag pipeline, and a [PL] downscaler on the FrameBus feeding it.
- [APU] inference service using the Vitis AI runtime, reading downscaled frames from the frame store.
- [APU] ground-plane object localization and an `ObjectData` topic with exposure timestamps, flagging pieces that are likely off the floor (single-camera range only holds for pieces on the floor).
- Training data from the modules' own recordings (Stage 3's dataset mode), and a comparison of the same model on OV9782 color against OV9281 mono.

## Exit criteria

- [ ] On-device results match the quantized model on the host.
- [ ] mAP on our validation set within a stated margin of the float model.
- [ ] ≥30 FPS on the target input size.
- [ ] Tag-path latency still meets its Stage 5 numbers with the DPU running flat out (memory contention test).

## Risks

- DPU size vs. remaining PL resources. Options: smaller DPU, smaller model, lower FPS.
- Vitis AI support for this chip family. Fallback, recorded here if used: CPU-only inference on the A53s at lower FPS and input size.
