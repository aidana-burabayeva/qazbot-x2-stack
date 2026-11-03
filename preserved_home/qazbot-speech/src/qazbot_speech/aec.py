# Ported unchanged from aec-demo-handoff (fdaf_aec.py, snapshot 2026-10-12) apart from this note. numpy only.
"""
Real-time-capable AEC: partitioned-block frequency-domain adaptive filter
(PBFDAF, overlap-save) with a Geigel-style double-talk detector.

Same idea as nlms_aec.py (predict the echo from the known reference, subtract
it, freeze learning while the human talks), but:

  - works on whole blocks with FFTs instead of a Python loop per sample, so it
    runs far faster than real time (the per-sample NLMS needs ~0.4 s of CPU
    per second of audio, which is too tight for a live audio callback);
  - covers a much longer echo tail (default 8 partitions x 512 = 4096 taps,
    ~170 ms at 24 kHz), which a laptop speaker/mic round trip needs because
    audio I/O latency alone is often 20-60 ms before any room reflection.

Interface: call `process(ref_block, mic_block)` with float arrays of exactly
`block_size` samples (ref = what was sent to the speaker, mic = what the
microphone captured over the same time span). Returns the cleaned block.
"""

from __future__ import annotations

import numpy as np


class FDAFEchoCanceller:
    def __init__(
        self,
        block_size: int = 512,
        partitions: int = 8,
        mu: float = 0.3,
        dt_threshold: float = 1.0,
        power_smoothing: float = 0.9,
        ref_gate: float = 1e-4,
        suppress: float = 0.3,
        dt_hold_blocks: int = 25,
    ) -> None:
        """
        block_size: samples per call; FFT size is 2 * block_size.
        partitions: filter length = partitions * block_size taps.
        mu: adaptation step (0 < mu <= 1).
        dt_threshold: double-talk is declared when the mic block RMS exceeds
            dt_threshold * (tracked echo gain) * (recent reference RMS).
        power_smoothing: smoothing of the per-bin reference power used to
            normalise the update.
        ref_gate: reference RMS below this means the robot is silent; the
            filter is not updated (nothing to learn from).
        suppress: residual-echo suppression. While the robot is talking and
            no double-talk is detected (echo only), the output is scaled by
            this gain (0.3 = -10 dB) to hide what the linear filter could not
            cancel. 1.0 disables it. Full gain is restored as soon as
            double-talk is detected (and for dt_hold_blocks blocks after), so
            a human barging in is passed through -- but a human quieter than
            the echo may be missed; set 1.0 to rule this out.
        """
        self.B = block_size
        self.P = partitions
        self.mu = mu
        self.dt_threshold = dt_threshold
        self.alpha = power_smoothing
        self.ref_gate = ref_gate

        nbins = self.B + 1
        self.W = np.zeros((self.P, nbins), dtype=np.complex128)   # filter partitions
        self.X = np.zeros((self.P, nbins), dtype=np.complex128)   # ref spectra history
        self.ref_prev = np.zeros(self.B)
        self.power = np.full(nbins, 1e-6)
        self.ref_rms_hist = np.zeros(self.P)
        self.echo_gain = 1.0          # tracked mic_rms / ref_rms in echo-only blocks
        self.double_talk = False      # state of the last block (for logging)
        self.suppress = suppress
        self.dt_hold_blocks = dt_hold_blocks
        self._dt_hold = 0
        self._gain = 1.0

    def process(self, ref: np.ndarray, mic: np.ndarray) -> np.ndarray:
        B = self.B
        assert len(ref) == B and len(mic) == B, f"need exactly {B} samples"
        ref = ref.astype(np.float64)
        mic = mic.astype(np.float64)

        # Spectrum of [previous block, current block] (overlap-save window).
        Xk = np.fft.rfft(np.concatenate([self.ref_prev, ref]))
        self.ref_prev = ref
        self.X = np.roll(self.X, 1, axis=0)
        self.X[0] = Xk

        # Predicted echo = sum over partitions of W_p * X_p; keep last B samples.
        echo_hat = np.fft.irfft((self.W * self.X).sum(axis=0))[B:]
        err = mic - echo_hat

        # --- double-talk / adaptation control (Geigel on block RMS) ---
        ref_rms = float(np.sqrt(np.mean(ref ** 2)))
        mic_rms = float(np.sqrt(np.mean(mic ** 2)))
        self.ref_rms_hist = np.roll(self.ref_rms_hist, 1)
        self.ref_rms_hist[0] = ref_rms
        recent_ref = float(self.ref_rms_hist.max())

        robot_active = recent_ref > self.ref_gate
        self.double_talk = mic_rms > self.dt_threshold * self.echo_gain * recent_ref
        adapt = robot_active and not self.double_talk

        if adapt:
            # Track the echo-path gain only while we believe it's echo-only.
            if ref_rms > self.ref_gate:
                g = mic_rms / ref_rms
                self.echo_gain = 0.95 * self.echo_gain + 0.05 * min(g, 4.0)

            inst = (np.abs(self.X) ** 2).sum(axis=0)
            self.power = self.alpha * self.power + (1 - self.alpha) * inst
            # Never normalise by less than the instantaneous power (a lagging
            # estimate at a speech onset makes the step overshoot and diverge),
            # and floor near-empty bins so they aren't amplified.
            norm = np.maximum(self.power, inst)
            norm = norm + 1e-2 * norm.mean() + 1e-8
            Ek = np.fft.rfft(np.concatenate([np.zeros(B), err]))
            G = np.conj(self.X) * (Ek / norm)                    # (P, nbins)
            g_time = np.fft.irfft(G, axis=1)
            g_time[:, B:] = 0.0                                    # gradient constraint
            self.W += self.mu * np.fft.rfft(g_time, axis=1)

        # --- residual echo suppression (simple gate with smooth gain ramp) ---
        if self.double_talk:
            self._dt_hold = self.dt_hold_blocks
        elif self._dt_hold > 0:
            self._dt_hold -= 1
        passthrough = self.double_talk or self._dt_hold > 0 or not robot_active
        target = 1.0 if passthrough else self.suppress
        ramp = np.linspace(self._gain, target, B, endpoint=False)
        self._gain = target
        return err * ramp
