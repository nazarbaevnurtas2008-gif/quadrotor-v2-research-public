# Pre-execution correction notice — final condition seed (2026-10-17)

**Written BEFORE any final condition draws.** This is a provenance correction, NOT a controller, model, endpoint, or analysis change.

The frozen `FROZEN_FINAL_PROTOCOL.json` contains an internal inconsistency: `randomization_and_independence.conditions_seed` = `7122826376398605640` but `final_future_seed` = `4229883130818374818`.

The source `build_freeze_package.py` assigns both fields the SAME seed derived from the SHA-256 of `SEED_COMMITMENT.json`, while `future_final_runner.py` explicitly draws final conditions from `final_future_seed`. The SHA-256 commitment `3ab3904bedd8aca22957292fb3aba4f49430b13afd992915732c40a15c1c062b` unambiguously derives `4229883130818374818` from its first 16 hexadecimal digits. Thus the `conditions_seed` value `7122826376398605640` is a stale transcription error.

**Before opening any final data, the declared correction is:** the authoritative condition seed is `4229883130818374818`, matching both the deterministic hash commitment and the sealed final runner. The original frozen protocol and frozen controller source are NOT edited. Keep the discrepancy visible in reporting, and use this notice alongside the original manifest. No seed or setting may be changed after the first final output appears.

Runner checksum (pre-execution): `4d6720bf78ea552a2034b1b1210163501202cb0a60709d6c5c72b51f812e332a`.
Frozen protocol checksum (pre-execution): `14bdb7ba31010e03f9f90ce3528927a511f8721a0cb4cd7df96935a17b81abd5`.

Limitations: this clarification after initial protocol generation, before final outcomes, should be declared in the V2 manuscript.
