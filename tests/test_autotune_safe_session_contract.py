import struct
import unittest

import tools.pid.local_swd_runner as runner


STATUS_SIZE = 68
RESULT_SIZE = 68
STATUS_MAGIC = 0x53544154
FINAL_MAGIC = 0x52534C54
BOOT_MAGIC = 0x41544249


def _status_bytes(*, state, commit_seq=2, session_id=11, experiment_id=22,
                  boot_generation=3, session_generation=4,
                  sample_generation=0, start_sample_generation=0,
                  sample_count=0, expected_sample_count=32,
                  result_ack_request_id=0, crc_override=None):
    data = bytearray(STATUS_SIZE)
    struct.pack_into(
        "<IIHHIIIIIIIIIIIII",
        data,
        0,
        commit_seq,
        STATUS_MAGIC,
        1,
        STATUS_SIZE,
        0,
        session_id,
        experiment_id,
        state,
        boot_generation,
        session_generation,
        sample_generation,
        start_sample_generation,
        sample_count,
        expected_sample_count,
        0,
        0,
        result_ack_request_id,
    )
    crc = runner.crc32_ieee(bytes(data[4:64]))
    struct.pack_into("<I", data, 64, crc if crc_override is None else crc_override)
    return bytes(data)


def _result_bytes(*, magic=FINAL_MAGIC, session_id=11, experiment_id=22,
                  boot_generation=3, session_generation=4,
                  start_sample_generation=10, first_sample_generation=11,
                  last_sample_generation=42, sample_count=32,
                  final_state=None, result_crc=None):
    if final_state is None:
        final_state = runner.SESSION_COMPLETE_LATCHED
    data = bytearray(RESULT_SIZE)
    struct.pack_into(
        "<IHHIIIIIIIIIIIIIII",
        data,
        0,
        magic,
        1,
        RESULT_SIZE,
        session_id,
        experiment_id,
        boot_generation,
        session_generation,
        start_sample_generation,
        first_sample_generation,
        last_sample_generation,
        sample_count,
        0,
        sample_count,
        final_state,
        0,
        1,
        0,
        0,
    )
    if result_crc is None:
        crc = runner.crc32_ieee(bytes(data[:60]))
    else:
        crc = result_crc
    struct.pack_into("<I", data, 64, crc)
    return bytes(data)


def _boot_identity_bytes(*, commit_seq=2, profile=1, build_id=99,
                         boot_generation=3, state=runner.BOOT_READY):
    data = bytearray(44)
    struct.pack_into(
        "<IIHHIIIIIII", data, 0, commit_seq, BOOT_MAGIC, 1, 44,
        profile, build_id, boot_generation, 1, state, 1, 1,
    )
    struct.pack_into("<I", data, 40, runner.crc32_ieee(bytes(data[4:40])))
    return bytes(data)


class AutotuneSafeSessionContractTest(unittest.TestCase):
    def test_boot_identity_seqlock_and_crc_are_required_for_ready(self):
        identity = runner.decode_boot_identity(_boot_identity_bytes())
        self.assertEqual(identity["boot_state"], runner.BOOT_READY)
        self.assertIsNone(
            runner.read_stable_boot_identity([_boot_identity_bytes(commit_seq=3)])
        )

    def test_latched_complete_survives_delayed_poll(self):
        snapshot = runner.decode_status_snapshot(
            _status_bytes(state=runner.SESSION_COMPLETE_LATCHED,
                          sample_generation=32, sample_count=32)
        )
        self.assertEqual(snapshot["state"], runner.SESSION_COMPLETE_LATCHED)

    def test_latched_abort_survives_delayed_poll(self):
        snapshot = runner.decode_status_snapshot(
            _status_bytes(state=runner.SESSION_ABORT_LATCHED)
        )
        self.assertEqual(snapshot["state"], runner.SESSION_ABORT_LATCHED)

    def test_start_accepts_first_sample_before_running_read(self):
        before = 0xFFFFFFFE
        snapshot = runner.decode_status_snapshot(
            _status_bytes(state=runner.SESSION_RUNNING,
                          sample_generation=0,
                          start_sample_generation=before)
        )
        self.assertTrue(
            runner.validate_running_handshake(snapshot, before, 11, 22)
        )

    def test_seqlock_snapshot_rejects_torn_status(self):
        torn = _status_bytes(state=runner.SESSION_RUNNING, commit_seq=3)
        self.assertIsNone(runner.read_stable_snapshot([torn]))

    def test_crc_retry_accepts_later_stable_snapshot(self):
        bad = _status_bytes(state=runner.SESSION_RUNNING, crc_override=0)
        good = _status_bytes(state=runner.SESSION_RUNNING)
        self.assertEqual(
            runner.read_stable_snapshot([bad, good], retry_budget=2)["state"],
            runner.SESSION_RUNNING,
        )

    def test_result_magic_last_rejects_half_write(self):
        result = _result_bytes(magic=0)
        self.assertEqual(
            runner.validate_result(result, {
                "build_id": 99,
                "boot_generation": 3,
                "session_generation": 4,
                "session_id": 11,
                "experiment_id": 22,
            }),
            "RESULT_COMMIT_INCOMPLETE",
        )

    def test_generation_wrap_is_forward(self):
        self.assertEqual(runner.generation_delta_u32(0, 0xFFFFFFFF), 1)
        self.assertTrue(runner.generation_is_forward(0, 0xFFFFFFFF))

    def test_por_generation_one_does_not_accept_old_joint_identity(self):
        old = {"build_id": 7, "boot_generation": 9,
               "session_generation": 2, "session_id": 11,
               "experiment_id": 22}
        new = dict(old, boot_generation=1)
        self.assertFalse(runner.validate_joint_identity(new, old))

    def test_each_joint_identity_component_is_required(self):
        expected = {"build_id": 7, "boot_generation": 9,
                    "session_generation": 2, "session_id": 11,
                    "experiment_id": 22}
        for field in expected:
            actual = dict(expected, **{field: expected[field] + 1})
            self.assertFalse(
                runner.validate_joint_identity(actual, expected), field
            )


if __name__ == "__main__":
    unittest.main()
