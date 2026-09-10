# AUTOTUNE_SAFE_SWD_SESSION_VALIDATION

- Scope: ST-LINK/SWD session orchestration only
- Terminal states: `COMPLETE_LATCHED` / `ABORT_LATCHED`
- Generation ordering: uint32 wrap-aware delta
- Snapshot rule: odd/changing sequence or transient CRC mismatch retries
- Result rule: final magic is committed last; magic=0 is incomplete
- Failure category: `VALID`
- Failure code: `NONE`

```json
{
  "profile": "LOCAL_SESSION_SMOKE",
  "smoke_elf": "D:\\STM32cubemx\\Project\\ros\\build\\LocalSessionSmoke\\ros.elf",
  "debug_elf": "D:\\STM32cubemx\\Project\\ros\\build\\Debug\\ros.elf",
  "serial": "3E3703013212354D434B4E00",
  "gdb_host": "localhost",
  "gdb_port": 61234,
  "poll_ms": 50,
  "events": [
    "build",
    "programmer_exit",
    "gdb_attach"
  ],
  "boot_identity": {
    "commit_seq": 4,
    "profile": 1,
    "firmware_build_id": 1045280436,
    "boot_generation": 8,
    "boot_reason": 0,
    "boot_state": 2,
    "mailbox_version": 1,
    "boot_count": 8,
    "boot_crc32": 220159647
  },
  "armed_status": {
    "commit_seq": 4,
    "accepted_request_id": 1198297021,
    "accepted_session_id": 338287855,
    "active_experiment_id": 0,
    "state": 3,
    "boot_generation": 8,
    "session_generation": 1,
    "sample_generation": 0,
    "start_sample_generation": 0,
    "sample_count": 0,
    "expected_sample_count": 32,
    "last_rejected_request_id": 0,
    "last_reject_reason": 0,
    "result_ack_request_id": 0,
    "status_crc32": 2478368544
  },
  "running_status": {
    "commit_seq": 8,
    "accepted_request_id": 1198297022,
    "accepted_session_id": 338287855,
    "active_experiment_id": 37019533,
    "state": 4,
    "boot_generation": 8,
    "session_generation": 1,
    "sample_generation": 1,
    "start_sample_generation": 0,
    "sample_count": 1,
    "expected_sample_count": 32,
    "last_rejected_request_id": 0,
    "last_reject_reason": 0,
    "result_ack_request_id": 0,
    "status_crc32": 3264157430
  },
  "terminal_status": {
    "commit_seq": 72,
    "accepted_request_id": 1198297022,
    "accepted_session_id": 338287855,
    "active_experiment_id": 37019533,
    "state": 5,
    "boot_generation": 8,
    "session_generation": 1,
    "sample_generation": 32,
    "start_sample_generation": 0,
    "sample_count": 32,
    "expected_sample_count": 32,
    "last_rejected_request_id": 0,
    "last_reject_reason": 0,
    "result_ack_request_id": 0,
    "status_crc32": 626700609
  },
  "result": {
    "magic": 1381190740,
    "version": 1,
    "size": 68,
    "session_id": 338287855,
    "experiment_id": 37019533,
    "boot_generation": 8,
    "session_generation": 1,
    "start_sample_generation": 0,
    "first_sample_generation": 1,
    "last_sample_generation": 32,
    "sample_count": 32,
    "ring_start_index": 0,
    "ring_count": 32,
    "final_state": 5,
    "abort_reason": 0,
    "result_flags": 1,
    "ring_crc32": 2259740556,
    "result_crc32": 1619233047
  },
  "ring": [
    {
      "sample_generation": 1,
      "timestamp_ms": 2151,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 0,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 2,
      "timestamp_ms": 2161,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 1,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 3,
      "timestamp_ms": 2171,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 2,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 4,
      "timestamp_ms": 2181,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 3,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 5,
      "timestamp_ms": 2191,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 4,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 6,
      "timestamp_ms": 2201,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 5,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 7,
      "timestamp_ms": 2211,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 6,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 8,
      "timestamp_ms": 2221,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 7,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 9,
      "timestamp_ms": 2231,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 8,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 10,
      "timestamp_ms": 2241,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 9,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 11,
      "timestamp_ms": 2251,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 10,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 12,
      "timestamp_ms": 2261,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 11,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 13,
      "timestamp_ms": 2271,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 12,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 14,
      "timestamp_ms": 2281,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 13,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 15,
      "timestamp_ms": 2291,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 14,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 16,
      "timestamp_ms": 2301,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 15,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 17,
      "timestamp_ms": 2311,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 16,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 18,
      "timestamp_ms": 2321,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 17,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 19,
      "timestamp_ms": 2331,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 18,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 20,
      "timestamp_ms": 2341,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 19,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 21,
      "timestamp_ms": 2351,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 20,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 22,
      "timestamp_ms": 2361,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 21,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 23,
      "timestamp_ms": 2371,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 22,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 24,
      "timestamp_ms": 2381,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 23,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 25,
      "timestamp_ms": 2391,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 24,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 26,
      "timestamp_ms": 2401,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 25,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 27,
      "timestamp_ms": 2411,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 26,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 28,
      "timestamp_ms": 2421,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 27,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 29,
      "timestamp_ms": 2431,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 28,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 30,
      "timestamp_ms": 2441,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 29,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 31,
      "timestamp_ms": 2451,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 30,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    },
    {
      "sample_generation": 32,
      "timestamp_ms": 2461,
      "session_id": 338287855,
      "experiment_id": 37019533,
      "sample_index": 31,
      "target": 100,
      "actual": 100,
      "pwm": 0,
      "state": 4,
      "flags": 1
    }
  ],
  "debug_restore": {
    "status": "OK",
    "attempts": [
      {
        "attempt": 1,
        "record": {
          "argv": [
            "D:\\stm32cubeclt\\STM32CubeCLT_1.19.0\\STM32CubeProgrammer\\bin\\STM32_Programmer_CLI.exe",
            "-c",
            "port=SWD sn=3E3703013212354D434B4E00",
            "freq=4000",
            "-w",
            "D:\\STM32cubemx\\Project\\ros\\build\\Debug\\ros.elf",
            "-v",
            "-rst",
            "-run"
          ],
          "returncode": 0,
          "stdout": "      -------------------------------------------------------------------\n                       STM32CubeProgrammer v2.20.0                  \n      -------------------------------------------------------------------\n\nST-LINK SN  : 3E3703013212354D434B4E00\nST-LINK FW  : V2J46S7\nBoard       : --\nVoltage     : 3.19V\nSWD freq    : 4000 KHz\nConnect mode: Normal\nReset mode  : Software reset\nDevice ID   : 0x413\nRevision ID : --\nDevice name : STM32F405xx/F407xx/F415xx/F417xx\nFlash size  : 512 KBytes\nDevice type : MCU\nDevice CPU  : Cortex-M4\nBL Version  : 0x91\n\n\n\nOpening and parsing file: ros.elf\n\n\nMemory Programming ...\n  File          : ros.elf\n  Size          : 40.28 KB \n  Address       : 0x08000000\n\n\nErasing memory corresponding to segment 0:\nErasing internal memory sectors [0 2]\nDownload in Progress:\n北北北北北北北北北北北北北北北北北北北北北北北北北 0%\n圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹 100%\n\nFile download complete\nTime elapsed during download operation: 00:00:01.219\n\n\n\nVerifying...\n\n\nRead progress:\n北北北北北北北北北北北北北北北北北北北北北北北北北 50%\n圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹 100%\n\nTime elapsed during verifying operation: 00:00:00.261\n\n\nDownload verified successfully \n\n\n\nMCU Reset\n\nSoftware reset is performed\nCore run\n",
          "stderr": "",
          "started_at": "2026-09-02T09:31:56.363142+00:00",
          "exited_at": "2026-09-02T09:31:57.957286+00:00"
        }
      }
    ]
  },
  "smoke_elf_sha256": "834dada27e903e3cf424c0a53fc2e4cb78cd8b743fa28befcc4c5600b481607e",
  "debug_elf_sha256": "6b27bf3e3c6db09862eb7ee8f1c81c20a34b1571d431edc8b96dc0d46e5afe25",
  "gdb_server": {
    "argv": [
      "D:\\stm32cubeclt\\STM32CubeCLT_1.19.0\\STLink-gdb-server\\bin\\ST-LINK_gdbserver.exe",
      "--swd",
      "--attach",
      "--port-number",
      "61234",
      "--frequency",
      "4000",
      "--stm32cubeprogrammer-path",
      "D:\\stm32cubeclt\\STM32CubeCLT_1.19.0\\STM32CubeProgrammer\\bin",
      "--serial-number",
      "3E3703013212354D434B4E00"
    ],
    "output": [
      "",
      "",
      "STMicroelectronics ST-LINK GDB server. Version 7.11.0",
      "Copyright (c) 2025, STMicroelectronics. All rights reserved.",
      "",
      "Starting server with the following options:",
      "        Persistent Mode            : Disabled",
      "        Logging Level              : 1",
      "        Listen Port Number         : 61234",
      "        Status Refresh Delay       : 15s",
      "        Verbose Mode               : Disabled",
      "        SWD Debug                  : Enabled",
      "",
      "Waiting for debugger connection...",
      "Debugger connected",
      "Waiting for debugger connection..."
    ],
    "no_reset_attach": true
  },
  "generation_before_start": 0,
  "outcome": {
    "code": "OK",
    "category": "SUCCESS",
    "detail": ""
  },
  "commands": [
    {
      "argv": [
        "D:\\stm32cubeclt\\STM32CubeCLT_1.19.0\\STM32CubeProgrammer\\bin\\STM32_Programmer_CLI.exe",
        "-c",
        "port=SWD sn=3E3703013212354D434B4E00",
        "freq=4000",
        "-w",
        "D:\\STM32cubemx\\Project\\ros\\build\\LocalSessionSmoke\\ros.elf",
        "-v",
        "-rst",
        "-run"
      ],
      "returncode": 0,
      "stdout": "      -------------------------------------------------------------------\n                       STM32CubeProgrammer v2.20.0                  \n      -------------------------------------------------------------------\n\nST-LINK SN  : 3E3703013212354D434B4E00\nST-LINK FW  : V2J46S7\nBoard       : --\nVoltage     : 3.19V\nSWD freq    : 4000 KHz\nConnect mode: Normal\nReset mode  : Software reset\nDevice ID   : 0x413\nRevision ID : --\nDevice name : STM32F405xx/F407xx/F415xx/F417xx\nFlash size  : 512 KBytes\nDevice type : MCU\nDevice CPU  : Cortex-M4\nBL Version  : 0x91\n\n\n\nOpening and parsing file: ros.elf\n\n\nMemory Programming ...\n  File          : ros.elf\n  Size          : 7.95 KB \n  Address       : 0x08000000\n\n\nErasing memory corresponding to segment 0:\nErasing internal memory sector 0\nDownload in Progress:\n北北北北北北北北北北北北北北北北北北北北北北北北北 0%\n圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹 100%\n\nFile download complete\nTime elapsed during download operation: 00:00:00.413\n\n\n\nVerifying...\n\n\nRead progress:\n北北北北北北北北北北北北北北北北北北北北北北北北北 50%\n圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹 100%\n\nTime elapsed during verifying operation: 00:00:00.052\n\n\nDownload verified successfully \n\n\n\nMCU Reset\n\nSoftware reset is performed\nCore run\n",
      "stderr": "",
      "started_at": "2026-09-02T09:31:51.862239+00:00",
      "exited_at": "2026-09-02T09:31:52.452862+00:00"
    },
    {
      "argv": [
        "D:\\stm32cubeclt\\STM32CubeCLT_1.19.0\\STLink-gdb-server\\bin\\ST-LINK_gdbserver.exe",
        "--help"
      ],
      "returncode": 0,
      "stdout": "\nUSAGE: \n\n   D:\\stm32cubeclt\\STM32CubeCLT_1.19.0\\STLink-gdb-server\\bin\\ST-LINK_gdbser\n                                        ver.exe  [--semihost-console-port\n                                        <port number>] [--semihosting\n                                        <semihost level>] [--external-init]\n                                        [--pend-halt-timeout <Pending halt\n                                        timeout>] [--halt] [-c <config\n                                        file>] [-e] [-f <log file>] [-l\n                                        <log level>] [-p <port number>]\n                                        [-v] [-r <refresh delay>]\n                                        [--incremental] [-s] [-d] [-z <port\n                                        number>] [-a <cpu clock>] [-b <SWO\n                                        CLOCKDIV>] [-k] [-q] [-i <ST-LINK\n                                        S/N>] [-m <apID>] [-g] [-t]\n                                        [--erase-all] [--memory-map <device\n                                        id>] [--ext-memory-loaders] [-ei\n                                        <file_path>] ...  [-el <file_path>]\n                                        ...  [-cp <path>] [--temp-path\n                                        <path>] [--preserve-temps]\n                                        [--frequency <max freq kHz>]\n                                        [--licenses] [--] [--version] [-h]\n\n\nWhere: \n\n   --semihost-console-port <port number>\n     Port number for semihost console clients\n\n   --semihosting <semihost level>\n     Select semihosting mode\n\n   --external-init\n     Run Init() from external loader after reset\n\n   --pend-halt-timeout <Pending halt timeout>\n     Maximum time to wait for core to halt\n\n   --halt\n     Halt all cores during reset\n\n   -c <config file>,  --config-file <config file>\n     Read the config params from config file\n\n   -e,  --persistent\n     Enable persistent mode\n\n   -f <log file>,  --log-file <log file>\n     Path to log file\n\n   -l <log level>,  --log-level <log level>\n     Logging level between 0 to 31\n\n   -p <port number>,  --port-number <port number>\n     TCP port number for GDB client\n\n   -v,  --verbose\n     Turn ON verbose mode\n\n   -r <refresh delay>,  --refresh-delay <refresh delay>\n     Minimum delay in seconds for hardware status refresh\n\n   --incremental\n     Turn ON incremental flash programming\n\n   -s,  --verify\n     Turn ON flash download verify\n\n   -d,  --swd\n     Enable SWD debug mode\t[use for SWV]\n\n   -z <port number>,  --swo-port <port number>\n     SWO output port number\t[use for SWV]\n\n   -a <cpu clock>,  --cpu-clock <cpu clock>\n     CPU clock speed in Hz\t[use for SWV]\n\n   -b <SWO CLOCKDIV>,  --swo-clock-div <SWO CLOCKDIV>\n     Divide CPU clock by SWO CLOCKDIV\n\n   -k,  --initialize-reset\n     Initialize the device under reset condition\n\n   -q,  --debuggers\n     List serial number for connected ST-LINK devices\n\n   -i <ST-LINK S/N>,  --serial-number <ST-LINK S/N>\n     ST-LINK serial number\n\n   -m <apID>,  --apid <apID>\n     Select core on multi-core devices\n\n   -g,  --attach\n     Attach to running target\n\n   -t,  --shared\n     Shareable ST-LINK\t[using ST-LINK server]\n\n   --erase-all\n     Erase all memories\n\n   --memory-map <device id>\n     Show memory map for device id\n\n   --ext-memory-loaders\n     List of available external memory-loaders\n\n   -ei <file_path>,  --extload_init <file_path>  (accepted multiple times)\n     Custom external memory-loader with initialization after reset\n\n   -el <file_path>,  --extload <file_path>  (accepted multiple times)\n     Custom external memory-loader\n\n   -cp <path>,  --stm32cubeprogrammer-path <path>\n     Path to STM32CubeProgrammer installation\n\n   --temp-path <path>\n     Temporary files for starting a debug session are stored in the\n     provided path\n\n   --preserve-temps\n     Will not remove used temporary files\n\n   --frequency <max freq kHz>\n     Select com frequency in kHz\n\n   --licenses\n     List of used tools and licenses\n\n   --,  --ignore-rest\n     Ignores the rest of the labeled arguments following this flag\n\n   --version\n     Displays version information and exits\n\n   -h,  --help\n     Displays usage information and exits\n\n\n   ST-LINK GDB server\n\n",
      "stderr": "",
      "started_at": "2026-09-02T09:31:52.452862+00:00",
      "exited_at": "2026-09-02T09:31:52.466796+00:00"
    },
    {
      "argv": [
        "D:\\stm32cubeclt\\STM32CubeCLT_1.19.0\\STM32CubeProgrammer\\bin\\STM32_Programmer_CLI.exe",
        "-c",
        "port=SWD sn=3E3703013212354D434B4E00",
        "freq=4000",
        "-w",
        "D:\\STM32cubemx\\Project\\ros\\build\\Debug\\ros.elf",
        "-v",
        "-rst",
        "-run"
      ],
      "returncode": 0,
      "stdout": "      -------------------------------------------------------------------\n                       STM32CubeProgrammer v2.20.0                  \n      -------------------------------------------------------------------\n\nST-LINK SN  : 3E3703013212354D434B4E00\nST-LINK FW  : V2J46S7\nBoard       : --\nVoltage     : 3.19V\nSWD freq    : 4000 KHz\nConnect mode: Normal\nReset mode  : Software reset\nDevice ID   : 0x413\nRevision ID : --\nDevice name : STM32F405xx/F407xx/F415xx/F417xx\nFlash size  : 512 KBytes\nDevice type : MCU\nDevice CPU  : Cortex-M4\nBL Version  : 0x91\n\n\n\nOpening and parsing file: ros.elf\n\n\nMemory Programming ...\n  File          : ros.elf\n  Size          : 40.28 KB \n  Address       : 0x08000000\n\n\nErasing memory corresponding to segment 0:\nErasing internal memory sectors [0 2]\nDownload in Progress:\n北北北北北北北北北北北北北北北北北北北北北北北北北 0%\n圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹 100%\n\nFile download complete\nTime elapsed during download operation: 00:00:01.219\n\n\n\nVerifying...\n\n\nRead progress:\n北北北北北北北北北北北北北北北北北北北北北北北北北 50%\n圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹圹 100%\n\nTime elapsed during verifying operation: 00:00:00.261\n\n\nDownload verified successfully \n\n\n\nMCU Reset\n\nSoftware reset is performed\nCore run\n",
      "stderr": "",
      "started_at": "2026-09-02T09:31:56.363142+00:00",
      "exited_at": "2026-09-02T09:31:57.957286+00:00"
    }
  ],
  "created_at": "2026-09-02T09:31:57.957286+00:00"
}
```
