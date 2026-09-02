.. SPDX-License-Identifier: GPL-2.0-only

==========================================
Gigabyte GSA1 WMI SMBus driver (aorus-wmi)
==========================================

Overview
========

``aorus-wmi`` is an SMBus controller driver for Gigabyte motherboards
whose firmware exposes the ``\GSA1`` ACPI-WMI device. The firmware
implements two SMBus host controllers of the AMD FCH, and drives them
itself from the AML interpreter. This driver binds to the firmware's
``WMBB`` WMI method and exposes both hosts as standard Linux i2c
adapters.

The driver is a general SMBus transport, not a device driver for any
particular peripheral: everything attached to those buses becomes
reachable through the standard i2c subsystem. On the reference platform
this enables RGB memory control via OpenRGB and memory temperature/SPD
support via the in-kernel SPD clients: ``ee1004`` (DDR4 SPD EEPROMs)
binds on the memory bus, and the ``ee1004`` client itself instantiates
the ``jc42`` temperature sensors found on DDR3/DDR4 modules.
``spd5118`` (DDR5 modules) binds on DDR5 boards and is inert on DDR4.

No kernel command-line options or ACPI resource-check workarounds (such
as ``acpi_enforce_resources=lax``) are required: the driver requests no
I/O port resources and performs no direct hardware access. Every bus
transaction is performed by the firmware.

.. note::

   Throughout this document, statements about what the AML *does* are
   **observed firmware behaviour** — decompiled from the ACPI tables of
   the boards tested and verified live. Statements about what the driver
   *relies on* are the **required driver contract**. The two coincide on
   the boards tested; they are not guaranteed to coincide anywhere else.
   The DMI table in the driver is the compatibility boundary — the WMI
   GUID alone does not standardize this ABI across all Gigabyte boards.

Supported hardware
==================

The firmware pattern exists across Gigabyte's AM4/AM5 desktop range, but
ABI compatibility cannot be assumed from the presence of the WMI GUID
alone (the firmware exposes no version or capability register for the
SMBus interface). The driver therefore binds automatically only to
boards in its DMI allowlist, which lists boards on which the wire ABI
below has been verified:

* Gigabyte X570 AORUS XTREME (the board version string is not
  populated by the firmware, so revisions cannot be distinguished)

An unlisted board can be tried with ``aorus_wmi.force=1``; registration
then performs no bus traffic either way, and device discovery is left to
the normal client drivers.

WMI interface
=============

The firmware's ``_WDG`` table declares the WMI method object:

======================  =================================================
GUID                    ``DEADBEEF-2001-0000-00A0-C90629100000``
Object ID               ``BB``
Type                    WMI method (``WMBB``)
======================  =================================================

The driver evaluates ``WMBB(instance, function_code, argument_buffer)``
via ``wmidev_evaluate_method()``: the function code travels as the WMI
``method_id`` argument — it is **never** a field of the argument buffer.
The argument buffer selects the bus and carries the transaction
parameters.

The WMBB dispatch implements several unrelated function families (board
information, sleep/wake notifications, raw I/O, PCI configuration and
physical memory access as a service). This driver implements only the
SMBus transaction set; everything else is intentionally not exposed.
The SMBus function codes (observed from the decompiled AML):

===================  =========================================
``0x62`` / ``0x63``  Quick write / quick read (bus 0 only)
``0x65`` / ``0x66``  Send byte / receive byte (bus 0 only)
``0x67`` / ``0x68``  Byte-data write / read
``0x69`` / ``0x6A``  Word-data write / read
``0x6B``             SMBus block write (payload ≤ 32 B)
``0x5F``             Block read (status header + up to 256 B)
===================  =========================================

Request format
==============

The WMBB argument buffer, as overlaid by the AML on ``Arg2``. Field
offsets are firmware-fixed; the fields deliberately overlap.

================  ====  ==============================================
Offset            Size  Meaning
================  ====  ==============================================
``[0]``           1     Bus selector: ``0x02`` (host at I/O ``0x0B00``)
                        or ``0x03`` (host at ``0x0B20``)
``[1]``           1     Slave address, pre-shifted: 7-bit address << 1.
                        The AML sets the read bit itself for reads.
``[2]``           1     Command/register byte; send byte takes its
                        value here
``[3]``           1     Byte value; also low byte of the block length
``[4..5]``        2     Word value, little endian
``[3..6]``        4     Block length, little endian (dword)
``[7..]``         N     Block write payload (N = length dword)
================  ====  ==============================================

Buffers shorter than 7 bytes have been observed to break some function
codes, so the driver always sends at least 7 bytes. The word field sits
at offset **4**, not 3: a one-byte error here makes writes land on the
wrong device register while the firmware still reports success.

Response format
===============

All functions return their result through the WMI core, which unmarshals
the ACPI object into a linear byte buffer of at least the per-function
minimum length (see the transaction table above). A missing result, or
one shorter than that minimum, is a transport failure and maps to
``-EIO``. The core coerces non-buffer results (e.g. integers) to 4-byte
buffers, so a differently-typed result is not a failure.

* **Write transactions** (quick write, send byte, byte/word/block
  write): a dword that is ``1`` on success and ``0`` on failure
  (mapped to ``-ENXIO``).
* **Byte-data read** (``0x68``): dword value; ``0xFFFF`` means
  failure/NAK (``-ENXIO``). Known limitation: a device that legitimately
  returns the data value ``0xFFFF`` is indistinguishable from a NAK.
* **Word-data read** (``0x6A``): dword value; ``0xFFFFFFFF`` means
  failure/NAK (``-ENXIO``).
* **Quick read** (``0x63``) / **receive byte** (``0x66``): word value;
  ``0xFFFF`` means failure/NAK.
* **Block read** (``0x5F``): ``[0..1]`` status word, ``[2..3]`` count
  word, then ``count`` data bytes. Status word bit 15 set means error
  (``-EIO``, or ``-ENXIO`` if the low bits carry the SMBus DEV_ERR
  status). The count is additionally clamped by the driver to the SMBus
  maximum of 32 bytes and to the length of the returned buffer.

SMBus transaction types
=======================

The firmware implements SMBus transactions only; there is no raw I2C
message support. The driver registers ``smbus_xfer``-only adapters and
maps:

========================  ==================  =========================
i2c-core transaction      Function code       Notes
========================  ==================  =========================
``I2C_SMBUS_QUICK``       0x62 / 0x63         bus 0 only
``I2C_SMBUS_BYTE``        0x65 / 0x66         bus 0 only; send byte
                                              carries its value in the
                                              command field
``I2C_SMBUS_BYTE_DATA``   0x67 / 0x68
``I2C_SMBUS_WORD_DATA``   0x69 / 0x6A
``I2C_SMBUS_BLOCK_DATA``  0x6B write /        read clamped to 32 B
                          ``0x5F`` read
========================  ==================  =========================

Bus capabilities
================

Both buses implement byte-data, word-data and block transactions. Bus 0
additionally implements quick and send/receive byte transactions; the
firmware stubs these out on bus 1 (constant failure results), so the
driver rejects them with ``-EOPNOTSUPP`` both in ``functionality()``
and in ``smbus_xfer`` — they are never sent to the firmware.

Packet Error Checking (PEC) is not implemented: the decompiled AML
exposes no PEC controls (the host control register carries only the
transaction size bits, START and KILL), so ``I2C_FUNC_SMBUS_PEC`` is
deliberately not advertised.

Serialization
=============

Observed firmware behaviour: the ``WMBB`` method is declared
``Serialized``, so ACPICA serializes *all* invocations — both buses —
at the method boundary. Each bus additionally holds a per-bus ACPI mutex
(``SME0``/``SME3``), acquired per transaction, guarding the physical
hosts against any non-``WMBB`` accessor of the controller I/O regions
(an ACPI-table sweep found none).

Required driver contract: the kernel driver keeps one mutex per device,
not per adapter. It protects the driver's own state (result-buffer
handling, re-entry); it does **not** exist to serialize the firmware,
which serializes itself. Per-adapter kernel locking would deliver no
cross-bus parallelism for the WMI transport and is recorded as a
possible future optimization, to be reconsidered only after the complete
firmware call graph for these regions is verified.

Timeout/error semantics
=======================

Observed firmware behaviour — a single ``WMBB`` call is fully
synchronous: it performs the ready-wait, the transaction, the
completion poll and the status clear, then returns. Worst-case bounds
derived from the decompiled AML:

=====================  =============================================  ============
Condition             Cost                                           Source
=====================  =============================================  ============
Healthy transaction,  ~0.5–1 ms (a NAK sets DONE alongside DEV_ERR,  measured
or absent/NAK'ing     so it terminates on the first completion poll;
device                a full 120-address scan took ≈67 ms, ~0.56 ms
                      per absent address)
Stale status on entry up to +200 ms of 1 ms sleeps before the bus is  firmware bound
                      usable again
Hung bus (wedged      ~400–600 ms, then the firmware issues KILL     firmware bound
device, no DONE)
=====================  =============================================  ============

Bound composition: ready-wait ≤ 200 ms + busy poll ≤ 200 ms +
completion ≤ 200 ms ≈ **600 ms worst case**. Measured healthy path:
0.81 ms average, 1.0 ms maximum (1000 samples).

**No kernel-side early timeout.** ``wmidev_evaluate_method()`` is
synchronous and the AML always completes within the bounds above before
returning. The driver must never invent a timeout that returns early
while the AML may still be executing — that would allow a new
transaction to start against a bus the firmware considers busy. Any
slow-transaction logging happens strictly after completion (rate-limited
warning above 50 ms, which makes tier-3 wedged-bus behaviour visible
without noise from normal NAK probing).

Device map and i2cdetect caveats
================================

Real devices observed on the reference platform (bus 0):

======================  =============================================
Address                 Device
======================  =============================================
``0x18``–``0x1B``       ``jc42`` DDR3/DDR4 module temperature sensors
``0x36``/``0x37``       SPD page-select reservation (``ee1004``)
``0x50``–``0x53``       ``ee1004`` DDR4 SPD EEPROMs
``0x70``–``0x77``       ENE RGB memory controllers (address changes
                        between boots; observed at 0x66, 0x70–0x74
                        and 0x77 on different boots)
======================  =============================================

``i2cdetect`` scans return deterministic **false positives** on these
adapters: some addresses ACK although no device is present, because the
host controller or the firmware path acknowledges the transaction.

* quick-write scan reports: ``0x10, 0x13, 0x15, 0x3A, 0x4A, 0x68,
  0x6C, 0x70–0x74, 0x78, 0x7A, 0x7C, 0x7E``
* receive-byte scan adds: ``0x08, 0x30–0x35``
* bus 1 (no real devices on the reference board): ``0x4F, 0x51, 0x6A``

Treat scan results as a starting point and confirm devices by reading
their identity registers. Do not use bus 1 for detection: the firmware
stubs the quick and receive-byte transactions there.

Known limitations
=================

* **Sentinel collision (0xFFFF).** A byte-data read that legitimately
  returns the data value ``0xFFFF`` is indistinguishable from a NAK —
  both map to ``-ENXIO`` (see the response-format conventions above).
* **Scan false positives.** ``i2cdetect`` scans report deterministic
  ACKs for addresses with no device behind them (host/firmware
  artifacts); see the device map above for the per-scan address lists.
  Confirm devices by reading their identity registers.
* **Removal blocks on open descriptors.** Module removal (``modprobe
  -r``, unbind, or a DKMS upgrade) waits for any open ``/dev/i2c-N``
  file descriptor — standard i2c-core behaviour. Close OpenRGB (its
  autostart instance holds the descriptors indefinitely) before
  removing or upgrading the driver.

Hardware topology
=================

The two SMBus hosts are the I/O ranges of the AMD FCH SMBus PCI function
``1022:790b`` (bus ``00:14.0`` on the boards tested): the AML declares
``OperationRegion(SMBI, SystemIO, 0x0B00, 0x10)`` and
``OperationRegion(SMG0, SystemIO, 0x0B20, 0x20)``, and the PCI function
decodes no BARs on these boards — the fixed decodes are its only
hardware presence.

The driver therefore parents its i2c adapters at that PCI function.
This reflects the physical topology of the hardware; as a practical
consequence, userspace DRAM tooling (e.g. OpenRGB) discovers
DRAM-capable SMBus buses by the PCI ID of the adapter's sysfs parent,
and memory SPD/temperature client instantiation
(``i2c_register_spd_write_enable()``) then finds the modules: ``ee1004``
SPD EEPROMs — and the ``ee1004`` client in turn instantiates the
``jc42`` temperature sensors. ``spd5118`` covers DDR5 boards.

Why WMI rather than native i2c-piix4
====================================

The FCH SMBus controller is implemented using fixed I/O resources that
are permanently claimed by ACPI OperationRegions; the firmware exposes a
serialized WMI interface performing transactions against those same
hosts; using WMI preserves the firmware's existing arbitration, while
taking ownership from ACPI would bypass that arbitration and potentially
race other firmware consumers that are not visible to the OS.

Concretely: ``i2c-piix4`` cannot bind these ports on affected boards —
its ``request_region()`` for ``0x0B00`` fails against the AML
``OperationRegion(\GSA1.SMBI, SystemIO, 0x0B00, 0x10)`` conflict under
default strict ACPI resource enforcement (observed as a silent
``-ENODEV``). The conflict is structural: the OperationRegions are
registered when the ACPI namespace loads, independent of any OS driver.
Working around this would require a quirk overriding an AML-declared
region on DMI-matched boards, removing the WMI driver (two owners of one
bus), and accepting that AML and SMM access to those regions proceeds
outside the native driver's arbitration. Direct native access would
operate outside the synchronization mechanisms provided by the firmware
interface and cannot provide equivalent guarantees for firmware
consumers that are not visible to the OS.

The WMI path dissolves rather than suppresses the resource question: the
driver claims no I/O resources, so no ``acpi_enforce_resources``
conflict can arise, and no kernel command-line workaround is needed.

Relationship to the gigabyte-wmi driver
=======================================

The in-tree ``gigabyte-wmi`` driver (``drivers/platform/x86/``) binds to
the **same WMI GUID** as this driver
(``DEADBEEF-2001-0000-00A0-C90629100000`` — the ``WMBB`` method), so on
any board where the device is present, only one of the two drivers can
own it at a time. The drivers expose disjoint ABI surfaces (hwmon
attributes vs i2c adapters) and their firmware features do not co-occur
on currently known boards, but the GUID collision makes their
coexistence explicit design rather than accident:

=============================================  ==================================================
Scenario                                       Outcome
=============================================  ==================================================
aorus-wmi on a DMI-listed board                aorus-wmi binds the device (the WMI device is
                                               present — that is what aorus-wmi matches)
gigabyte-wmi probes first on a listed board    it binds, issues its temperature query (WMBB
                                               function ``0x125``), fails with ``-ENODEV``
                                               because this firmware's dispatch has no
                                               ``0x125`` branch, and **releases the device**;
                                               aorus-wmi's registration re-probes and binds
aorus-wmi probes first on a non-listed board   aorus-wmi refuses (DMI boundary); gigabyte-wmi
                                               binds and serves temperature monitoring
board with both dispatch branches              winner-takes-all — none known; covered by the
                                               coordination change below
=============================================  ==================================================

The failed-probe release behaviour was verified experimentally
(gigabyte-wmi bind-fail-release, followed by a successful aorus-wmi
bind and full client-chain recovery). Recovery relies on the later
driver's registration re-probing unbound devices — not on polling. A
small coordination change for ``gigabyte-wmi`` (yield its probe on
boards where the GSA1 SMBus driver owns the device) converts the
load-order race into a deterministic contract and accompanies the
upstream submission of this driver.

