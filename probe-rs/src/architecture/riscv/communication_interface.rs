//! Debug Module Communication
//!
//! This module implements communication with a
//! Debug Module, as described in the RISCV debug
//! specification v0.13.2 .

use super::{
    dtm::{DmiOperation, DmiOperationStatus, Dtm},
    register, Dmcontrol, Dmstatus,
};
use crate::DebugProbeError;
use crate::{
    architecture::riscv::*,
    probe::{CommandResult, DeferredResultIndex},
};
use crate::{MemoryInterface, Probe};

use crate::{probe::JTAGAccess, Error as ProbeRsError, RegisterId};

use crate::memory::valid_32_address;

use bitfield::bitfield;
use std::{
    collections::HashMap,
    time::{Duration, Instant},
};

/// Something error occurered when working with the RISC-V core.
#[derive(thiserror::Error, Debug)]
pub enum RiscvError {
    /// An error during read/write of the DMI register happened.
    #[error("Error during read/write to the DMI register: {0:?}")]
    DmiTransfer(DmiOperationStatus),
    /// An error with operating the debug probe occurred.
    #[error("Debug Probe Error")]
    DebugProbe(#[from] DebugProbeError),
    /// A timeout occurred during JTAG register access.
    #[error("Timeout during JTAG register access.")]
    Timeout,
    /// An error occurred during the execution of an abstract command.
    #[error("Error occurred during execution of an abstract command: {0:?}")]
    AbstractCommand(AbstractCommandErrorKind),
    /// The request for reset, resume or halt was not acknowledged.
    #[error("The core did not acknowledge a request for reset, resume or halt")]
    RequestNotAcknowledged,
    /// This debug transport module (DTM) version is currently not supported.
    #[error("The version '{0}' of the debug transport module (DTM) is currently not supported.")]
    UnsupportedDebugTransportModuleVersion(u8),
    /// This version of the debug module is not supported.
    #[error("The version '{0:?}' of the debug module is currently not supported.")]
    UnsupportedDebugModuleVersion(DebugModuleVersion),
    /// The given program buffer register is not supported.
    #[error("Program buffer register '{0}' is currently not supported.")]
    UnsupportedProgramBufferRegister(usize),
    /// The program buffer is too small for the supplied program.
    #[error("Program buffer is too small for supplied program.")]
    ProgramBufferTooSmall,
    /// Memory width larger than 32 bits is not supported yet.
    #[error("Memory width larger than 32 bits is not supported yet.")]
    UnsupportedBusAccessWidth(RiscvBusAccess),
    /// An error during system bus access occurred.
    #[error("Error using system bus")]
    SystemBusAccess,
    /// The given trigger type is not available for the address breakpoint.
    #[error("Unexpected trigger type {0} for address breakpoint.")]
    UnexpectedTriggerType(u32),
}

impl From<RiscvError> for ProbeRsError {
    fn from(err: RiscvError) -> Self {
        match err {
            RiscvError::DebugProbe(e) => e.into(),
            other => ProbeRsError::ArchitectureSpecific(Box::new(other)),
        }
    }
}

/// Errors which can occur while executing an abstract command.
#[derive(Debug)]
pub enum AbstractCommandErrorKind {
    /// No error happened.
    None = 0,
    /// An abstract command was executing
    /// while command, abstractcs, or abstractauto
    /// was written, or when one of the data or progbuf
    /// registers was read or written. This status is only
    /// written if cmderr contains 0.
    Busy = 1,
    /// The requested command is not supported, reg
    NotSupported = 2,
    /// An exception occurred while executing the command (e.g. while executing the Program Buffer).
    Exception = 3,
    /// The abstract command couldn’t
    /// execute because the hart wasn’t in the required
    /// state (running/halted), or unavailable.
    HaltResume = 4,
    /// The abstract command failed due to a
    /// bus error (e.g. alignment, access size, or timeout).
    Bus = 5,
    /// A reserved code. Should not occur.
    _Reserved = 6,
    /// The command failed for another reason.
    Other = 7,
}

impl AbstractCommandErrorKind {
    fn parse(value: u8) -> Self {
        use AbstractCommandErrorKind::*;

        match value {
            0 => None,
            1 => Busy,
            2 => NotSupported,
            3 => Exception,
            4 => HaltResume,
            5 => Bus,
            6 => _Reserved,
            7 => Other,
            _ => panic!("cmderr is a 3 bit value, values higher than 7 should not occur."),
        }
    }
}

/// List of all debug module versions.
///
/// The version of the debug module can be read from the version field of the `dmstatus`
/// register.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum DebugModuleVersion {
    /// There is no debug module present.
    NoModule,
    /// The debug module conforms to the version 0.11 of the RISCV Debug Specification.
    Version0_11,
    /// The debug module conforms to the version 0.13 of the RISCV Debug Specification.
    Version0_13,
    /// The debug module is present, but does not conform to any available version of the RISCV Debug Specification.
    NonConforming,
    /// Unknown debug module version.
    Unknown(u8),
}

impl From<u8> for DebugModuleVersion {
    fn from(raw: u8) -> Self {
        match raw {
            0 => DebugModuleVersion::NoModule,
            1 => DebugModuleVersion::Version0_11,
            2 => DebugModuleVersion::Version0_13,
            15 => DebugModuleVersion::NonConforming,
            other => DebugModuleVersion::Unknown(other),
        }
    }
}

#[derive(Copy, Clone, Debug)]
struct CoreRegisterAbstractCmdSupport(u8);

impl CoreRegisterAbstractCmdSupport {
    const READ: Self = Self(1 << 0);
    const WRITE: Self = Self(1 << 1);
    const BOTH: Self = Self(Self::READ.0 | Self::WRITE.0);

    fn supports(&self, o: Self) -> bool {
        self.0 & o.0 == o.0
    }

    fn unset(&mut self, o: Self) {
        self.0 &= !(o.0);
    }
}

/// A state to carry all the state data across multiple core switches in a session.
#[derive(Debug)]
pub struct RiscvCommunicationInterfaceState {
    /// Debug specification version
    debug_version: DebugModuleVersion,

    /// Size of the program buffer, in 32-bit words
    progbuf_size: u8,

    /// Cache for the program buffer.
    progbuf_cache: [u32; 16],

    /// Implicit `ebreak` instruction is present after the
    /// the program buffer.
    implicit_ebreak: bool,

    /// Number of data registers for abstract commands
    data_register_count: u8,

    nscratch: u8,

    supports_autoexec: bool,

    /// Pointer to the configuration string
    confstrptr: Option<u128>,

    /// Width of the hartsel register
    hartsellen: u8,

    /// Number of harts
    num_harts: u32,

    memory_access_info: HashMap<RiscvBusAccess, MemoryAccessMethod>,

    /// describes, if the given register can be read / written with an
    /// abstract command
    abstract_cmd_register_info: HashMap<RegisterId, CoreRegisterAbstractCmdSupport>,
}

/// Timeout for RISCV operations.
const RISCV_TIMEOUT: Duration = Duration::from_secs(5);

impl RiscvCommunicationInterfaceState {
    /// Create a new interface state.
    pub fn new() -> Self {
        RiscvCommunicationInterfaceState {
            // Set to the minimum here, will be set to the correct value below
            progbuf_size: 0,
            progbuf_cache: [0u32; 16],

            debug_version: DebugModuleVersion::NonConforming,

            // Assume the implicit ebreak is not present
            implicit_ebreak: false,

            // Set to the minimum here, will be set to the correct value below
            data_register_count: 1,

            nscratch: 0,

            supports_autoexec: false,

            confstrptr: None,

            // Assume maximum value, will be determined exactly alter.
            hartsellen: 20,

            // We assume only a singe hart exisits initially
            num_harts: 1,

            memory_access_info: HashMap::new(),

            abstract_cmd_register_info: HashMap::new(),
        }
    }

    /// Get the memory access method which should be used for an
    /// access with the specified width.
    fn memory_access_method(&mut self, access_width: RiscvBusAccess) -> MemoryAccessMethod {
        *self
            .memory_access_info
            .entry(access_width)
            .or_insert(MemoryAccessMethod::ProgramBuffer)
    }
}

impl Default for RiscvCommunicationInterfaceState {
    fn default() -> Self {
        Self::new()
    }
}

/// A interface that implements controls for RISC-V cores.
#[derive(Debug)]
pub struct RiscvCommunicationInterface {
    /// The Debug Transport Module (DTM) is used to
    /// communicate with the Debug Module on the target chip.
    dtm: Dtm,
    state: RiscvCommunicationInterfaceState,
}

impl<'probe> RiscvCommunicationInterface {
    /// Creates a new RISC-V communication interface with a given probe driver.
    pub fn new(probe: Box<dyn JTAGAccess>) -> Result<Self, (Box<dyn JTAGAccess>, DebugProbeError)> {
        let state = RiscvCommunicationInterfaceState::new();
        let dtm = Dtm::new(probe).map_err(|(probe, e)| match e {
            RiscvError::DebugProbe(err) => (probe, err),
            other_error => (
                probe,
                DebugProbeError::ArchitectureSpecific(Box::new(other_error)),
            ),
        })?;

        let mut s = Self { dtm, state };

        if let Err(err) = s.enter_debug_mode() {
            return Err((s.dtm.probe, DebugProbeError::from(anyhow!(err))));
        }

        Ok(s)
    }

    /// Deassert the target reset.
    pub fn target_reset_deassert(&mut self) -> Result<(), DebugProbeError> {
        self.dtm.target_reset_deassert()
    }

    /// Read the targets IDCODE.
    pub fn read_idcode(&mut self) -> Result<u32, DebugProbeError> {
        self.dtm.read_idcode()
    }

    fn enter_debug_mode(&mut self) -> Result<(), RiscvError> {
        // We need a jtag interface

        log::debug!("Building RISCV interface");

        // Reset error bits from previous connections
        self.dtm.reset()?;

        // read the  version of the debug module
        let status: Dmstatus = self.read_dm_register()?;

        self.state.debug_version = DebugModuleVersion::from(status.version() as u8);

        // Only version of 0.13 of the debug specification is currently supported.
        if self.state.debug_version != DebugModuleVersion::Version0_13 {
            return Err(RiscvError::UnsupportedDebugModuleVersion(
                self.state.debug_version,
            ));
        }

        self.state.implicit_ebreak = status.impebreak();

        // check if the configuration string pointer is valid, and retrieve it, if valid
        self.state.confstrptr = if status.confstrptrvalid() {
            let confstrptr_0: Confstrptr0 = self.read_dm_register()?;
            let confstrptr_1: Confstrptr1 = self.read_dm_register()?;
            let confstrptr_2: Confstrptr2 = self.read_dm_register()?;
            let confstrptr_3: Confstrptr3 = self.read_dm_register()?;

            let confstrptr = (u32::from(confstrptr_0) as u128)
                | (u32::from(confstrptr_1) as u128) << 8
                | (u32::from(confstrptr_2) as u128) << 16
                | (u32::from(confstrptr_3) as u128) << 32;

            Some(confstrptr)
        } else {
            None
        };

        log::debug!("dmstatus: {:?}", status);

        // enable the debug module
        let mut control = Dmcontrol(0);
        control.set_dmactive(true);
        self.write_dm_register(control)?;

        // Select all harts to determine the width
        // of the hartsel register.
        control.set_hartsel(0xffff_ffff);

        self.write_dm_register(control)?;

        let control: Dmcontrol = self.read_dm_register()?;

        self.state.hartsellen = control.hartsel().count_ones() as u8;

        log::debug!("HARTSELLEN: {}", self.state.hartsellen);

        // Determine number of harts

        let max_hart_index = 2u32.pow(self.state.hartsellen as u32);

        let mut num_harts = 1;

        // Hart 0 exists on every chip
        for hart_index in 1..max_hart_index {
            let mut control = Dmcontrol(0);
            control.set_dmactive(true);
            control.set_hartsel(hart_index);

            self.write_dm_register(control)?;

            // Check if the current hart exists
            let status: Dmstatus = self.read_dm_register()?;

            if status.anynonexistent() {
                break;
            }

            num_harts += 1;
        }

        log::debug!("Number of harts: {}", num_harts);

        self.state.num_harts = num_harts;

        // Select hart 0 again
        let mut control = Dmcontrol(0);
        control.set_hartsel(0);
        control.set_dmactive(true);

        self.write_dm_register(control)?;

        // determine size of the program buffer, and number of data
        // registers for abstract commands
        let abstractcs: Abstractcs = self.read_dm_register()?;

        self.state.progbuf_size = abstractcs.progbufsize() as u8;
        log::debug!("Program buffer size: {}", self.state.progbuf_size);

        self.state.data_register_count = abstractcs.datacount() as u8;
        log::debug!(
            "Number of data registers: {}",
            self.state.data_register_count
        );

        // determine more information about hart
        let hartinfo: Hartinfo = self.read_dm_register()?;

        self.state.nscratch = hartinfo.nscratch() as u8;
        log::debug!("Number of dscratch registers: {}", self.state.nscratch);

        // determine if autoexec works
        let mut abstractauto = Abstractauto(0);
        abstractauto.set_autoexecprogbuf(2u32.pow(self.state.progbuf_size as u32) - 1);
        abstractauto.set_autoexecdata(2u32.pow(self.state.data_register_count as u32) - 1);

        self.write_dm_register(abstractauto)?;

        let abstractauto_readback: Abstractauto = self.read_dm_register()?;

        self.state.supports_autoexec = abstractauto_readback == abstractauto;
        log::debug!("Support for autoexec: {}", self.state.supports_autoexec);

        // clear abstractauto
        abstractauto = Abstractauto(0);
        self.write_dm_register(abstractauto)?;

        // determine support system bus access
        let sbcs = self.read_dm_register::<Sbcs>()?;

        // Only version 1 is supported, this means that
        // the system bus access conforms to the debug
        // specification 13.2.
        if sbcs.sbversion() == 1 {
            // When possible, we use system bus access for memory access

            if sbcs.sbaccess8() {
                self.state
                    .memory_access_info
                    .insert(RiscvBusAccess::A8, MemoryAccessMethod::SystemBus);
            }

            if sbcs.sbaccess16() {
                self.state
                    .memory_access_info
                    .insert(RiscvBusAccess::A16, MemoryAccessMethod::SystemBus);
            }

            if sbcs.sbaccess32() {
                self.state
                    .memory_access_info
                    .insert(RiscvBusAccess::A32, MemoryAccessMethod::SystemBus);
            }

            if sbcs.sbaccess64() {
                self.state
                    .memory_access_info
                    .insert(RiscvBusAccess::A64, MemoryAccessMethod::SystemBus);
            }

            if sbcs.sbaccess128() {
                self.state
                    .memory_access_info
                    .insert(RiscvBusAccess::A128, MemoryAccessMethod::SystemBus);
            }
        } else {
            log::debug!(
                "System bus interface version {} is not supported.",
                sbcs.sbversion()
            );
        }

        Ok(())
    }

    pub(super) fn read_dm_register<R: DebugRegister>(&mut self) -> Result<R, RiscvError> {
        log::debug!("Reading DM register '{}' at {:#010x}", R::NAME, R::ADDRESS);

        let register_value = self.read_dm_register_untyped(R::ADDRESS as u64)?.into();

        log::debug!(
            "Read DM register '{}' at {:#010x} = {:x?}",
            R::NAME,
            R::ADDRESS,
            register_value
        );

        Ok(register_value)
    }

    /// Read from a DM register
    ///
    /// Use the [`read_dm_register`] function if possible.
    fn read_dm_register_untyped(&mut self, address: u64) -> Result<u32, RiscvError> {
        // Prepare the read by sending a read request with the register address
        self.dtm
            .dmi_register_access_with_timeout(address, 0, DmiOperation::Read, RISCV_TIMEOUT)?;

        // Read back the response from the previous request.
        self.dtm
            .dmi_register_access_with_timeout(0, 0, DmiOperation::NoOp, RISCV_TIMEOUT)
    }

    pub(super) fn write_dm_register<R: DebugRegister>(
        &mut self,
        register: R,
    ) -> Result<(), RiscvError> {
        // write write command to dmi register

        log::debug!(
            "Write DM register '{}' at {:#010x} = {:x?}",
            R::NAME,
            R::ADDRESS,
            register
        );

        self.write_dm_register_untyped(R::ADDRESS as u64, register.into())
    }

    /// Write to a DM register
    ///
    /// Use the [`write_dm_register`] function if possible.
    fn write_dm_register_untyped(&mut self, address: u64, value: u32) -> Result<(), RiscvError> {
        self.dtm.dmi_register_access_with_timeout(
            address,
            value,
            DmiOperation::Write,
            RISCV_TIMEOUT,
        )?;

        Ok(())
    }

    fn write_progbuf(&mut self, index: usize, value: u32) -> Result<(), RiscvError> {
        match index {
            0 => self.write_dm_register(Progbuf0(value)),
            1 => self.write_dm_register(Progbuf1(value)),
            2 => self.write_dm_register(Progbuf2(value)),
            3 => self.write_dm_register(Progbuf3(value)),
            4 => self.write_dm_register(Progbuf4(value)),
            5 => self.write_dm_register(Progbuf5(value)),
            6 => self.write_dm_register(Progbuf6(value)),
            7 => self.write_dm_register(Progbuf7(value)),
            8 => self.write_dm_register(Progbuf8(value)),
            9 => self.write_dm_register(Progbuf9(value)),
            10 => self.write_dm_register(Progbuf10(value)),
            11 => self.write_dm_register(Progbuf11(value)),
            12 => self.write_dm_register(Progbuf12(value)),
            13 => self.write_dm_register(Progbuf13(value)),
            14 => self.write_dm_register(Progbuf14(value)),
            15 => self.write_dm_register(Progbuf15(value)),
            e => Err(RiscvError::UnsupportedProgramBufferRegister(e)),
        }
    }

    pub(crate) fn setup_program_buffer(&mut self, data: &[u32]) -> Result<(), RiscvError> {
        let required_len = if self.state.implicit_ebreak {
            data.len()
        } else {
            data.len() + 1
        };

        if required_len > self.state.progbuf_size as usize {
            return Err(RiscvError::ProgramBufferTooSmall);
        }

        if data == &self.state.progbuf_cache[..data.len()] {
            // Check if we actually have to write the program buffer
            log::debug!("Program buffer is up-to-date, skipping write.");
            return Ok(());
        }

        for (index, word) in data.iter().enumerate() {
            self.write_progbuf(index, *word)?;
        }

        // Add manual ebreak if necessary.
        //
        // This is necessary when we either don't need the full program buffer,
        // or if there is no implict ebreak after the last program buffer word.
        if !self.state.implicit_ebreak || data.len() < self.state.progbuf_size as usize {
            self.write_progbuf(data.len(), assembly::EBREAK)?;
        }

        // Update the cache
        self.state.progbuf_cache[..data.len()].copy_from_slice(data);

        Ok(())
    }

    /// Perform a single read from a memory location, using system bus access.
    fn perform_memory_read_sysbus<V: RiscvValue>(&mut self, address: u32) -> Result<V, RiscvError> {
        let mut sbcs = Sbcs(0);

        sbcs.set_sbaccess(V::WIDTH as u32);
        sbcs.set_sbreadonaddr(true);

        self.write_dm_register(sbcs)?;

        self.write_dm_register(Sbaddress0(address))?;
        let data = self.read_large_dtm_register::<V, Sbdata>()?;

        // Check that the read was succesful
        let sbcs = self.read_dm_register::<Sbcs>()?;

        if sbcs.sberror() != 0 {
            Err(RiscvError::SystemBusAccess)
        } else {
            Ok(data)
        }
    }

    /// Perform multiple reads from consecutive memory locations
    /// using system bus access.
    /// Only reads up to a width of 32 bits are currently supported.
    fn perform_memory_read_multiple_sysbus<V: RiscvValue32>(
        &mut self,
        address: u32,
        data: &mut [V],
    ) -> Result<(), RiscvError> {
        let mut sbcs = Sbcs(0);

        sbcs.set_sbaccess(V::WIDTH as u32);

        sbcs.set_sbreadonaddr(true);

        sbcs.set_sbreadondata(true);
        sbcs.set_sbautoincrement(true);

        self.schedule_write_dm_register(sbcs)?;

        self.schedule_write_dm_register(Sbaddress0(address))?;

        let data_len = data.len();

        let mut read_results: Vec<usize> = vec![];
        for _ in data[..data_len - 1].iter() {
            let idx = self.schedule_read_large_dtm_register::<V, Sbdata>()?;
            read_results.push(idx);
        }

        sbcs.set_sbautoincrement(false);
        self.schedule_write_dm_register(sbcs)?;

        // Read last value
        read_results.push(self.schedule_read_large_dtm_register::<V, Sbdata>()?);

        let sbcs_result = self.schedule_read_dm_register::<Sbcs>()?;

        let result = self.execute();

        let result = result?;
        for (out_index, &idx) in read_results.iter().enumerate() {
            data[out_index] = match result[idx] {
                CommandResult::U32(data) => V::from_register_value(data),
                _ => panic!("Internal error occurred."),
            };
        }

        // Check that the read was succesful
        let sbcs = match result[sbcs_result] {
            CommandResult::U32(res) => res,
            _ => panic!("Internal error occurred."),
        };

        let sbcs = Sbcs(sbcs);

        if sbcs.sberror() != 0 {
            Err(RiscvError::SystemBusAccess)
        } else {
            Ok(())
        }
    }

    /// Perform memory read from a single location using the program buffer.
    /// Only reads up to a width of 32 bits are currently supported.
    fn perform_memory_read_progbuf<V: RiscvValue32>(
        &mut self,
        address: u32,
    ) -> Result<V, RiscvError> {
        // assemble
        //  lb s1, 0(s0)

        // Backup register s0
        let s0 = self.abstract_cmd_register_read(&register::S0)?;

        let lw_command: u32 = assembly::lw(0, 8, V::WIDTH as u8, 8);

        self.setup_program_buffer(&[lw_command])?;

        self.write_dm_register(Data0(address))?;

        // Write s0, then execute program buffer
        let mut command = AccessRegisterCommand(0);
        command.set_cmd_type(0);
        command.set_transfer(true);
        command.set_write(true);

        // registers are 32 bit, so we have size 2 here
        command.set_aarsize(RiscvBusAccess::A32);
        command.set_postexec(true);

        // register s0, ie. 0x1008
        command.set_regno((register::S0).id.0 as u32);

        self.write_dm_register(command)?;

        let status: Abstractcs = self.read_dm_register()?;

        if status.cmderr() != 0 {
            return Err(RiscvError::AbstractCommand(
                AbstractCommandErrorKind::parse(status.cmderr() as u8),
            ));
        }

        // Read back s0
        let value = self.abstract_cmd_register_read(&register::S0)?;

        // Restore s0 register
        self.abstract_cmd_register_write(&register::S0, s0)?;

        Ok(V::from_register_value(value))
    }

    fn perform_memory_read_multiple_progbuf<V: RiscvValue32>(
        &mut self,
        address: u32,
        data: &mut [V],
    ) -> Result<(), RiscvError> {
        // Backup registers s0 and s1
        let s0 = self.abstract_cmd_register_read(&register::S0)?;
        let s1 = self.abstract_cmd_register_read(&register::S1)?;

        // Load a word from address in register 8 (S0), with offset 0, into register 9 (S9)
        let lw_command: u32 = assembly::lw(0, 8, V::WIDTH as u8, 9);

        self.setup_program_buffer(&[
            lw_command,
            assembly::addi(8, 8, V::WIDTH.byte_width() as u16),
        ])?;

        self.write_dm_register(Data0(address))?;

        // Write s0, then execute program buffer
        let mut command = AccessRegisterCommand(0);
        command.set_cmd_type(0);
        command.set_transfer(true);
        command.set_write(true);

        // registers are 32 bit, so we have size 2 here
        command.set_aarsize(RiscvBusAccess::A32);
        command.set_postexec(true);

        // register s0, ie. 0x1008
        command.set_regno((register::S0).id.0 as u32);

        self.write_dm_register(command)?;

        let data_len = data.len();

        for word in &mut data[..data_len - 1] {
            let mut command = AccessRegisterCommand(0);
            command.set_cmd_type(0);
            command.set_transfer(true);
            command.set_write(false);

            // registers are 32 bit, so we have size 2 here
            command.set_aarsize(RiscvBusAccess::A32);
            command.set_postexec(true);

            command.set_regno((register::S1).id.0 as u32);

            self.write_dm_register(command)?;

            // Read back s1
            let value: Data0 = self.read_dm_register()?;

            *word = V::from_register_value(value.0);
        }

        let last_value = self.abstract_cmd_register_read(&register::S1)?;

        data[data.len() - 1] = V::from_register_value(last_value);

        let status: Abstractcs = self.read_dm_register()?;

        if status.cmderr() != 0 {
            return Err(RiscvError::AbstractCommand(
                AbstractCommandErrorKind::parse(status.cmderr() as u8),
            ));
        }

        self.abstract_cmd_register_write(&register::S0, s0)?;
        self.abstract_cmd_register_write(&register::S1, s1)?;

        Ok(())
    }

    /// Memory write using system bus
    fn perform_memory_write_sysbus<V: RiscvValue>(
        &mut self,
        address: u32,
        data: &[V],
    ) -> Result<(), RiscvError> {
        let mut sbcs = Sbcs(0);

        // Set correct access width
        sbcs.set_sbaccess(V::WIDTH as u32);
        sbcs.set_sbautoincrement(true);

        self.schedule_write_dm_register(sbcs)?;

        self.schedule_write_dm_register(Sbaddress0(address))?;

        for value in data {
            self.schedule_write_large_dtm_register::<V, Sbdata>(*value)?;
        }

        // Check that the write was succesful
        let ok_index = self.schedule_read_dm_register::<Sbcs>()?;

        let result = self.execute()?;

        // Check that the write was succesful
        let sbcs = match result[ok_index] {
            CommandResult::U32(res) => res,
            _ => panic!("Internal error occurred."),
        };

        let sbcs = Sbcs(sbcs);

        if sbcs.sberror() != 0 {
            Err(RiscvError::SystemBusAccess)
        } else {
            Ok(())
        }
    }

    /// Perform memory write to a single location using the program buffer.
    /// Only writes up to a width of 32 bits are currently supported.
    fn perform_memory_write_progbuf<V: RiscvValue32>(
        &mut self,
        address: u32,
        data: V,
    ) -> Result<(), RiscvError> {
        log::debug!(
            "Memory write using progbuf - {:#010x} = {:#?}",
            address,
            data
        );

        // Backup registers s0 and s1
        let s0 = self.abstract_cmd_register_read(&register::S0)?;
        let s1 = self.abstract_cmd_register_read(&register::S1)?;

        let sw_command = assembly::sw(0, 8, V::WIDTH as u32, 9);

        self.setup_program_buffer(&[sw_command])?;

        // write address into s0
        self.abstract_cmd_register_write(&register::S0, address)?;

        // write data into data 0
        self.write_dm_register(Data0(data.into()))?;

        // Write s1, then execute program buffer
        let mut command = AccessRegisterCommand(0);
        command.set_cmd_type(0);
        command.set_transfer(true);
        command.set_write(true);

        // registers are 32 bit, so we have size 2 here
        command.set_aarsize(RiscvBusAccess::A32);
        command.set_postexec(true);

        // register s1, ie. 0x1009
        command.set_regno((register::S1).id.0 as u32);

        self.write_dm_register(command)?;

        let status: Abstractcs = self.read_dm_register()?;

        if status.cmderr() != 0 {
            let error = AbstractCommandErrorKind::parse(status.cmderr() as u8);

            log::error!(
                "Executing the abstract command for perform_memory_write failed: {:?} ({:x?})",
                error,
                status,
            );

            return Err(RiscvError::AbstractCommand(error));
        }

        // Restore register s0 and s1

        self.abstract_cmd_register_write(&register::S0, s0)?;
        self.abstract_cmd_register_write(&register::S1, s1)?;

        Ok(())
    }

    /// Perform multiple memory writes to consecutive locations using the program buffer.
    /// Only writes up to a width of 32 bits are currently supported.
    fn perform_memory_write_multiple_progbuf<V: RiscvValue32>(
        &mut self,
        address: u32,
        data: &[V],
    ) -> Result<(), RiscvError> {
        let s0 = self.abstract_cmd_register_read(&register::S0)?;
        let s1 = self.abstract_cmd_register_read(&register::S1)?;

        // Setup program buffer for multiple writes
        // Store value from register s9 into memory,
        // then increase the address for next write.
        let sw_command = assembly::sw(0, 8, V::WIDTH as u32, 9);

        self.setup_program_buffer(&[
            sw_command,
            assembly::addi(8, 8, V::WIDTH.byte_width() as u16),
        ])?;

        // write address into s0
        self.abstract_cmd_register_write(&register::S0, address)?;

        for value in data {
            // write address into data 0
            self.write_dm_register(Data0((*value).into()))?;

            // Write s0, then execute program buffer
            let mut command = AccessRegisterCommand(0);
            command.set_cmd_type(0);
            command.set_transfer(true);
            command.set_write(true);

            // registers are 32 bit, so we have size 2 here
            command.set_aarsize(RiscvBusAccess::A32);
            command.set_postexec(true);

            // register s1
            command.set_regno((register::S1).id.0 as u32);

            self.write_dm_register(command)?;
        }

        // Errors are sticky, so we can just check at the end if everything worked.
        let status: Abstractcs = self.read_dm_register()?;

        if status.cmderr() != 0 {
            let error = AbstractCommandErrorKind::parse(status.cmderr() as u8);

            log::error!(
                "Executing the abstract command for write_32 failed: {:?} ({:x?})",
                error,
                status,
            );

            return Err(DebugProbeError::ArchitectureSpecific(Box::new(
                RiscvError::AbstractCommand(error),
            ))
            .into());
        }

        // Restore register s0 and s1

        self.abstract_cmd_register_write(&register::S0, s0)?;
        self.abstract_cmd_register_write(&register::S1, s1)?;

        Ok(())
    }

    pub(crate) fn execute_abstract_command(&mut self, command: u32) -> Result<(), RiscvError> {
        // ensure that preconditions are fullfileld
        // haltreq      = 0
        // resumereq    = 0
        // ackhavereset = 0

        let mut dmcontrol = Dmcontrol(0);
        dmcontrol.set_haltreq(false);
        dmcontrol.set_resumereq(false);
        dmcontrol.set_ackhavereset(true);
        dmcontrol.set_dmactive(true);
        self.write_dm_register(dmcontrol)?;

        // read abstractcs to see its state
        let abstractcs_prev: Abstractcs = self.read_dm_register()?;

        log::debug!("abstractcs: {:?}", abstractcs_prev);

        if abstractcs_prev.cmderr() != 0 {
            // Clear previous command error.
            let mut abstractcs_clear = Abstractcs(0);
            abstractcs_clear.set_cmderr(0x7);

            self.write_dm_register(abstractcs_clear)?;
        }

        self.write_dm_register(Command(command))?;

        // poll busy flag in abstractcs

        let start_time = Instant::now();

        let mut abstractcs: Abstractcs;

        loop {
            abstractcs = self.read_dm_register()?;

            if !abstractcs.busy() {
                break;
            }

            if start_time.elapsed() > RISCV_TIMEOUT {
                return Err(RiscvError::Timeout);
            }
        }

        log::debug!("abstracts: {:?}", abstractcs);

        // check cmderr
        if abstractcs.cmderr() != 0 {
            return Err(RiscvError::AbstractCommand(
                AbstractCommandErrorKind::parse(abstractcs.cmderr() as u8),
            ));
        }

        Ok(())
    }

    /// Check if a register can be accessed via abstract commands
    fn check_abstract_cmd_register_support(
        &self,
        regno: RegisterId,
        rw: CoreRegisterAbstractCmdSupport,
    ) -> bool {
        if let Some(status) = self.state.abstract_cmd_register_info.get(&regno) {
            status.supports(rw)
        } else {
            // If not cached yet, assume the register is accessible
            true
        }
    }

    /// Remember, that the given register can not be accessed via abstract commands
    fn set_abstract_cmd_register_unsupported(
        &mut self,
        regno: RegisterId,
        rw: CoreRegisterAbstractCmdSupport,
    ) {
        let entry = self
            .state
            .abstract_cmd_register_info
            .entry(regno)
            .or_insert(CoreRegisterAbstractCmdSupport::BOTH);

        entry.unset(rw);
    }

    // Read a core register using an abstract command
    pub(crate) fn abstract_cmd_register_read(
        &mut self,
        regno: impl Into<RegisterId>,
    ) -> Result<u32, RiscvError> {
        let regno = regno.into();

        // Check if the register was already tried via abstract cmd
        if !self.check_abstract_cmd_register_support(regno, CoreRegisterAbstractCmdSupport::READ) {
            return Err(RiscvError::AbstractCommand(
                AbstractCommandErrorKind::NotSupported,
            ));
        }

        // read from data0
        let mut command = AccessRegisterCommand(0);
        command.set_cmd_type(0);
        command.set_transfer(true);
        command.set_aarsize(RiscvBusAccess::A32);

        command.set_regno(regno.0 as u32);

        match self.execute_abstract_command(command.0) {
            Ok(_) => (),
            err @ Err(RiscvError::AbstractCommand(AbstractCommandErrorKind::NotSupported)) => {
                // Remember, that this register is unsupported
                self.set_abstract_cmd_register_unsupported(
                    regno,
                    CoreRegisterAbstractCmdSupport::READ,
                );
                err?;
            }
            Err(e) => return Err(e),
        }

        let register_value: Data0 = self.read_dm_register()?;

        Ok(register_value.into())
    }

    pub(crate) fn abstract_cmd_register_write<V: RiscvValue>(
        &mut self,
        regno: impl Into<RegisterId>,
        value: V,
    ) -> Result<(), RiscvError> {
        let regno = regno.into();

        // Check if the register was already tried via abstract cmd
        if !self.check_abstract_cmd_register_support(regno, CoreRegisterAbstractCmdSupport::WRITE) {
            return Err(RiscvError::AbstractCommand(
                AbstractCommandErrorKind::NotSupported,
            ));
        }

        // write to data0
        let mut command = AccessRegisterCommand(0);
        command.set_cmd_type(0);
        command.set_transfer(true);
        command.set_write(true);
        command.set_aarsize(V::WIDTH);

        command.set_regno(regno.0 as u32);

        self.write_large_dtm_register::<V, Arg0>(value)?;

        match self.execute_abstract_command(command.0) {
            Ok(_) => Ok(()),
            err @ Err(RiscvError::AbstractCommand(AbstractCommandErrorKind::NotSupported)) => {
                // Remember, that this register is unsupported
                self.set_abstract_cmd_register_unsupported(
                    regno,
                    CoreRegisterAbstractCmdSupport::WRITE,
                );
                err
            }
            Err(e) => Err(e),
        }
    }

    /// Read the CSR progbuf register.
    pub fn read_csr_progbuf(&mut self, address: u16) -> Result<u32, RiscvError> {
        log::debug!("Reading CSR {:#04x}", address);

        let s0 = self.abstract_cmd_register_read(&register::S0)?;

        // Read csr value into register 8 (s0)
        let csrr_cmd = assembly::csrr(8, address);

        self.setup_program_buffer(&[csrr_cmd])?;

        // command: postexec
        let mut postexec_cmd = AccessRegisterCommand(0);
        postexec_cmd.set_postexec(true);

        self.execute_abstract_command(postexec_cmd.0)?;

        // read the s0 value
        let reg_value = self.abstract_cmd_register_read(&register::S0)?;

        // restore original value in s0
        self.abstract_cmd_register_write(&register::S0, s0)?;

        Ok(reg_value)
    }

    /// Write the CSR progbuf register.
    pub fn write_csr_progbuf(&mut self, address: u16, value: u32) -> Result<(), RiscvError> {
        log::debug!("Writing CSR {:#04x}={}", address, value);

        // Backup register s0
        let s0 = self.abstract_cmd_register_read(&register::S0)?;

        // Write value into s0
        self.abstract_cmd_register_write(&register::S0, value)?;

        // Built the CSRW command to write into the program buffer
        let csrw_cmd = assembly::csrw(address, 8);
        self.setup_program_buffer(&[csrw_cmd])?;

        // command: postexec
        let mut postexec_cmd = AccessRegisterCommand(0);
        postexec_cmd.set_postexec(true);

        self.execute_abstract_command(postexec_cmd.0)?;

        // command: transfer, regno = 0x1008
        // restore original value in s0
        self.abstract_cmd_register_write(&register::S0, s0)?;

        Ok(())
    }

    fn read_large_dtm_register<V, R>(&mut self) -> Result<V, RiscvError>
    where
        V: RiscvValue,
        R: LargeRegister,
    {
        V::read_from_register::<R>(self)
    }

    fn write_large_dtm_register<V, R>(&mut self, value: V) -> Result<(), RiscvError>
    where
        V: RiscvValue,
        R: LargeRegister,
    {
        V::write_to_register::<R>(self, value)
    }

    fn read_word<V: RiscvValue32>(&mut self, address: u32) -> Result<V, crate::Error> {
        let result = match self.state.memory_access_method(V::WIDTH) {
            MemoryAccessMethod::ProgramBuffer => self.perform_memory_read_progbuf(address)?,
            MemoryAccessMethod::SystemBus => self.perform_memory_read_sysbus(address)?,
            MemoryAccessMethod::AbstractCommand => {
                unimplemented!("Memory access using abstract commands is not implemted")
            }
        };

        Ok(result)
    }

    fn read_multiple<V: RiscvValue32>(
        &mut self,
        address: u32,
        data: &mut [V],
    ) -> Result<(), crate::Error> {
        log::debug!("read_32 from {:#08x}", address);

        match self.state.memory_access_method(RiscvBusAccess::A32) {
            MemoryAccessMethod::ProgramBuffer => {
                self.perform_memory_read_multiple_progbuf(address, data)?;
            }
            MemoryAccessMethod::SystemBus => {
                self.perform_memory_read_multiple_sysbus(address, data)?;
            }
            MemoryAccessMethod::AbstractCommand => {
                unimplemented!("Memory access using abstract commands is not implemted")
            }
        };

        Ok(())
    }

    fn write_word<V: RiscvValue32>(&mut self, address: u32, data: V) -> Result<(), crate::Error> {
        match self.state.memory_access_method(V::WIDTH) {
            MemoryAccessMethod::ProgramBuffer => {
                self.perform_memory_write_progbuf(address, data)?
            }
            MemoryAccessMethod::SystemBus => self.perform_memory_write_sysbus(address, &[data])?,
            MemoryAccessMethod::AbstractCommand => {
                unimplemented!("Memory access using abstract commands is not implemted")
            }
        };

        Ok(())
    }

    fn write_multiple<V: RiscvValue32>(
        &mut self,
        address: u32,
        data: &[V],
    ) -> Result<(), crate::Error> {
        match self.state.memory_access_method(V::WIDTH) {
            MemoryAccessMethod::SystemBus => self.perform_memory_write_sysbus(address, data)?,
            MemoryAccessMethod::ProgramBuffer => {
                self.perform_memory_write_multiple_progbuf(address, data)?
            }
            MemoryAccessMethod::AbstractCommand => {
                unimplemented!("Memory access using abstract commands is not implemted")
            }
        }

        Ok(())
    }

    /// Destruct the interface and return the stored probe driver.
    pub fn close(self) -> Probe {
        Probe::from_attached_probe(self.dtm.probe.into_probe())
    }

    pub(super) fn execute(&mut self) -> Result<Vec<CommandResult>, DebugProbeError> {
        self.dtm.execute()
    }

    pub(super) fn schedule_write_dm_register<R: DebugRegister>(
        &mut self,
        register: R,
    ) -> Result<(), DebugProbeError> {
        // write write command to dmi register

        log::debug!(
            "Write DM register '{}' at {:#010x} = {:x?}",
            R::NAME,
            R::ADDRESS,
            register
        );

        self.schedule_write_dm_register_untyped(R::ADDRESS as u64, register.into())?;
        Ok(())
    }

    /// Write to a DM register
    ///
    /// Use the [`schedule_write_dm_register`] function if possible.
    fn schedule_write_dm_register_untyped(
        &mut self,
        address: u64,
        value: u32,
    ) -> Result<DeferredResultIndex, DebugProbeError> {
        self.dtm
            .schedule_dmi_register_access(address, value, DmiOperation::Write)
    }

    pub(super) fn schedule_read_dm_register<R: DebugRegister>(
        &mut self,
    ) -> Result<DeferredResultIndex, DebugProbeError> {
        log::debug!("Reading DM register '{}' at {:#010x}", R::NAME, R::ADDRESS);

        self.schedule_read_dm_register_untyped(R::ADDRESS as u64)
    }

    /// Read from a DM register
    ///
    /// Use the [`schedule_read_dm_register`] function if possible.
    fn schedule_read_dm_register_untyped(
        &mut self,
        address: u64,
    ) -> Result<DeferredResultIndex, DebugProbeError> {
        // Prepare the read by sending a read request with the register address
        self.dtm
            .schedule_dmi_register_access(address, 0, DmiOperation::Read)?;

        // Read back the response from the previous request.
        self.dtm
            .schedule_dmi_register_access(0, 0, DmiOperation::NoOp)
    }

    fn schedule_read_large_dtm_register<V, R>(
        &mut self,
    ) -> Result<DeferredResultIndex, DebugProbeError>
    where
        V: RiscvValue,
        R: LargeRegister,
    {
        V::schedule_read_from_register::<R>(self)
    }

    fn schedule_write_large_dtm_register<V, R>(
        &mut self,
        value: V,
    ) -> Result<DeferredResultIndex, DebugProbeError>
    where
        V: RiscvValue,
        R: LargeRegister,
    {
        V::schedule_write_to_register::<R>(self, value)
    }
}
pub(crate) trait LargeRegister {
    const R0_ADDRESS: u8;
    const R1_ADDRESS: u8;
    const R2_ADDRESS: u8;
    const R3_ADDRESS: u8;
}

struct Sbdata {}

impl LargeRegister for Sbdata {
    const R0_ADDRESS: u8 = Sbdata0::ADDRESS;
    const R1_ADDRESS: u8 = Sbdata1::ADDRESS;
    const R2_ADDRESS: u8 = Sbdata2::ADDRESS;
    const R3_ADDRESS: u8 = Sbdata3::ADDRESS;
}

struct Arg0 {}

impl LargeRegister for Arg0 {
    const R0_ADDRESS: u8 = Data0::ADDRESS;
    const R1_ADDRESS: u8 = Data1::ADDRESS;
    const R2_ADDRESS: u8 = Data2::ADDRESS;
    const R3_ADDRESS: u8 = Data3::ADDRESS;
}

/// Helper trait, limited to RiscvValue no larger than 32 bits
pub(crate) trait RiscvValue32: RiscvValue + Into<u32> {
    fn from_register_value(value: u32) -> Self;
}

impl RiscvValue32 for u8 {
    fn from_register_value(value: u32) -> Self {
        value as u8
    }
}
impl RiscvValue32 for u16 {
    fn from_register_value(value: u32) -> Self {
        value as u16
    }
}
impl RiscvValue32 for u32 {
    fn from_register_value(value: u32) -> Self {
        value
    }
}

/// Marker trait for different values which
/// can be read / written using the debug module.
pub(crate) trait RiscvValue: std::fmt::Debug + Copy + Sized {
    const WIDTH: RiscvBusAccess;

    fn read_from_register<R>(
        interface: &mut RiscvCommunicationInterface,
    ) -> Result<Self, RiscvError>
    where
        R: LargeRegister;

    fn write_to_register<R>(
        interface: &mut RiscvCommunicationInterface,
        value: Self,
    ) -> Result<(), RiscvError>
    where
        R: LargeRegister;

    fn schedule_read_from_register<R>(
        interface: &mut RiscvCommunicationInterface,
    ) -> Result<DeferredResultIndex, DebugProbeError>
    where
        R: LargeRegister;

    fn schedule_write_to_register<R>(
        interface: &mut RiscvCommunicationInterface,
        value: Self,
    ) -> Result<DeferredResultIndex, DebugProbeError>
    where
        R: LargeRegister;
}

impl RiscvValue for u8 {
    const WIDTH: RiscvBusAccess = RiscvBusAccess::A8;

    fn read_from_register<R>(
        interface: &mut RiscvCommunicationInterface,
    ) -> Result<Self, RiscvError>
    where
        R: LargeRegister,
    {
        interface
            .read_dm_register_untyped(R::R0_ADDRESS as u64)
            .map(|v| v as u8)
    }

    fn write_to_register<R>(
        interface: &mut RiscvCommunicationInterface,
        value: Self,
    ) -> Result<(), RiscvError>
    where
        R: LargeRegister,
    {
        interface.write_dm_register_untyped(R::R0_ADDRESS as u64, value as u32)
    }

    fn schedule_read_from_register<R>(
        interface: &mut RiscvCommunicationInterface,
    ) -> Result<DeferredResultIndex, DebugProbeError>
    where
        R: LargeRegister,
    {
        interface.schedule_read_dm_register_untyped(R::R0_ADDRESS as u64)
    }

    fn schedule_write_to_register<R>(
        interface: &mut RiscvCommunicationInterface,
        value: Self,
    ) -> Result<DeferredResultIndex, DebugProbeError>
    where
        R: LargeRegister,
    {
        interface.schedule_write_dm_register_untyped(R::R0_ADDRESS as u64, value as u32)
    }
}

impl RiscvValue for u16 {
    const WIDTH: RiscvBusAccess = RiscvBusAccess::A16;
    fn read_from_register<R>(
        interface: &mut RiscvCommunicationInterface,
    ) -> Result<Self, RiscvError>
    where
        R: LargeRegister,
    {
        interface
            .read_dm_register_untyped(R::R0_ADDRESS as u64)
            .map(|v| v as u16)
    }

    fn write_to_register<R>(
        interface: &mut RiscvCommunicationInterface,
        value: Self,
    ) -> Result<(), RiscvError>
    where
        R: LargeRegister,
    {
        interface.write_dm_register_untyped(R::R0_ADDRESS as u64, value as u32)
    }

    fn schedule_read_from_register<R>(
        interface: &mut RiscvCommunicationInterface,
    ) -> Result<DeferredResultIndex, DebugProbeError>
    where
        R: LargeRegister,
    {
        interface.schedule_read_dm_register_untyped(R::R0_ADDRESS as u64)
    }

    fn schedule_write_to_register<R>(
        interface: &mut RiscvCommunicationInterface,
        value: Self,
    ) -> Result<DeferredResultIndex, DebugProbeError>
    where
        R: LargeRegister,
    {
        interface.schedule_write_dm_register_untyped(R::R0_ADDRESS as u64, value as u32)
    }
}

impl RiscvValue for u32 {
    const WIDTH: RiscvBusAccess = RiscvBusAccess::A32;
    fn read_from_register<R>(
        interface: &mut RiscvCommunicationInterface,
    ) -> Result<Self, RiscvError>
    where
        R: LargeRegister,
    {
        interface.read_dm_register_untyped(R::R0_ADDRESS as u64)
    }

    fn write_to_register<R>(
        interface: &mut RiscvCommunicationInterface,
        value: Self,
    ) -> Result<(), RiscvError>
    where
        R: LargeRegister,
    {
        interface.write_dm_register_untyped(R::R0_ADDRESS as u64, value)
    }

    fn schedule_read_from_register<R>(
        interface: &mut RiscvCommunicationInterface,
    ) -> Result<DeferredResultIndex, DebugProbeError>
    where
        R: LargeRegister,
    {
        interface.schedule_read_dm_register_untyped(R::R0_ADDRESS as u64)
    }
    fn schedule_write_to_register<R>(
        interface: &mut RiscvCommunicationInterface,
        value: Self,
    ) -> Result<DeferredResultIndex, DebugProbeError>
    where
        R: LargeRegister,
    {
        interface.schedule_write_dm_register_untyped(R::R0_ADDRESS as u64, value as u32)
    }
}

impl RiscvValue for u64 {
    const WIDTH: RiscvBusAccess = RiscvBusAccess::A64;

    fn read_from_register<R>(
        interface: &mut RiscvCommunicationInterface,
    ) -> Result<Self, RiscvError>
    where
        R: LargeRegister,
    {
        // R0 has to be read last, side effects are triggerd by reads from
        // this register.
        let upper_bits = interface.read_dm_register_untyped(R::R1_ADDRESS as u64)?;
        let lower_bits = interface.read_dm_register_untyped(R::R0_ADDRESS as u64)?;

        Ok((upper_bits as u64) << 32 | lower_bits as u64)
    }

    fn write_to_register<R>(
        interface: &mut RiscvCommunicationInterface,
        value: Self,
    ) -> Result<(), RiscvError>
    where
        R: LargeRegister,
    {
        let upper_bits = (value >> 32) as u32;
        let lower_bits = (value & 0xffff_ffff) as u32;

        // R0 has to be written last, side effects are triggerd by writes from
        // this register.

        interface.write_dm_register_untyped(R::R1_ADDRESS as u64, upper_bits)?;
        interface.write_dm_register_untyped(R::R0_ADDRESS as u64, lower_bits)
    }

    fn schedule_read_from_register<R>(
        interface: &mut RiscvCommunicationInterface,
    ) -> Result<DeferredResultIndex, DebugProbeError>
    where
        R: LargeRegister,
    {
        interface.schedule_read_dm_register_untyped(R::R1_ADDRESS as u64)?;
        interface.schedule_read_dm_register_untyped(R::R0_ADDRESS as u64)
    }

    fn schedule_write_to_register<R>(
        interface: &mut RiscvCommunicationInterface,
        value: Self,
    ) -> Result<DeferredResultIndex, DebugProbeError>
    where
        R: LargeRegister,
    {
        let upper_bits = (value >> 32) as u32;
        let lower_bits = (value & 0xffff_ffff) as u32;

        // R0 has to be written last, side effects are triggerd by writes from
        // this register.

        interface.schedule_write_dm_register_untyped(R::R1_ADDRESS as u64, upper_bits)?;
        interface.schedule_write_dm_register_untyped(R::R0_ADDRESS as u64, lower_bits)
    }
}

impl RiscvValue for u128 {
    const WIDTH: RiscvBusAccess = RiscvBusAccess::A128;

    fn read_from_register<R>(
        interface: &mut RiscvCommunicationInterface,
    ) -> Result<Self, RiscvError>
    where
        R: LargeRegister,
    {
        // R0 has to be read last, side effects are triggerd by reads from
        // this register.
        let bits_3 = interface.read_dm_register_untyped(R::R3_ADDRESS as u64)?;
        let bits_2 = interface.read_dm_register_untyped(R::R2_ADDRESS as u64)?;
        let bits_1 = interface.read_dm_register_untyped(R::R1_ADDRESS as u64)?;
        let bits_0 = interface.read_dm_register_untyped(R::R0_ADDRESS as u64)?;

        Ok((bits_3 as u128) << 96
            | (bits_2 as u128) << 64
            | (bits_1 as u128) << 32
            | bits_0 as u128)
    }

    fn write_to_register<R>(
        interface: &mut RiscvCommunicationInterface,
        value: Self,
    ) -> Result<(), RiscvError>
    where
        R: LargeRegister,
    {
        let bits_3 = (value >> 96) as u32;
        let bits_2 = (value >> 64) as u32;
        let bits_1 = (value >> 32) as u32;
        let bits_0 = (value & 0xffff_ffff) as u32;

        // R0 has to be written last, side effects are triggerd by writes from
        // this register.

        interface.write_dm_register_untyped(R::R3_ADDRESS as u64, bits_3)?;
        interface.write_dm_register_untyped(R::R2_ADDRESS as u64, bits_2)?;
        interface.write_dm_register_untyped(R::R1_ADDRESS as u64, bits_1)?;
        interface.write_dm_register_untyped(R::R0_ADDRESS as u64, bits_0)
    }

    fn schedule_read_from_register<R>(
        interface: &mut RiscvCommunicationInterface,
    ) -> Result<DeferredResultIndex, DebugProbeError>
    where
        R: LargeRegister,
    {
        interface.schedule_read_dm_register_untyped(R::R3_ADDRESS as u64)?;
        interface.schedule_read_dm_register_untyped(R::R2_ADDRESS as u64)?;
        interface.schedule_read_dm_register_untyped(R::R1_ADDRESS as u64)?;
        interface.schedule_read_dm_register_untyped(R::R0_ADDRESS as u64)
    }

    fn schedule_write_to_register<R>(
        interface: &mut RiscvCommunicationInterface,
        value: Self,
    ) -> Result<DeferredResultIndex, DebugProbeError>
    where
        R: LargeRegister,
    {
        let bits_3 = (value >> 96) as u32;
        let bits_2 = (value >> 64) as u32;
        let bits_1 = (value >> 32) as u32;
        let bits_0 = (value & 0xffff_ffff) as u32;

        // R0 has to be written last, side effects are triggerd by writes from
        // this register.

        interface.schedule_write_dm_register_untyped(R::R3_ADDRESS as u64, bits_3)?;
        interface.schedule_write_dm_register_untyped(R::R2_ADDRESS as u64, bits_2)?;
        interface.schedule_write_dm_register_untyped(R::R1_ADDRESS as u64, bits_1)?;
        interface.schedule_write_dm_register_untyped(R::R0_ADDRESS as u64, bits_0)
    }
}

impl MemoryInterface for RiscvCommunicationInterface {
    fn supports_native_64bit_access(&mut self) -> bool {
        false
    }

    fn read_word_64(&mut self, address: u64) -> Result<u64, crate::error::Error> {
        let address = valid_32_address(address)?;
        let mut ret = self.read_word::<u32>(address)? as u64;
        ret |= (self.read_word::<u32>(address + 4)? as u64) << 32;

        Ok(ret)
    }

    fn read_word_32(&mut self, address: u64) -> Result<u32, crate::Error> {
        let address = valid_32_address(address)?;
        self.read_word(address)
    }

    fn read_word_8(&mut self, address: u64) -> Result<u8, crate::Error> {
        let address = valid_32_address(address)?;
        log::debug!("read_word_8 from {:#08x}", address);
        self.read_word(address)
    }

    fn read_64(&mut self, address: u64, data: &mut [u64]) -> Result<(), crate::error::Error> {
        let address = valid_32_address(address)?;
        log::debug!("read_64 from {:#08x}", address);

        for (i, d) in data.iter_mut().enumerate() {
            *d = self.read_word_64((address + (i as u32 * 8)).into())?;
        }

        Ok(())
    }

    fn read_32(&mut self, address: u64, data: &mut [u32]) -> Result<(), crate::Error> {
        let address = valid_32_address(address)?;
        log::debug!("read_32 from {:#08x}", address);
        self.read_multiple(address, data)
    }

    /// Read 8-bit values from target memory.
    fn read_8(&mut self, address: u64, data: &mut [u8]) -> Result<(), crate::Error> {
        let address = valid_32_address(address)?;
        log::debug!("read_8 from {:#08x}", address);

        self.read_multiple(address, data)
    }

    fn write_word_64(&mut self, address: u64, data: u64) -> Result<(), crate::error::Error> {
        let address = valid_32_address(address)?;
        let low_word = data as u32;
        let high_word = (data >> 32) as u32;

        self.write_word(address, low_word)?;
        self.write_word(address + 4, high_word)
    }

    fn write_word_32(&mut self, address: u64, data: u32) -> Result<(), crate::Error> {
        let address = valid_32_address(address)?;
        self.write_word(address, data)
    }

    fn write_word_8(&mut self, address: u64, data: u8) -> Result<(), crate::Error> {
        let address = valid_32_address(address)?;
        self.write_word(address, data)
    }

    fn write_64(&mut self, address: u64, data: &[u64]) -> Result<(), crate::error::Error> {
        let address = valid_32_address(address)?;
        log::debug!("write_64 to {:#08x}", address);

        for (i, d) in data.iter().enumerate() {
            self.write_word_64((address + (i as u32 * 8)).into(), *d)?;
        }

        Ok(())
    }

    fn write_32(&mut self, address: u64, data: &[u32]) -> Result<(), crate::Error> {
        let address = valid_32_address(address)?;
        log::debug!("write_32 to {:#08x}", address);

        self.write_multiple(address, data)
    }

    fn write_8(&mut self, address: u64, data: &[u8]) -> Result<(), crate::Error> {
        let address = valid_32_address(address)?;
        log::debug!("write_8 to {:#08x}", address);

        self.write_multiple(address, data)
    }

    fn flush(&mut self) -> Result<(), crate::Error> {
        Ok(())
    }
}

/// Access width for bus access.
/// This is used both for system bus access (`sbcs` register),
/// as well for abstract commands.
#[derive(Copy, Clone, PartialEq, PartialOrd, Hash, Eq, Debug)]
pub enum RiscvBusAccess {
    /// 1 byte
    A8 = 0,
    /// 2 bytes
    A16 = 1,
    /// 4 bytes
    A32 = 2,
    /// 8 bytes
    A64 = 3,
    /// 16 bytes.
    A128 = 4,
}

impl RiscvBusAccess {
    /// Width of an access in bytes
    const fn byte_width(&self) -> usize {
        match self {
            RiscvBusAccess::A8 => 1,
            RiscvBusAccess::A16 => 2,
            RiscvBusAccess::A32 => 4,
            RiscvBusAccess::A64 => 8,
            RiscvBusAccess::A128 => 16,
        }
    }
}

impl From<RiscvBusAccess> for u8 {
    fn from(value: RiscvBusAccess) -> Self {
        value as u8
    }
}

/// Different methods of memory access,
/// which can be supported by a debug module.
///
/// The `AbstractCommand` method for memory access is not implemented.
#[derive(Debug, Copy, Clone)]
#[allow(dead_code)]
enum MemoryAccessMethod {
    /// Memory access using the program buffer is supported
    ProgramBuffer,
    /// Memory access using an abstract command is supported
    AbstractCommand,
    /// Memory access using system bus access supported
    SystemBus,
}

bitfield! {
    /// Abstract command register, located at address 0x17
    /// This is not for all commands, only for the ones
    /// from the debug spec.
    pub struct AccessRegisterCommand(u32);
    impl Debug;
    /// This is 0 to indicate Access Register Command.
    pub _, set_cmd_type: 31, 24;
    /// 2: Access the lowest 32 bits of the register.\
    /// 3: Access the lowest 64 bits of the register.\
    /// 4: Access the lowest 128 bits of the register.
    ///
    /// If aarsize specifies a size larger than the register’s
    /// actual size, then the access must fail. If a register is accessible, then reads of aarsize less than
    /// or equal to the register’s actual size must be supported.
    ///
    /// This field controls the Argument Width as referenced in Table 3.1.
    pub u8, from into RiscvBusAccess, _, set_aarsize: 22, 20;
    /// 0: No effect. This variant must be supported.\
    /// 1: After a successful register access, regno is incremented (wrapping around to 0). Supporting
    /// this variant is optional.
    pub _, set_aarpostincrement: 19;
    /// 0: No effect. This variant must be supported, and
    /// is the only supported one if progbufsize is 0.\
    /// 1: Execute the program in the Program Buffer
    /// exactly once after performing the transfer, if any.
    /// Supporting this variant is optional.
    pub _, set_postexec: 18;
    /// 0: Don’t do the operation specified by write.\
    /// 1: Do the operation specified by write.
    /// This bit can be used to just execute the Program Buffer without having to worry about placing valid values into aarsize or regno
    pub _, set_transfer: 17;
    /// When transfer is set: 0: Copy data from the specified register into arg0 portion of data.
    /// 1: Copy data from arg0 portion of data into the
    /// specified register.
    pub _, set_write: 16;
    /// Number of the register to access, as described in
    /// Table 3.3. dpc may be used as an alias for PC if
    /// this command is supported on a non-halted hart.
    pub _, set_regno: 15, 0;
}

impl DebugRegister for AccessRegisterCommand {
    const ADDRESS: u8 = 0x17;
    const NAME: &'static str = "command";
}

impl From<AccessRegisterCommand> for u32 {
    fn from(register: AccessRegisterCommand) -> Self {
        register.0
    }
}

impl From<u32> for AccessRegisterCommand {
    fn from(value: u32) -> Self {
        Self(value)
    }
}

pub(super) trait DebugRegister: Into<u32> + From<u32> + std::fmt::Debug {
    const ADDRESS: u8;
    const NAME: &'static str;
}

bitfield! {
    /// System Bus Access Control and Status (see 3.12.18)
    #[derive(Copy, Clone)]
    pub struct Sbcs(u32);
    impl Debug;
    /// 0: The System Bus interface conforms to mainline
    /// drafts of this spec older than 1 January, 2018.\
    /// 1: The System Bus interface conforms to this version of the spec.
    ///
    /// Other values are reserved for future versions
    sbversion, _: 31, 29;
    /// Set when the debugger attempts to read data
    /// while a read is in progress, or when the debugger initiates a new access while one is already in
    /// progress (while sbbusy is set). It remains set until
    /// it’s explicitly cleared by the debugger.
    /// While this field is set, no more system bus accesses
    /// can be initiated by the Debug Module.
    sbbusyerror, set_sbbusyerror: 22;
    /// When 1, indicates the system bus master is busy.
    /// (Whether the system bus itself is busy is related,
    /// but not the same thing.) This bit goes high immediately when a read or write is requested for
    /// any reason, and does not go low until the access
    /// is fully completed.
    ///
    /// Writes to sbcs while sbbusy is high result in undefined behavior. A debugger must not write to
    /// sbcs until it reads sbbusy as 0.
    sbbusy, _: 21;
    /// When 1, every write to sbaddress0 automatically
    /// triggers a system bus read at the new address.
    sbreadonaddr, set_sbreadonaddr: 20;
    /// Select the access size to use for system bus accesses.
    ///
    /// 0: 8-bit\
    /// 1: 16-bit\
    /// 2: 32-bit\
    /// 3: 64-bit\
    /// 4: 128-bit
    ///
    /// If sbaccess has an unsupported value when the
    /// DM starts a bus access, the access is not performed and sberror is set to 4.
    sbaccess, set_sbaccess: 19, 17;
    /// When 1, sbaddress is incremented by the access
    /// size (in bytes) selected in sbaccess after every system bus access.
    sbautoincrement, set_sbautoincrement: 16;
    /// When 1, every read from sbdata0 automatically
    /// triggers a system bus read at the (possibly autoincremented) address.
    sbreadondata, set_sbreadondata: 15;
    /// When the Debug Module’s system bus master encounters an error, this field gets set. The bits in
    /// this field remain set until they are cleared by writing 1 to them. While this field is non-zero, no
    /// more system bus accesses can be initiated by the
    /// Debug Module.
    /// An implementation may report “Other” (7) for any error condition.
    ///
    /// 0: There was no bus error.\
    /// 1: There was a timeout.\
    /// 2: A bad address was accessed.\
    /// 3: There was an alignment error.\
    /// 4: An access of unsupported size was requested.\
    /// 7: Other.
    sberror, set_sberror: 14, 12;
    /// Width of system bus addresses in bits. (0 indicates there is no bus access support.)
    sbasize, _: 11, 5;
    /// 1 when 128-bit system bus accesses are supported.
    sbaccess128, _: 4;
    /// 1 when 64-bit system bus accesses are supported.
    sbaccess64, _: 3;
    /// 1 when 32-bit system bus accesses are supported.
    sbaccess32, _: 2;
    /// 1 when 16-bit system bus accesses are supported.
    sbaccess16, _: 1;
    /// 1 when 8-bit system bus accesses are supported.
    sbaccess8, _: 0;
}

impl DebugRegister for Sbcs {
    const ADDRESS: u8 = 0x38;
    const NAME: &'static str = "sbcs";
}

impl From<Sbcs> for u32 {
    fn from(register: Sbcs) -> Self {
        register.0
    }
}

bitfield! {
    /// Abstract Command Autoexec (see 3.12.8)
    #[derive(Copy, Clone, PartialEq)]
    pub struct Abstractauto(u32);
    impl Debug;
    /// When a bit in this field is 1, read or write accesses to the corresponding progbuf word cause
    /// the command in command to be executed again.
    autoexecprogbuf, set_autoexecprogbuf: 31, 16;
    /// When a bit in this field is 1, read or write accesses to the corresponding data word cause the
    /// command in command to be executed again.
    autoexecdata, set_autoexecdata: 11, 0;
}

impl DebugRegister for Abstractauto {
    const ADDRESS: u8 = 0x18;
    const NAME: &'static str = "abstractauto";
}

impl From<Abstractauto> for u32 {
    fn from(register: Abstractauto) -> Self {
        register.0
    }
}

impl From<u32> for Abstractauto {
    fn from(value: u32) -> Self {
        Self(value)
    }
}

impl From<u32> for Sbcs {
    fn from(value: u32) -> Self {
        Self(value)
    }
}

bitfield! {
    /// Abstract command register, located at address 0x17
    /// This is not for all commands, only for the ones
    /// from the debug spec. (see 3.6.1.3)
    pub struct AccessMemoryCommand(u32);
    impl Debug;
    /// This is 2 to indicate Access Memory Command.
    _, set_cmd_type: 31, 24;
    /// An implementation does not have to implement
    /// both virtual and physical accesses, but it must
    /// fail accesses that it doesn’t support.

    /// 0: Addresses are physical (to the hart they are
    /// performed on).\
    /// 1: Addresses are virtual, and translated the way
    /// they would be from M-mode, with MPRV set.
    pub _, set_aamvirtual: 23;
    /// 0: Access the lowest 8 bits of the memory location.\
    /// 1: Access the lowest 16 bits of the memory location.\
    /// 2: Access the lowest 32 bits of the memory location.\
    /// 3: Access the lowest 64 bits of the memory location.\
    /// 4: Access the lowest 128 bits of the memory location.
    pub _, set_aamsize: 22,20;
    /// After a memory access has completed, if this bit
    /// is 1, increment arg1 (which contains the address
    /// used) by the number of bytes encoded in aamsize.
    pub _, set_aampostincrement: 19;
    /// 0: Copy data from the memory location specified
    /// in arg1 into arg0 portion of data.\
    /// 1: Copy data from arg0 portion of data into the
    /// memory location specified in arg1.
    pub _, set_write: 16;
    /// These bits are reserved for target-specific uses.
    pub _, set_target_specific: 15, 14;
}

impl DebugRegister for AccessMemoryCommand {
    const ADDRESS: u8 = 0x17;
    const NAME: &'static str = "command";
}

impl From<AccessMemoryCommand> for u32 {
    fn from(register: AccessMemoryCommand) -> Self {
        let mut reg = register;
        reg.set_cmd_type(2);
        reg.0
    }
}

impl From<u32> for AccessMemoryCommand {
    fn from(value: u32) -> Self {
        Self(value)
    }
}

data_register! { Sbaddress0, 0x39, "sbaddress0" }
data_register! { Sbaddress1, 0x3a, "sbaddress1" }
data_register! { Sbaddress2, 0x3b, "sbaddress2" }
data_register! { Sbaddress3, 0x37, "sbaddress3" }

data_register! { Sbdata0, 0x3c, "sbdata0" }
data_register! { Sbdata1, 0x3d, "sbdata1" }
data_register! { Sbdata2, 0x3e, "sbdata2" }
data_register! { Sbdata3, 0x3f, "sbdata3" }

data_register! { Confstrptr0, 0x19, "confstrptr0" }
data_register! { Confstrptr1, 0x1a, "confstrptr1" }
data_register! { Confstrptr2, 0x1b, "confstrptr2" }
data_register! { Confstrptr3, 0x1c, "confstrptr3" }
