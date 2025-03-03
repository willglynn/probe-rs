pub(crate) mod communication_interface;

use crate::{CoreType, InstructionSet};
pub use communication_interface::CommunicationInterface;
pub use probe_rs_target::{Architecture, CoreAccessOptions};

use crate::architecture::{
    arm::core::CortexAState, arm::core::CortexMState,
    riscv::communication_interface::RiscvCommunicationInterface,
};
use crate::error;
use crate::Target;
use crate::{Error, Memory, MemoryInterface};
use anyhow::{anyhow, Result};
use std::time::Duration;

/// A memory mapped register, for instance ARM debug registers (DHCSR, etc).
pub trait MemoryMappedRegister: Clone + From<u32> + Into<u32> + Sized + std::fmt::Debug {
    /// The register's address in the target memory.
    const ADDRESS: u64;
    /// The register's name.
    const NAME: &'static str;
}

/// An struct for storing the current state of a core.
#[derive(Debug, Clone)]
pub struct CoreInformation {
    /// The current Program Counter.
    pub pc: u64,
}

/// The type of data stored in a register
#[derive(Debug, Clone, PartialEq)]
pub enum RegisterDataType {
    /// Unsigned integer data
    UnsignedInteger,
    /// Floating point data
    FloatingPoint,
}

/// Describes a register with its properties.
#[derive(Debug, Clone, PartialEq)]
pub struct RegisterDescription {
    pub(crate) name: &'static str,
    pub(crate) _kind: RegisterKind,
    pub(crate) id: RegisterId,
    pub(crate) _type: RegisterDataType,
    pub(crate) size_in_bits: usize,
}

impl RegisterDescription {
    /// Get the display name of this register
    pub fn name(&self) -> &'static str {
        self.name
    }

    /// Get the type of data stored in this register
    pub fn data_type(&self) -> RegisterDataType {
        self._type.clone()
    }

    /// Get the size, in bits, of this register
    pub fn size_in_bits(&self) -> usize {
        self.size_in_bits
    }

    /// Get the size, in bytes, of this register
    pub fn size_in_bytes(&self) -> usize {
        // Always round up
        (self.size_in_bits + 7) / 8
    }

    /// Get the width to format this register as a hex string
    /// Assumes a format string like {:#0<width>x}
    pub fn format_hex_width(&self) -> usize {
        (self.size_in_bytes() * 2) + 2
    }
}

impl From<RegisterDescription> for RegisterId {
    fn from(description: RegisterDescription) -> RegisterId {
        description.id
    }
}

impl From<&RegisterDescription> for RegisterId {
    fn from(description: &RegisterDescription) -> RegisterId {
        description.id
    }
}

/// The location of a CPU \register. This is not an actual memory address, but a core specific location that represents a specific core register.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct RegisterId(pub u16);

impl From<RegisterId> for u32 {
    fn from(value: RegisterId) -> Self {
        u32::from(value.0)
    }
}

impl From<u16> for RegisterId {
    fn from(value: u16) -> Self {
        RegisterId(value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum RegisterKind {
    General,
    PC,
    Fp,
}

/// A value of a core register
///
/// Creating a new `RegisterValue` should be done using From or Into.
/// Converting a value back to a primitive type can be done with either
/// a match arm or TryInto
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RegisterValue {
    /// 32-bit unsigned integer
    U32(u32),
    /// 64-bit unsigned integer
    U64(u64),
}

impl From<u32> for RegisterValue {
    fn from(val: u32) -> Self {
        Self::U32(val)
    }
}

impl From<u64> for RegisterValue {
    fn from(val: u64) -> Self {
        Self::U64(val)
    }
}

impl TryInto<u32> for RegisterValue {
    type Error = crate::Error;

    fn try_into(self) -> Result<u32, Self::Error> {
        match self {
            Self::U32(v) => Ok(v),
            Self::U64(v) => v
                .try_into()
                .map_err(|_| crate::Error::Other(anyhow!("Value '{}' too large for u32", v))),
        }
    }
}

impl TryInto<u64> for RegisterValue {
    type Error = crate::Error;

    fn try_into(self) -> Result<u64, Self::Error> {
        match self {
            Self::U32(v) => Ok(v.into()),
            Self::U64(v) => Ok(v),
        }
    }
}

/// Register description for a core.
#[derive(Debug, PartialEq)]
pub struct RegisterFile {
    pub(crate) platform_registers: &'static [RegisterDescription],

    /// Register description for the program counter
    pub(crate) program_counter: &'static RegisterDescription,

    pub(crate) stack_pointer: &'static RegisterDescription,

    pub(crate) return_address: &'static RegisterDescription,

    pub(crate) frame_pointer: &'static RegisterDescription,

    pub(crate) argument_registers: &'static [RegisterDescription],

    pub(crate) result_registers: &'static [RegisterDescription],

    pub(crate) msp: Option<&'static RegisterDescription>,

    pub(crate) psp: Option<&'static RegisterDescription>,

    pub(crate) extra: Option<&'static RegisterDescription>,

    pub(crate) psr: Option<&'static RegisterDescription>,

    pub(crate) fp_status: Option<&'static RegisterDescription>,

    pub(crate) fp_registers: Option<&'static [RegisterDescription]>,
}

impl RegisterFile {
    /// Returns an iterator over the descriptions of all the registers of this core.
    pub fn registers(&self) -> impl Iterator<Item = &RegisterDescription> {
        self.platform_registers.iter()
    }

    /// The frame pointer.
    pub fn frame_pointer(&self) -> &RegisterDescription {
        self.frame_pointer
    }

    /// The program counter.
    pub fn program_counter(&self) -> &RegisterDescription {
        self.program_counter
    }

    /// The stack pointer.
    pub fn stack_pointer(&self) -> &RegisterDescription {
        self.stack_pointer
    }

    /// The link register.
    pub fn return_address(&self) -> &RegisterDescription {
        self.return_address
    }

    /// Returns the nth argument register.
    ///
    /// # Panics
    ///
    /// Panics if the register at given index does not exist.
    pub fn argument_register(&self, index: usize) -> &RegisterDescription {
        &self.argument_registers[index]
    }

    /// Returns the nth argument register if it is exists, `None` otherwise.
    pub fn get_argument_register(&self, index: usize) -> Option<&RegisterDescription> {
        self.argument_registers.get(index)
    }

    /// Returns the nth result register.
    ///
    /// # Panics
    ///
    /// Panics if the register at given index does not exist.
    pub fn result_register(&self, index: usize) -> &RegisterDescription {
        &self.result_registers[index]
    }

    /// Returns the nth result register if it is exists, `None` otherwise.
    pub fn get_result_register(&self, index: usize) -> Option<&RegisterDescription> {
        self.result_registers.get(index)
    }

    /// Returns the nth platform register.
    ///
    /// # Panics
    ///
    /// Panics if the register at given index does not exist.
    pub fn platform_register(&self, index: usize) -> &RegisterDescription {
        &self.platform_registers[index]
    }

    /// Returns the nth platform register if it is exists, `None` otherwise.
    pub fn get_platform_register(&self, index: usize) -> Option<&RegisterDescription> {
        self.platform_registers.get(index)
    }

    /// The main stack pointer.
    pub fn msp(&self) -> Option<&RegisterDescription> {
        self.msp
    }

    /// The process stack pointer.
    pub fn psp(&self) -> Option<&RegisterDescription> {
        self.psp
    }

    /// The processor status register.
    pub fn psr(&self) -> Option<&RegisterDescription> {
        self.psr
    }

    // ARM DDI 0403E.d (ID070218)
    // C1.6.3 Debug Core Register Selector Register, DCRSR
    // Bits[31:24] CONTROL.
    // Bits[23:16] FAULTMASK.
    // Bits[15:8]  BASEPRI.
    // Bits[7:0]   PRIMASK.
    // In each field, the valid bits are packed with leading zeros. For example,
    // // FAULTMASK is always a single bit, DCRDR[16], and DCRDR[23:17] is 0b0000000.
    // pub fn extra(&self) -> Option<&RegisterDescription> {
    //     self.extra
    // }

    /// The fpu status register.
    pub fn fpscr(&self) -> Option<&RegisterDescription> {
        self.fp_status
    }

    /// Returns an iterator over the descriptions of all the registers of this core.
    pub fn fpu_registers(&self) -> Option<impl Iterator<Item = &RegisterDescription>> {
        self.fp_registers.map(|r| r.iter())
    }

    /// Returns the nth fpu register.
    ///
    /// # Panics
    ///
    /// Panics if the register at given index does not exist.
    pub fn fpu_register(&self, index: usize) -> Option<&RegisterDescription> {
        self.fp_registers.map(|r| &r[index])
    }

    /// Returns the nth fpu register if it is exists, `None` otherwise.
    pub fn get_fpu_register(&self, index: usize) -> Option<&RegisterDescription> {
        self.fp_registers.map(|r| r.get(index)).flatten()
    }
}

/// A generic interface to control a MCU core.
pub trait CoreInterface: MemoryInterface {
    /// Wait until the core is halted. If the core does not halt on its own,
    /// a [`DebugProbeError::Timeout`](crate::DebugProbeError::Timeout) error will be returned.
    fn wait_for_core_halted(&mut self, timeout: Duration) -> Result<(), error::Error>;

    /// Check if the core is halted. If the core does not halt on its own,
    /// a [`DebugProbeError::Timeout`](crate::DebugProbeError::Timeout) error will be returned.
    fn core_halted(&mut self) -> Result<bool, error::Error>;

    /// Returns the current status of the core.
    fn status(&mut self) -> Result<CoreStatus, error::Error>;

    /// Try to halt the core. This function ensures the core is actually halted, and
    /// returns a [`DebugProbeError::Timeout`](crate::DebugProbeError::Timeout) otherwise.
    fn halt(&mut self, timeout: Duration) -> Result<CoreInformation, error::Error>;

    /// Continue to execute instructions.
    fn run(&mut self) -> Result<(), error::Error>;

    /// Reset the core, and then continue to execute instructions. If the core
    /// should be halted after reset, use the [`reset_and_halt`] function.
    ///
    /// [`reset_and_halt`]: Core::reset_and_halt
    fn reset(&mut self) -> Result<(), error::Error>;

    /// Reset the core, and then immediately halt. To continue execution after
    /// reset, use the [`reset`] function.
    ///
    /// [`reset`]: Core::reset
    fn reset_and_halt(&mut self, timeout: Duration) -> Result<CoreInformation, error::Error>;

    /// Steps one instruction and then enters halted state again.
    fn step(&mut self) -> Result<CoreInformation, error::Error>;

    /// Read the value of a core register.
    fn read_core_reg(&mut self, address: RegisterId) -> Result<RegisterValue, error::Error>;

    /// Write the value of a core register.
    fn write_core_reg(&mut self, address: RegisterId, value: RegisterValue) -> Result<()>;

    /// Returns all the available breakpoint units of the core.
    fn available_breakpoint_units(&mut self) -> Result<u32, error::Error>;

    /// Read the hardware breakpoints from FpComp registers, and adds them to the Result Vector.
    /// A value of None in any position of the Vector indicates that the position is unset/available.
    /// We intentionally return all breakpoints, irrespective of whether they are enabled or not.
    fn hw_breakpoints(&mut self) -> Result<Vec<Option<u64>>, error::Error>;

    /// Enables breakpoints on this core. If a breakpoint is set, it will halt as soon as it is hit.
    fn enable_breakpoints(&mut self, state: bool) -> Result<(), error::Error>;

    /// Sets a breakpoint at `addr`. It does so by using unit `bp_unit_index`.
    fn set_hw_breakpoint(&mut self, unit_index: usize, addr: u64) -> Result<(), error::Error>;

    /// Clears the breakpoint configured in unit `unit_index`.
    fn clear_hw_breakpoint(&mut self, unit_index: usize) -> Result<(), error::Error>;

    /// Returns a list of all the registers of this core.
    fn registers(&self) -> &'static RegisterFile;

    /// Returns `true` if hwardware breakpoints are enabled, `false` otherwise.
    fn hw_breakpoints_enabled(&self) -> bool;

    /// Get the `Architecture` of the Core.
    fn architecture(&self) -> Architecture;

    /// Get the `CoreType` of the Core
    fn core_type(&self) -> CoreType;

    /// Determine the instruction set the core is operating in
    /// This must be queried while halted as this is a runtime
    /// decision for some core types
    fn instruction_set(&mut self) -> Result<InstructionSet, error::Error>;

    /// Determine if an FPU is present.
    /// This must be queried while halted as this is a runtime
    /// decision for some core types.
    fn fpu_support(&mut self) -> Result<bool, error::Error>;
}

impl<'probe> MemoryInterface for Core<'probe> {
    fn supports_native_64bit_access(&mut self) -> bool {
        self.inner.supports_native_64bit_access()
    }

    fn read_word_64(&mut self, address: u64) -> Result<u64, Error> {
        self.inner.read_word_64(address)
    }

    fn read_word_32(&mut self, address: u64) -> Result<u32, Error> {
        self.inner.read_word_32(address)
    }

    fn read_word_8(&mut self, address: u64) -> Result<u8, Error> {
        self.inner.read_word_8(address)
    }

    fn read_64(&mut self, address: u64, data: &mut [u64]) -> Result<(), Error> {
        self.inner.read_64(address, data)
    }

    fn read_32(&mut self, address: u64, data: &mut [u32]) -> Result<(), Error> {
        self.inner.read_32(address, data)
    }

    fn read_8(&mut self, address: u64, data: &mut [u8]) -> Result<(), Error> {
        self.inner.read_8(address, data)
    }

    fn write_word_64(&mut self, addr: u64, data: u64) -> Result<(), Error> {
        self.inner.write_word_64(addr, data)
    }

    fn write_word_32(&mut self, addr: u64, data: u32) -> Result<(), Error> {
        self.inner.write_word_32(addr, data)
    }

    fn write_word_8(&mut self, addr: u64, data: u8) -> Result<(), Error> {
        self.inner.write_word_8(addr, data)
    }

    fn write_64(&mut self, addr: u64, data: &[u64]) -> Result<(), Error> {
        self.inner.write_64(addr, data)
    }

    fn write_32(&mut self, addr: u64, data: &[u32]) -> Result<(), Error> {
        self.inner.write_32(addr, data)
    }

    fn write_8(&mut self, addr: u64, data: &[u8]) -> Result<(), Error> {
        self.inner.write_8(addr, data)
    }

    fn flush(&mut self) -> Result<(), Error> {
        self.inner.flush()
    }
}

/// A generic core state which caches the generic parts of the core state.
#[derive(Debug)]
pub struct CoreState {
    id: usize,

    /// Information needed to access the core
    core_access_options: CoreAccessOptions,
}

impl CoreState {
    /// Creates a new core state from the core ID.
    pub fn new(id: usize, core_access_options: CoreAccessOptions) -> Self {
        Self {
            id,
            core_access_options,
        }
    }

    /// Returns the core ID.

    pub fn id(&self) -> usize {
        self.id
    }
}

/// The architecture specific core state.
#[derive(Debug)]
pub enum SpecificCoreState {
    /// The state of an ARMv6-M core.
    Armv6m(CortexMState),
    /// The state of an ARMv7-A core.
    Armv7a(CortexAState),
    /// The state of an ARMv7-M core.
    Armv7m(CortexMState),
    /// The state of an ARMv7-EM core.
    Armv7em(CortexMState),
    /// The state of an ARMv8-A core.
    Armv8a(CortexAState),
    /// The state of an ARMv8-M core.
    Armv8m(CortexMState),
    /// The state of an RISC-V core.
    Riscv,
}

impl SpecificCoreState {
    pub(crate) fn from_core_type(typ: CoreType) -> Self {
        match typ {
            CoreType::Armv6m => SpecificCoreState::Armv6m(CortexMState::new()),
            CoreType::Armv7a => SpecificCoreState::Armv7a(CortexAState::new()),
            CoreType::Armv7m => SpecificCoreState::Armv7m(CortexMState::new()),
            CoreType::Armv7em => SpecificCoreState::Armv7m(CortexMState::new()),
            CoreType::Armv8a => SpecificCoreState::Armv8a(CortexAState::new()),
            CoreType::Armv8m => SpecificCoreState::Armv8m(CortexMState::new()),
            CoreType::Riscv => SpecificCoreState::Riscv,
        }
    }

    pub(crate) fn core_type(&self) -> CoreType {
        match self {
            SpecificCoreState::Armv6m(_) => CoreType::Armv6m,
            SpecificCoreState::Armv7a(_) => CoreType::Armv7a,
            SpecificCoreState::Armv7m(_) => CoreType::Armv7m,
            SpecificCoreState::Armv7em(_) => CoreType::Armv7em,
            SpecificCoreState::Armv8a(_) => CoreType::Armv8a,
            SpecificCoreState::Armv8m(_) => CoreType::Armv8m,
            SpecificCoreState::Riscv => CoreType::Riscv,
        }
    }

    pub(crate) fn attach_arm<'probe, 'target: 'probe>(
        &'probe mut self,
        state: &'probe mut CoreState,
        memory: Memory<'probe>,
        target: &'target Target,
    ) -> Result<Core<'probe>, Error> {
        let debug_sequence = match &target.debug_sequence {
            crate::config::DebugSequence::Arm(sequence) => sequence.clone(),
            crate::config::DebugSequence::Riscv(_) => {
                return Err(Error::UnableToOpenProbe(
                    "Core architecture and Probe mismatch.",
                ))
            }
        };

        let options = match &state.core_access_options {
            CoreAccessOptions::Arm(options) => options,
            CoreAccessOptions::Riscv(_) => {
                return Err(Error::UnableToOpenProbe(
                    "Core architecture and Probe mismatch.",
                ))
            }
        };

        Ok(match self {
            SpecificCoreState::Armv6m(s) => Core::new(
                crate::architecture::arm::armv6m::Armv6m::new(memory, s, debug_sequence)?,
                state,
            ),
            SpecificCoreState::Armv7a(s) => Core::new(
                crate::architecture::arm::armv7a::Armv7a::new(
                    memory,
                    s,
                    options.debug_base.expect("base_address not specified"),
                    debug_sequence,
                )?,
                state,
            ),
            SpecificCoreState::Armv7m(s) | SpecificCoreState::Armv7em(s) => Core::new(
                crate::architecture::arm::armv7m::Armv7m::new(memory, s, debug_sequence)?,
                state,
            ),
            SpecificCoreState::Armv8a(s) => Core::new(
                crate::architecture::arm::armv8a::Armv8a::new(
                    memory,
                    s,
                    options.debug_base.expect("base_address not specified"),
                    options.cti_base.expect("cti_address not specified"),
                    debug_sequence,
                )?,
                state,
            ),
            SpecificCoreState::Armv8m(s) => Core::new(
                crate::architecture::arm::armv8m::Armv8m::new(memory, s, debug_sequence)?,
                state,
            ),
            _ => {
                return Err(Error::UnableToOpenProbe(
                    "Core architecture and Probe mismatch.",
                ))
            }
        })
    }

    pub(crate) fn attach_riscv<'probe>(
        &self,
        state: &'probe mut CoreState,
        interface: &'probe mut RiscvCommunicationInterface,
    ) -> Result<Core<'probe>, Error> {
        Ok(match self {
            SpecificCoreState::Riscv => {
                Core::new(crate::architecture::riscv::Riscv32::new(interface), state)
            }
            _ => {
                return Err(Error::UnableToOpenProbe(
                    "Core architecture and Probe mismatch.",
                ))
            }
        })
    }
}

/// Generic core handle representing a physical core on an MCU.
///
/// This should be considere as a temporary view of the core which locks the debug probe driver to as single consumer by borrowing it.
///
/// As soon as you did your atomic task (e.g. halt the core, read the core state and all other debug relevant info) you should drop this object,
/// to allow potential other shareholders of the session struct to grab a core handle too.
pub struct Core<'probe> {
    inner: Box<dyn CoreInterface + 'probe>,
    state: &'probe mut CoreState,
}

impl<'probe> Core<'probe> {
    /// Create a new [`Core`].
    pub fn new(core: impl CoreInterface + 'probe, state: &'probe mut CoreState) -> Core<'probe> {
        Self {
            inner: Box::new(core),
            state,
        }
    }

    /// Creates a new [`CoreState`]
    pub fn create_state(id: usize, options: CoreAccessOptions) -> CoreState {
        CoreState::new(id, options)
    }

    /// Returns the ID of this core.
    pub fn id(&self) -> usize {
        self.state.id
    }

    /// Wait until the core is halted. If the core does not halt on its own,
    /// a [`DebugProbeError::Timeout`](crate::DebugProbeError::Timeout) error will be returned.
    pub fn wait_for_core_halted(&mut self, timeout: Duration) -> Result<(), error::Error> {
        self.inner.wait_for_core_halted(timeout)
    }

    /// Check if the core is halted. If the core does not halt on its own,
    /// a [`DebugProbeError::Timeout`](crate::DebugProbeError::Timeout) error will be returned.
    pub fn core_halted(&mut self) -> Result<bool, error::Error> {
        self.inner.core_halted()
    }

    /// Try to halt the core. This function ensures the core is actually halted, and
    /// returns a [`DebugProbeError::Timeout`](crate::DebugProbeError::Timeout) otherwise.
    pub fn halt(&mut self, timeout: Duration) -> Result<CoreInformation, error::Error> {
        self.inner.halt(timeout)
    }

    /// Continue to execute instructions.
    pub fn run(&mut self) -> Result<(), error::Error> {
        self.inner.run()
    }

    /// Reset the core, and then continue to execute instructions. If the core
    /// should be halted after reset, use the [`reset_and_halt`] function.
    ///
    /// [`reset_and_halt`]: Core::reset_and_halt
    pub fn reset(&mut self) -> Result<(), error::Error> {
        self.inner.reset()
    }

    /// Reset the core, and then immediately halt. To continue execution after
    /// reset, use the [`reset`] function.
    ///
    /// [`reset`]: Core::reset
    pub fn reset_and_halt(&mut self, timeout: Duration) -> Result<CoreInformation, error::Error> {
        self.inner.reset_and_halt(timeout)
    }

    /// Steps one instruction and then enters halted state again.
    pub fn step(&mut self) -> Result<CoreInformation, error::Error> {
        self.inner.step()
    }

    /// Returns the current status of the core.
    pub fn status(&mut self) -> Result<CoreStatus, error::Error> {
        self.inner.status()
    }

    /// Read the value of a core register.
    ///
    /// # Errors
    ///
    /// If `T` isn't large enough to hold the register value an error will be raised.
    pub fn read_core_reg<T>(&mut self, address: impl Into<RegisterId>) -> Result<T, error::Error>
    where
        RegisterValue: TryInto<T, Error = error::Error>,
    {
        let value = self.inner.read_core_reg(address.into())?;

        value.try_into()
    }

    /// Write the value of a core register.
    ///
    /// # Errors
    ///
    /// If T is too large to write to the target register an error will be raised.
    pub fn write_core_reg<T>(&mut self, address: RegisterId, value: T) -> Result<(), error::Error>
    where
        T: Into<RegisterValue>,
    {
        Ok(self.inner.write_core_reg(address, value.into())?)
    }

    /// Returns all the available breakpoint units of the core.
    pub fn available_breakpoint_units(&mut self) -> Result<u32, error::Error> {
        self.inner.available_breakpoint_units()
    }

    /// Enables breakpoints on this core. If a breakpoint is set, it will halt as soon as it is hit.
    fn enable_breakpoints(&mut self, state: bool) -> Result<(), error::Error> {
        self.inner.enable_breakpoints(state)
    }

    /// Returns a list of all the registers of this core.
    pub fn registers(&self) -> &'static RegisterFile {
        self.inner.registers()
    }

    /// Find the index of the next available HW breakpoint comparator.
    fn find_free_breakpoint_comparator_index(&mut self) -> Result<usize, error::Error> {
        let mut next_available_hw_breakpoint = 0;
        for breakpoint in self.inner.hw_breakpoints()? {
            if breakpoint.is_none() {
                return Ok(next_available_hw_breakpoint);
            } else {
                next_available_hw_breakpoint += 1;
            }
        }
        Err(error::Error::Other(anyhow!(
            "No available hardware breakpoints"
        )))
    }

    /// Set a hardware breakpoint
    ///
    /// This function will try to set a hardware breakpoint att `address`.
    ///
    /// The amount of hardware breakpoints which are supported is chip specific,
    /// and can be queried using the `get_available_breakpoint_units` function.
    pub fn set_hw_breakpoint(&mut self, address: u64) -> Result<(), error::Error> {
        if !self.inner.hw_breakpoints_enabled() {
            self.enable_breakpoints(true)?;
        }

        // If there is a breakpoint set already, return its bp_unit_index, else find the next free index.
        let breakpoint_comparator_index = match self
            .inner
            .hw_breakpoints()?
            .iter()
            .position(|&bp| bp == Some(address))
        {
            Some(breakpoint_comparator_index) => breakpoint_comparator_index,
            None => self.find_free_breakpoint_comparator_index()?,
        };

        log::debug!(
            "Trying to set HW breakpoint #{} with comparator address  {:#08x}",
            breakpoint_comparator_index,
            address
        );

        // Actually set the breakpoint. Even if it has been set, set it again so it will be active.
        self.inner
            .set_hw_breakpoint(breakpoint_comparator_index, address)?;
        Ok(())
    }

    /// Set a hardware breakpoint
    ///
    /// This function will try to clear a hardware breakpoint at `address` if there exists a breakpoint at that address.
    pub fn clear_hw_breakpoint(&mut self, address: u64) -> Result<(), error::Error> {
        let bp_position = self
            .inner
            .hw_breakpoints()?
            .iter()
            .position(|bp| bp.is_some() && bp.unwrap() == address);

        log::debug!(
            "Will clear HW breakpoint    #{} with comparator address    {:#08x}",
            bp_position.unwrap_or(usize::MAX),
            address
        );

        match bp_position {
            Some(bp_position) => {
                self.inner.clear_hw_breakpoint(bp_position)?;
                Ok(())
            }
            None => Err(error::Error::Other(anyhow!(
                "No breakpoint found at address {:#010x}",
                address
            ))),
        }
    }

    /// Clear all hardware breakpoints
    ///
    /// This function will clear all HW breakpoints which are configured on the target,
    /// regardless if they are set by probe-rs, AND regardless if they are enabled or not.
    /// Also used as a helper function in [`Session::drop`](crate::session::Session).
    pub fn clear_all_hw_breakpoints(&mut self) -> Result<(), error::Error> {
        for breakpoint in (self.inner.hw_breakpoints()?).into_iter().flatten() {
            self.clear_hw_breakpoint(breakpoint)?
        }
        Ok(())
    }

    /// Returns the architecture of the core.
    pub fn architecture(&self) -> Architecture {
        self.inner.architecture()
    }

    /// Returns the core type of the core
    pub fn core_type(&self) -> CoreType {
        self.inner.core_type()
    }

    /// Determine the instruction set the core is operating in
    /// This must be queried while halted as this is a runtime
    /// decision for some core types
    pub fn instruction_set(&mut self) -> Result<InstructionSet, error::Error> {
        self.inner.instruction_set()
    }

    /// Determine if an FPU is present.
    /// This must be queried while halted as this is a runtime
    /// decision for some core types.
    pub fn fpu_support(&mut self) -> Result<bool, error::Error> {
        self.inner.fpu_support()
    }
}

/// The id of a breakpoint.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct BreakpointId(usize);

impl BreakpointId {
    /// Creates a new breakpoint ID from an `usize`.
    pub fn new(id: usize) -> Self {
        BreakpointId(id)
    }
}

/// The status of the core.
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum CoreStatus {
    /// The core is currently running.
    Running,
    /// The core is currently halted. This also specifies the reason as a payload.
    Halted(HaltReason),
    /// This is a Cortex-M specific status, and will not be set or handled by RISCV code.
    LockedUp,
    /// The core is currently sleeping.
    Sleeping,
    /// The core state is currently unknown. This is always the case when the core is first created.
    Unknown,
}

impl CoreStatus {
    /// Returns `true` if the core is currently halted.
    pub fn is_halted(&self) -> bool {
        matches!(self, CoreStatus::Halted(_))
    }
}

/// The reason why a core was halted.
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum HaltReason {
    /// Multiple reasons for a halt.
    ///
    /// This can happen for example when a single instruction
    /// step ends up on a breakpoint, after which both breakpoint and step / request
    /// are set.
    Multiple,
    /// Core halted due to a breakpoint, either
    /// a *soft* or a *hard* breakpoint.
    Breakpoint,
    /// Core halted due to an exception, e.g. an
    /// an interrupt.
    Exception,
    /// Core halted due to a data watchpoint
    Watchpoint,
    /// Core halted after single step
    Step,
    /// Core halted because of a debugger request
    Request,
    /// External halt request
    External,
    /// Unknown reason for halt.
    ///
    /// This can happen for example when the core is already halted when we connect.
    Unknown,
}
